//! # Dual IIR, low latency
//!
//! A variant of [Dual IIR](../dual_iir/index.html) with the same settings and stream
//! format, processing each sample individually to reduce input-to-output latency.
//!
//! ## Sample processing
//!
//! The sampling timer starts the ADC conversions. Once ADC1's result is available, its SPI
//! interrupt reads both ADC receive FIFOs and writes each DAC code directly to its SPI transmit
//! FIFO as soon as it is computed. Channel 0 is processed first and has the lower latency.
//! The DAC updates follow the processing, independently of the sampling timer.
//!
//! Each biquad needs only a multiplication, an addition and a clamp between input and output.
//! [`stabilizer::dual_iir::Stage`] precomputes the remaining terms after the previous sample.
//! Signal sources advance after the DAC writes; constant sources are not iterated. A lower
//! priority task assembles stream frames and samples the digital inputs every eight samples.
//! `External` run/hold therefore responds at batch granularity.
//!
//! ## Timing and limits
//!
//! The default sample period is 128 timer ticks (1.28 µs, 781.25 kHz), recommended for CPU
//! headroom. In Rev 1.3 measurements, the two-channel build also ran at 112 and 100 ticks
//! without late samples, with idle-source stream losses of about 5% and 40%, respectively.
//! A simplified single-channel build ran at 84 ticks; 80 ticks produced ADC SPI errors and
//! 64 silently halved the sampling rate. The exact ADC readout limit remains unresolved.
//! The compile-time floor of 100 ticks is a conservative margin from these readout failures.
//! Non-power-of-two periods are allowed. The `process` and `late` telemetry cannot detect
//! skipped ADC triggers.
//!
//! `dual-iir` introduces two batch periods of buffering delay (20.48 µs at the default rate).
//! Here, the sampling-to-DAC update time consists of ADC conversion/readout, interrupt entry,
//! computation and DAC SPI transmission. With DAC-to-ADC loopbacks on a Rev 1.3 board at the
//! default rate, measured loop delays were 3.34 µs (channel 0) and 3.56 µs (channel 1), against
//! about 23 µs for batched `dual-iir`. These include the analog filters and zero-order hold.
//!
//! Processing should finish within one sample period. An active signal source adds CPU load;
//! both sources active caused about 20–28% stream loss on the test board at the default rate,
//! while the control loop kept up. Check stream loss and network responsiveness under the
//! intended load when changing the period or sources.
//!
//! Samples arriving before processing completes wait in the ADC receive FIFOs and contribute
//! to `late`; FIFO overflow panics with an ADC SPI error. Settings updates briefly mask the
//! processing interrupt and can also cause late samples. Library critical sections allow it
//! to preempt USB and SysTick (see [`stabilizer::hardware::basepri_critical_section`]).
//!
//! `process` reports mean and maximum handler execution time; `output` reports the maximum
//! time from handler start to the last DAC write. Both exclude interrupt entry, exit and
//! floating-point context stacking. Use them to assess CPU headroom alongside the stream
//! and network checks above.
//!
//! ## Interfaces
//!
//! See [DualIir] for settings and [Telemetry] for telemetry. The UDP [stream] contains raw ADC
//! and DAC codes in batches of eight samples, as in `dual-iir`.
//!
//! Both variants use the MQTT prefix `dt/sinara/dual-iir/<id>/` and share retained settings.
//! The retained `meta` message identifies the binary in `application`: `dual-iir-ll` here,
//! or `dual-iir` for the batched variant. USB `platform service` reports the same identity.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use core::sync::atomic::{AtomicBool, Ordering};

use miniconf::Tree;

use idsp::iir::pid::Units;

use platform::{AppSettings, NetSettings};
use serde::Serialize;
use signal_generator::Source;
use stabilizer::{
    convert::{AdcCode, DacCode},
    dual_iir::{Biquad, Channel, DualIir, IIR_CASCADE_LENGTH, Run, Stage},
    telemetry::{CPU_CYCLE, PounderTelemetry, Timing},
};

// The number of samples in each streamed batch
const BATCH_SIZE: usize = 8;

// Timer ticks at 100 MHz: 128 ticks give 781.25 kHz with more CPU headroom than 100 ticks.
// The 100-tick floor is a conservative margin from readout failures observed around 80 ticks;
// the usable rate also depends on CPU load. `process` and `late` miss skipped ADC triggers.
// The shadow sampling timer has a 16-bit period. Non-power-of-two periods are supported.
const SAMPLE_TICKS: u32 = 128;
const _: () = assert!(
    SAMPLE_TICKS >= 100 && SAMPLE_TICKS <= u16::MAX as u32,
    "dual-iir-ll requires 100..=65535 timer ticks per sample"
);
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * stabilizer::design_parameters::TIMER_PERIOD;

const UNITS: Units<f32> = Units {
    t: SAMPLE_PERIOD,
    x: AdcCode::VOLT_PER_LSB,
    y: DacCode::VOLT_PER_LSB,
};

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    dual_iir: DualIir,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            dual_iir: DualIir::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            dual_iir: DualIir::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

/// The run-time state of one channel.
#[derive(Clone, Debug)]
pub struct Active {
    run: Run,
    biquad: [Biquad; IIR_CASCADE_LENGTH],
    stage: [Stage; IIR_CASCADE_LENGTH],
    /// The signal source. `None` if its output is constant.
    source: Option<Source>,
    /// The signal source contribution to the next output sample in DAC LSB.
    offset: i16,
}

/// The input and the outputs of the filter stages for one sample.
///
/// They are kept from [`Active::process()`] to [`Active::update()`].
pub struct Sample {
    x: f32,
    y: [f32; IIR_CASCADE_LENGTH],
}

impl Active {
    fn new(ch: &Channel) -> Self {
        let mut active = Self {
            source: None,
            offset: 0,
            stage: Default::default(),
            run: ch.run,
            biquad: ch.biquad.each_ref().map(|biquad| biquad.build(&UNITS)),
        };
        active.set_filter(active.run, active.biquad.clone());
        active.set_source(
            ch.source
                .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip())
                .unwrap(),
        );
        active
    }

    /// Replace the filter. The transfer is bumpless.
    fn set_filter(&mut self, run: Run, biquad: [Biquad; IIR_CASCADE_LENGTH]) {
        self.run = run;
        self.biquad = biquad;
        for (stage, biquad) in self.stage.iter_mut().zip(&self.biquad) {
            stage.prepare(biquad);
        }
    }

    /// Replace the signal source.
    ///
    /// Sources with constant output are not iterated.
    fn set_source(&mut self, source: Source) {
        if let Some(offset) = source.constant() {
            self.offset = (offset >> 16) as _;
            self.source = None;
        } else {
            self.source = Some(source);
        }
    }

    /// Filter one ADC sample and add the signal source contribution.
    ///
    /// This is the latency critical part. It only uses what [`update()`](Self::update)
    /// precomputed after the previous sample.
    ///
    /// # Args
    /// * `di` - The digital input state for run/hold control.
    /// * `adc` - The ADC code of the input sample.
    ///
    /// # Returns
    /// The DAC code for the output sample and the data for [`update()`](Self::update).
    #[inline(always)]
    fn process(&self, di: &AtomicBool, adc: u16) -> (u16, Sample) {
        let run = match self.run {
            Run::Run => true,
            Run::Hold => false,
            Run::External => di.load(Ordering::Relaxed),
        };
        let x = f32::from(adc as i16);
        let mut y = [0.0; IIR_CASCADE_LENGTH];
        let mut v = x;
        for ((biquad, stage), y) in
            self.biquad.iter().zip(&self.stage).zip(&mut y)
        {
            v = if run {
                stage.output(biquad, v)
            } else {
                stage.hold()
            };
            *y = v;
        }

        // Note(unsafe): The filter limits must ensure that the value is in range.
        // The truncation introduces 1/2 LSB distortion.
        let v: i16 = unsafe { v.to_int_unchecked() };
        (
            DacCode::from(v.saturating_add(self.offset)).0,
            Sample { x, y },
        )
    }

    /// Update the filter state with the sample and prepare for the next one.
    #[inline(always)]
    fn update(&mut self, sample: Sample) {
        let mut x = sample.x;
        for ((biquad, stage), y) in
            self.biquad.iter().zip(&mut self.stage).zip(sample.y)
        {
            stage.update(biquad, x, y);
            x = y;
        }
    }

    /// Advance the signal source for the next sample.
    #[inline(always)]
    fn update_source(&mut self) {
        if let Some(source) = &mut self.source {
            self.offset = next(source);
        }
    }
}

/// The next signal source sample in DAC LSB.
///
/// This is kept out of line and next to the sample processing routine in ITCM: there is a
/// single copy of the generator code for both channels and it only runs for active sources.
#[inline(never)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".itcm.process.source"))]
fn next(source: &mut Source) -> i16 {
    (source.next().unwrap() >> 16) as _
}

/// Execution time statistics of the sample processing routine in CPU cycles.
#[derive(Clone, Copy, Default)]
pub struct Times {
    /// Maximum time until the last DAC code has been written
    output_max: u32,
    /// Maximum duration of the routine
    process_max: u32,
    /// Sum of the durations of the routine
    process_sum: u64,
    /// Number of samples that were already pending when the previous one was done
    late: u32,
}

impl Times {
    #[inline(always)]
    fn update(&mut self, output: u32, process: u32, late: bool) {
        self.output_max = self.output_max.max(output);
        self.process_max = self.process_max.max(process);
        self.process_sum = self.process_sum.wrapping_add(process as _);
        self.late = self.late.wrapping_add(late as _);
    }
}

/// The state of the sample processing routine.
///
/// It is shared with the settings update and telemetry tasks.
pub struct State {
    ch: [Active; 2],
    times: Times,
}

/// The telemetry reported by this application.
#[derive(Serialize)]
pub struct Telemetry {
    /// Most recent input voltage measurement.
    pub adcs: [f32; 2],
    /// Most recent output voltage.
    pub dacs: [f32; 2],
    /// Most recent digital input assertion state.
    pub digital_inputs: [bool; 2],
    /// The CPU temperature in degrees Celsius.
    pub cpu_temp: f32,
    /// Execution time of the sample processing routine. Excludes interrupt entry and exit.
    pub process: Timing,
    /// Maximum time in seconds from the start of the sample processing routine until the
    /// last DAC code has been written.
    pub output: f32,
    /// Number of samples since the previous report that arrived before the processing of the
    /// preceding sample was complete. They are processed with increased latency, not lost.
    pub late: u32,
    /// Pounder measurements, if applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pounder: Option<PounderTelemetry>,
}

#[cfg(not(target_os = "none"))]
fn main() {
    use miniconf::{json::to_json_value, json_schema::TreeJsonSchema};
    let s = Settings::default();
    println!(
        "{}",
        serde_json::to_string_pretty(&to_json_value(&s).unwrap()).unwrap()
    );
    let mut schema = TreeJsonSchema::new(Some(&s)).unwrap();
    schema
        .root
        .insert("title".to_string(), "Stabilizer dual-iir-ll".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC]))]
mod app {
    use super::*;
    use cortex_m::peripheral::DWT;
    use fugit::ExtU32 as _;
    use rtic_monotonics::Monotonic;

    use stabilizer::{
        hardware::{
            self, DigitalInput0, DigitalInput1, Pgia, SerialTerminal,
            SystemTimer, Systick, UsbDevice,
            adc::{Adc0Input, Adc1Input},
            dac::{Dac0Output, Dac1Output},
            hal,
            net::{NetworkState, NetworkUsers},
            setup::{Mezzanine, Pounder},
            timers::SamplingTimer,
        },
        pounder_config::ClockConfig,
        telemetry::TelemetryBuffer,
    };
    use stream::{FrameGenerator, Pop, SampleRing};

    // The most recent samples, handed from `process` to `streamer` and `telemetry`.
    static RING: SampleRing<512> = SampleRing::new();

    // The digital input states, sampled by `streamer` once per batch.
    static DI: [AtomicBool; 2] =
        [AtomicBool::new(false), AtomicBool::new(false)];

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<DualIir>,
        settings: Settings,
        state: State,
        pounder: Option<Pounder>,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: [Pgia; 2],
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        dds_clock: Option<ClockConfig>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (mut stabilizer, mezzanine, _eem) =
            hardware::setup::setup::<Settings>(
                c.core,
                c.device,
                clock,
                env!("CARGO_BIN_NAME"),
                hardware::Sampling::LowLatency,
                SAMPLE_TICKS,
            );

        let pounder = match mezzanine {
            Mezzanine::Pounder(pounder) => Some(pounder),
            Mezzanine::CurrentSense(_) => None,
        };
        if pounder.is_some() {
            stabilizer
                .settings
                .dual_iir
                .pounder
                .get_or_insert_with(Default::default);
        }
        // The DDS clock is initially configured to the default by `setup()`.
        let dds_clock = pounder.as_ref().map(|_| ClockConfig::default());

        let mut network = NetworkUsers::new(
            stabilizer.network_devices.stack,
            stabilizer.network_devices.phy,
            clock,
            "dual-iir",
            &stabilizer.settings.net,
            stabilizer.metadata,
        );

        let generator = network.configure_streaming(stream::Format::AdcDacData);

        let shared = Shared {
            usb: stabilizer.usb,
            network,
            state: State {
                ch: stabilizer.settings.dual_iir.ch.each_ref().map(Active::new),
                times: Default::default(),
            },
            settings: stabilizer.settings,
            pounder,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.sampling_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            dds_clock,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
        start::spawn().unwrap();

        (shared, local)
    }

    #[task(priority = 1, local=[sampling_timer])]
    async fn start(c: start::Context) {
        Systick::delay(100.millis()).await;
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// Runs once per sample, as soon as the ADC1 sample is available in its SPI receive FIFO.
    /// ADC0 is sampled just before ADC1. The DAC codes are written to the DACs immediately after
    /// they have been computed. Everything that is not needed to compute them comes after or
    /// is left to lower priority tasks.
    ///
    /// # Note
    /// Processing should complete within one sample period. Samples that arrive earlier wait
    /// in the ADC SPI receive FIFOs and are counted as late. If those FIFOs overflow, firmware
    /// will panic due to an ADC SPI error.
    #[task(binds = SPI3, local = [adcs, dacs], shared = [state], priority = 3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(mut c: process::Context) {
        let start = DWT::cycle_count();

        let (adc0, adc1) = c.local.adcs;
        let (dac0, dac1) = c.local.dacs;

        c.shared.state.lock(|state| {
            let State {
                ch: [ch0, ch1],
                times,
            } = state;

            // The latency critical part: write each DAC code as soon as it is available.
            // ADC0 is sampled before ADC1, whose interrupt this is.
            let adc0 = adc0.read();
            let (dac0_code, sample0) = ch0.process(&DI[0], adc0);
            dac0.write(dac0_code);
            let adc1_code = adc1.read();
            let (dac1_code, sample1) = ch1.process(&DI[1], adc1_code);
            dac1.write(dac1_code);
            let output = DWT::cycle_count();

            // Prepare the filters for the next sample.
            ch0.update(sample0);
            ch1.update(sample1);

            // Hand the sample to the stream task and wake it once per batch.
            let count = RING.push([adc0, adc1_code, dac0_code, dac1_code]);
            if count.is_multiple_of(BATCH_SIZE as u32) {
                rtic::pend(hal::stm32::Interrupt::LTDC);
            }

            ch0.update_source();
            ch1.update_source();

            // Detect SPI errors and backlog, e.g. after a settings update.
            let late = adc1.check();

            times.update(
                output.wrapping_sub(start),
                DWT::cycle_count().wrapping_sub(start),
                late,
            );
        });
    }

    /// Assemble stream frames from the samples handed over by `process` and sample the
    /// digital inputs.
    ///
    /// Pended by `process` once per batch of samples. This keeps the frame generation
    /// (allocation, header, copy, queueing) and the GPIO accesses out of the sample processing.
    ///
    /// LTDC is a spare vector pended in software; the display peripheral is unused.
    /// A direct binding avoids async executor overhead. It runs below DSP and above
    /// network/USB. Wakeups can coalesce because this task drains the sample ring.
    #[task(binds = LTDC, local = [digital_inputs, generator, tail: u32 = 0], priority = 2)]
    fn streamer(c: streamer::Context) {
        let streamer::LocalResources {
            digital_inputs,
            generator,
            tail,
            ..
        } = c.local;

        DI[0].store(digital_inputs.0.is_high(), Ordering::Relaxed);
        DI[1].store(digital_inputs.1.is_high(), Ordering::Relaxed);

        loop {
            match RING.pop::<BATCH_SIZE>(tail) {
                Pop::Empty => break,
                Pop::Batch(batch) => generator.add(|buf| {
                    let data = bytemuck::cast_slice(&batch);
                    buf[..data.len()].copy_from_slice(data);
                    data.len()
                }),
                Pop::Lost(batches) => generator.skip(batches),
            }
        }
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.dual_iir))
            {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap();
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    // We can't sleep if USB is not in suspend.
                    if c.shared.usb.lock(|usb| {
                        usb.state()
                            == usb_device::device::UsbDeviceState::Suspend
                    }) {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }

    #[task(priority = 1, local=[afes, dds_clock], shared=[network, settings, state, pounder])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.local.afes[0].set_gain(settings.dual_iir.ch[0].gain);
            c.local.afes[1].set_gain(settings.dual_iir.ch[1].gain);

            if settings.dual_iir.trigger {
                settings.dual_iir.trigger = false;
                let s = settings.dual_iir.ch.each_ref().map(|ch| {
                    let s = ch
                        .source
                        .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip());
                    if let Err(err) = &s {
                        log::error!("Failed to update source: {:?}", err);
                    }
                    s
                });
                c.shared.state.lock(|state| {
                    for (ch, s) in state.ch.iter_mut().zip(s) {
                        if let Ok(s) = s {
                            ch.set_source(s);
                        }
                    }
                });
            }
            let b = settings.dual_iir.ch.each_ref().map(|ch| {
                (ch.run, ch.biquad.each_ref().map(|b| b.build(&UNITS)))
            });
            c.shared.state.lock(|state| {
                for (a, (run, biquad)) in state.ch.iter_mut().zip(b) {
                    a.set_filter(run, biquad);
                }
            });
            c.shared.pounder.lock(|pounder| {
                if let (Some(pounder), Some(config)) =
                    (pounder, &settings.dual_iir.pounder)
                {
                    pounder.update_dds(config, c.local.dds_clock);
                }
            });
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.dual_iir.stream));
        });
    }

    #[task(priority = 1, shared=[network, settings, state, pounder], local=[cpu_temp_sensor, count: u32 = 0])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            // Take the timing statistics accumulated since the previous report and the
            // number of samples they cover.
            let (times, head) = c
                .shared
                .state
                .lock(|state| (core::mem::take(&mut state.times), RING.head()));
            let count = head.wrapping_sub(*c.local.count);
            *c.local.count = head;

            let [adc0, adc1, dac0, dac1] = RING.latest();
            let telemetry = TelemetryBuffer {
                adcs: [AdcCode(adc0), AdcCode(adc1)],
                dacs: [DacCode(dac0), DacCode(dac1)],
                digital_inputs: [
                    DI[0].load(Ordering::Relaxed),
                    DI[1].load(Ordering::Relaxed),
                ],
            };

            let (gains, telemetry_period) =
                c.shared.settings.lock(|settings| {
                    (
                        settings.dual_iir.ch.each_ref().map(|ch| ch.gain),
                        settings.dual_iir.telemetry_period,
                    )
                });

            let telemetry = telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            );
            let telemetry = Telemetry {
                adcs: telemetry.adcs,
                dacs: telemetry.dacs,
                digital_inputs: telemetry.digital_inputs,
                cpu_temp: telemetry.cpu_temp,
                process: Timing::from_cycles(
                    times.process_sum,
                    count,
                    times.process_max,
                ),
                output: times.output_max as f32 * CPU_CYCLE,
                late: times.late,
                pounder: c.shared.pounder.lock(|pounder| {
                    pounder.as_mut().map(|p| p.pounder.get_telemetry())
                }),
            };

            c.shared.network.lock(|net| {
                net.telemetry.publish_telemetry("/telemetry", &telemetry)
            });

            Systick::delay(((telemetry_period * 1000.0) as u32).millis()).await;
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) -> ! {
        loop {
            // Handle the USB serial terminal.
            c.shared.usb.lock(|usb| {
                usb.poll(&mut [c
                    .local
                    .usb_terminal
                    .interface_mut()
                    .inner_mut()]);
            });

            c.shared.settings.lock(|settings| {
                if c.local.usb_terminal.poll(settings).unwrap() {
                    settings_update::spawn().unwrap()
                }
            });

            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, shared=[network])]
    async fn ethernet_link(mut c: ethernet_link::Context) -> ! {
        loop {
            c.shared.network.lock(|net| net.processor.handle_link());
            Systick::delay(1.secs()).await;
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    // ADC1 SPI errors are detected by `process`, which handles its interrupt.
    #[task(binds = SPI2, priority = 4)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI4, priority = 4)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 SPI error");
    }

    #[task(binds = SPI5, priority = 4)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 SPI error");
    }
}
