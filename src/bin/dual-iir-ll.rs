//! # Dual IIR, low latency
//!
//! A variant of the [Dual IIR](../dual_iir/index.html) application that minimizes the
//! input-to-output latency. It has the same settings, telemetry and stream format.
//!
//! ## Features
//! * Two indpenendent channels
//! * Timed sampling
//! * Run-time filter configuration
//! * Input/Output data streaming
//! * Each sample is processed individually and immediately
//! * f32 IIR math
//! * Generic biquad (second order) IIR filter
//! * Anti-windup
//! * Derivative kick avoidance
//!
//! ## Latency
//!
//! `dual-iir` collects ADC samples into batches by DMA, processes one batch at a time and
//! lets DMA write the resulting DAC codes three batch periods after the corresponding ADC
//! samples were acquired. This minimizes the CPU overhead per sample, but the input-to-output
//! latency is about three batch periods (about 30 µs).
//!
//! Here, each sample is processed individually and immediately:
//!
//! * The ADC conversions are started by the sampling timer as in `dual-iir`. No CPU time is
//!   spent on this and the sampling instants are free of jitter.
//! * As soon as the conversion results are available in the ADC SPI receive FIFOs, the
//!   processing interrupt reads them from there. No DMA is involved in moving sample data.
//! * Each DAC code is written directly into the DAC SPI transmit FIFO right after it has been
//!   computed, from where it is transmitted immediately. The DAC updates are thus not aligned
//!   to the sampling timer.
//! * Everything else is kept away from that path: the signal sources are advanced after the
//!   DAC codes have been written and only if they are active, stream frames are assembled and
//!   the digital inputs are sampled by a lower priority task once per `BATCH_SIZE` samples.
//!
//! The digital inputs controlling `External` run/hold are therefore applied with a delay of up
//! to two batches, like in `dual-iir`.
//!
//! This comes at the expense of one interrupt per sample. The processing of a sample must
//! complete within one sample period. If it does not, the firmware panics with an "ADC1 sample
//! overrun". Reduce the sample rate (`SAMPLE_TICKS`) in that case. The telemetry reports the
//! mean and maximum execution time of the processing to judge the margin.
//!
//! ## Settings
//! Refer to the [DualIir] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stream] for more information. The data is grouped into batches of `BATCH_SIZE` samples like
//! in `dual-iir`.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use core::sync::atomic::{AtomicBool, Ordering};

use miniconf::Tree;

use dsp_process::SplitProcess;
use idsp::iir::{self, pid::Units};

use platform::{AppSettings, NetSettings};
use serde::Serialize;
use signal_generator::Source;
use stabilizer::{
    convert::{AdcCode, DacCode},
    dual_iir::{Biquad, Channel, DualIir, IIR_CASCADE_LENGTH, Run},
    telemetry::{ProcessTime, Timing},
};

// The number of samples in each streamed batch
const BATCH_SIZE: usize = 8;

// The number of 100MHz timer ticks between each sample. With a value of 128, there is 1.28uS per
// sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS: u32 = 128;
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
    state: [iir::DirectForm1<f32>; IIR_CASCADE_LENGTH],
    /// The signal source. `None` if its output is constant.
    source: Option<Source>,
    /// The signal source contribution to the next output sample in DAC LSB.
    offset: i16,
}

impl Active {
    fn new(ch: &Channel) -> Self {
        let mut active = Self {
            source: None,
            offset: 0,
            state: Default::default(),
            run: ch.run,
            biquad: ch.biquad.each_ref().map(|biquad| biquad.build(&UNITS)),
        };
        active.set_source(
            ch.source
                .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip())
                .unwrap(),
        );
        active
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
    /// # Args
    /// * `di` - The digital input state for run/hold control.
    /// * `adc` - The ADC code of the input sample.
    ///
    /// # Returns
    /// The DAC code for the output sample.
    #[inline(always)]
    fn process(&mut self, di: &AtomicBool, adc: u16) -> u16 {
        let run = match self.run {
            Run::Run => true,
            Run::Hold => false,
            Run::External => di.load(Ordering::Relaxed),
        };
        let x = f32::from(adc as i16);
        let y = self.biquad.iter().zip(self.state.iter_mut()).fold(
            x,
            |y, (ch, state)| {
                if run {
                    ch.process(state, y)
                } else {
                    iir::Biquad::<f32>::HOLD.process(state, y)
                }
            },
        );

        // Note(unsafe): The filter limits must ensure that the value is in range.
        // The truncation introduces 1/2 LSB distortion.
        let y: i16 = unsafe { y.to_int_unchecked() };
        DacCode::from(y.saturating_add(self.offset)).0
    }

    /// Advance the signal source for the next sample.
    #[inline(always)]
    fn update(&mut self) {
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

/// The state of the sample processing routine.
///
/// It is shared with the settings update and telemetry tasks.
pub struct State {
    ch: [Active; 2],
    /// Execution time of the entire routine and time until the last DAC write.
    timing: [ProcessTime; 2],
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
    /// Time from the start of the sample processing routine until the last DAC code has
    /// been written.
    pub output: Timing,
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
            timers::SamplingTimer,
        },
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
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (stabilizer, _mezzanine, _eem) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            hardware::Sampling::LowLatency,
            SAMPLE_TICKS,
        );

        let mut network = NetworkUsers::new(
            stabilizer.network_devices.stack,
            stabilizer.network_devices.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &stabilizer.settings.net,
            stabilizer.metadata,
        );

        let generator = network.configure_streaming(stream::Format::AdcDacData);

        let shared = Shared {
            usb: stabilizer.usb,
            network,
            state: State {
                ch: stabilizer.settings.dual_iir.ch.each_ref().map(Active::new),
                timing: Default::default(),
            },
            settings: stabilizer.settings,
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
    /// Processing must complete within one sample period. If this constraint is not met,
    /// firmware will panic due to an ADC sample overrun.
    #[task(binds = SPI3, local = [adcs, dacs], shared = [state], priority = 3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(mut c: process::Context) {
        let start = DWT::cycle_count();

        let (adc0, adc1) = c.local.adcs;
        let (dac0, dac1) = c.local.dacs;

        let adc = [adc0.read(), adc1.read()];

        c.shared.state.lock(|state| {
            let State {
                ch: [ch0, ch1],
                timing,
            } = state;

            // Update each DAC as soon as its code is available.
            let dac = [ch0.process(&DI[0], adc[0]), 0];
            dac0.write(dac[0]);
            let dac = [dac[0], ch1.process(&DI[1], adc[1])];
            dac1.write(dac[1]);
            let output = DWT::cycle_count();

            // Hand the sample to the stream task and wake it once per batch.
            let count = RING.push([adc[0], adc[1], dac[0], dac[1]]);
            if count.is_multiple_of(BATCH_SIZE as u32) {
                rtic::pend(hal::stm32::Interrupt::LTDC);
            }

            ch0.update();
            ch1.update();

            // Detect SPI errors and whether this took longer than a sample period.
            adc1.check();

            timing[1].update(output.wrapping_sub(start));
            timing[0].update(DWT::cycle_count().wrapping_sub(start));
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

    #[task(priority = 1, local=[afes], shared=[network, settings, state])]
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
                for (a, b) in state.ch.iter_mut().zip(b) {
                    (a.run, a.biquad) = b;
                }
            });
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.dual_iir.stream));
        });
    }

    #[task(priority = 1, shared=[network, settings, state], local=[cpu_temp_sensor])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            // Take the timing statistics accumulated since the previous report.
            let timing = c
                .shared
                .state
                .lock(|state| core::mem::take(&mut state.timing));

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
                process: timing[0].finalize(),
                output: timing[1].finalize(),
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
