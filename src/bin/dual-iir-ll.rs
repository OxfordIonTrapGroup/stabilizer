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
//! Here, each sample is processed individually and immediately: the ADC conversions are still
//! started by the sampling timer and the samples are transferred to memory by DMA, but one sample
//! at a time. As soon as a sample has arrived, the processing routine filters it and writes the
//! resulting DAC code directly into the DAC SPI transmit FIFO, from where it is transmitted
//! immediately. The DAC updates are thus not aligned to the sampling timer.
//!
//! This comes at the expense of one interrupt per sample and thus a much higher CPU load. The
//! processing of a sample must complete within one sample period. If it does not, the firmware
//! panics due to an ADC input overrun. Reduce the sample rate (`SAMPLE_TICKS`) in that case.
//!
//! ## Settings
//! Refer to the [DualIir] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [stabilizer::telemetry::Telemetry] for information about telemetry reported by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stream] for more information. The data is grouped into batches of `BATCH_SIZE` samples like
//! in `dual-iir`.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use miniconf::Tree;

use dsp_process::SplitProcess;
use idsp::iir::{self, pid::Units};

use platform::{AppSettings, NetSettings};
use signal_generator::Source;
use stabilizer::{
    convert::{AdcCode, DacCode},
    dual_iir::{Biquad, Channel, DualIir, IIR_CASCADE_LENGTH, Run},
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

#[derive(Clone, Debug)]
pub struct Active {
    run: Run,
    biquad: [Biquad; IIR_CASCADE_LENGTH],
    state: [iir::DirectForm1<f32>; IIR_CASCADE_LENGTH],
    source: Source,
}

impl Active {
    fn new(ch: &Channel) -> Self {
        Self {
            source: ch
                .source
                .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip())
                .unwrap(),
            state: Default::default(),
            run: ch.run,
            biquad: ch.biquad.each_ref().map(|biquad| biquad.build(&UNITS)),
        }
    }

    /// Filter one ADC sample and add the signal source contribution.
    ///
    /// # Args
    /// * `di` - The digital input state for run/hold control.
    /// * `adc` - The ADC code of the input sample.
    /// * `source` - The signal source contribution in DAC LSB.
    ///
    /// # Returns
    /// The DAC code for the output sample.
    #[inline(always)]
    fn process(&mut self, di: bool, adc: u16, source: i16) -> u16 {
        let x = f32::from(adc as i16);
        let y = self.biquad.iter().zip(self.state.iter_mut()).fold(
            x,
            |y, (ch, state)| {
                if self.run.run(di) {
                    ch.process(state, y)
                } else {
                    iir::Biquad::<f32>::HOLD.process(state, y)
                }
            },
        );

        // Note(unsafe): The filter limits must ensure that the value is in range.
        // The truncation introduces 1/2 LSB distortion.
        let y: i16 = unsafe { y.to_int_unchecked() };
        DacCode::from(y.saturating_add(source)).0
    }
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
    use core::sync::atomic::{Ordering, fence};
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

    // The most recent samples, handed from `process` to `streamer`.
    static RING: SampleRing<512> = SampleRing::new();

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<DualIir>,
        settings: Settings,
        active: [Active; 2],
        telemetry: TelemetryBuffer,
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
            active: stabilizer.settings.dual_iir.ch.each_ref().map(Active::new),
            telemetry: TelemetryBuffer::default(),
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
    /// Runs once per sample. The DAC codes are written to the DACs immediately after they have
    /// been computed and before the telemetry and signal source bookkeeping. Stream frames are
    /// assembled by the lower priority `streamer` task.
    ///
    /// # Note
    /// Processing must complete within one sample period. If this constraint is not met,
    /// firmware will panic due to an ADC input overrun.
    #[task(
        binds=DMA1_STR4,
        local=[digital_inputs, adcs, dacs, source: [i16; 2] = [0; 2]],
        shared=[active, telemetry],
        priority=3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::SharedResources {
            active, telemetry, ..
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            source,
            ..
        } = c.local;

        (active, telemetry).lock(|active, telemetry| {
            (adc0, adc1).lock(|adc0, adc1| {
                // Preserve instruction and data ordering w.r.t. DMA flag access before and after.
                fence(Ordering::SeqCst);
                let adc = [adc0[0], adc1[0]];
                let di = telemetry.digital_inputs;

                // Update each DAC as soon as its code is available.
                let y0 = active[0].process(di[0], adc[0], source[0]);
                dac0.write(y0);
                let y1 = active[1].process(di[1], adc[1], source[1]);
                dac1.write(y1);
                let dac = [y0, y1];

                telemetry.adcs = [AdcCode(adc[0]), AdcCode(adc[1])];
                telemetry.dacs = [DacCode(dac[0]), DacCode(dac[1])];

                // Hand the sample to the stream task and wake it once per batch.
                let count = RING.push([adc[0], adc[1], dac[0], dac[1]]);
                if count.is_multiple_of(BATCH_SIZE as u32) {
                    rtic::pend(hal::stm32::Interrupt::LTDC);
                }

                fence(Ordering::SeqCst);
            });

            *source = active
                .each_mut()
                .map(|ch| (ch.source.next().unwrap() >> 16) as _);
            telemetry.digital_inputs =
                [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
        });
    }

    /// Assemble stream frames from the samples handed over by `process`.
    ///
    /// Pended by `process` once per batch of samples. This keeps the frame generation
    /// (allocation, header, copy, queueing) out of the sample processing.
    ///
    /// LTDC is a spare vector pended in software; the display peripheral is unused.
    /// A direct binding avoids async executor overhead. It runs below DSP and above
    /// network/USB. Wakeups can coalesce because this task drains the sample ring.
    #[task(binds = LTDC, local = [generator, tail: u32 = 0], priority = 2)]
    fn streamer(c: streamer::Context) {
        let streamer::LocalResources {
            generator, tail, ..
        } = c.local;
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

    #[task(priority = 1, local=[afes], shared=[network, settings, active])]
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
                c.shared.active.lock(|ch| {
                    for (ch, s) in ch.iter_mut().zip(s) {
                        if let Ok(s) = s {
                            ch.source = s;
                        }
                    }
                });
            }
            let b = settings.dual_iir.ch.each_ref().map(|ch| {
                (ch.run, ch.biquad.each_ref().map(|b| b.build(&UNITS)))
            });
            c.shared.active.lock(|active| {
                for (a, b) in active.iter_mut().zip(b) {
                    (a.run, a.biquad) = b;
                }
            });
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.dual_iir.stream));
        });
    }

    #[task(priority = 1, shared=[network, settings, telemetry], local=[cpu_temp_sensor])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            let telemetry =
                c.shared.telemetry.lock(|telemetry| telemetry.clone());

            let (gains, telemetry_period) =
                c.shared.settings.lock(|settings| {
                    (
                        settings.dual_iir.ch.each_ref().map(|ch| ch.gain),
                        settings.dual_iir.telemetry_period,
                    )
                });

            c.shared.network.lock(|net| {
                net.telemetry.publish_telemetry(
                    "/telemetry",
                    &telemetry.finalize(
                        gains[0],
                        gains[1],
                        c.local.cpu_temp_sensor.get_temperature().unwrap(),
                    ),
                )
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

    #[task(binds = SPI2, priority = 4)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 4)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
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
