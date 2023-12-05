//! # Dual IIR
//!
//! The Dual IIR application exposes two configurable channels. Stabilizer samples input at a fixed
//! rate, digitally filters the data, and then generates filtered output signals on the respective
//! channel outputs.
//!
//! ## Features
//! * Two indpenendent channels
//! * up to 800 kHz rate, timed sampling
//! * Run-time filter configuration
//! * Input/Output data streaming
//! * Down to 2 µs latency
//! * f32 IIR math
//! * Generic biquad (second order) IIR filter
//! * Anti-windup
//! * Derivative kick avoidance
//!
//! ## Settings
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
//!
//! ## Livestreaming
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stabilizer::net::data_stream](../stabilizer/net/data_stream/index.html) for more information.
#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::sync::atomic::{fence, Ordering};

use fugit::ExtU64;
use mutex_trait::prelude::*;

use idsp::iir;

use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        hal, pounder,
        serial_terminal::SerialTerminal,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, SystemTimer, Systick, AFE0, AFE1,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Tree,
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The number of samples in each batch process
const BATCH_SIZE: usize = 1;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 9;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;
// const SAMPLE_PERIOD: f32 =
//     SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

#[derive(Clone, Copy, Debug, Tree)]
pub struct Settings {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    #[tree]
    afe: [Gain; 2],

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    /// * `<m>` specifies which cascade to configure. `<m>` := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    #[tree(depth(2))]
    iir_ch: [[iir::IIR<f32>; IIR_CASCADE_LENGTH]; 2],

    /// Specified true if DI1 should be used as a "hold" input.
    ///
    /// # Path
    /// `allow_hold`
    ///
    /// # Value
    /// "true" or "false"
    allow_hold: bool,

    /// Specified true if "hold" should be forced regardless of DI1 state and hold allowance.
    ///
    /// # Path
    /// `force_hold`
    ///
    /// # Value
    /// "true" or "false"
    force_hold: bool,

    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any non-zero value less than 65536.
    telemetry_period: u16,

    /// Specifies the target for data livestreaming.
    ///
    /// # Path
    /// `stream_target`
    ///
    /// # Value
    /// See [StreamTarget#miniconf]
    stream_target: StreamTarget,

    /// Specifies the centre frequency of the fnc double-pass AOM in hertz
    ///
    /// # Path
    /// `aom_centre_f`
    ///
    /// # Value
    /// A positive 32-bit float in the range [1 MHz, 200 Mhz]
    aom_centre_f: f32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],

            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream_target: StreamTarget::default(),

            aom_centre_f: 50_000.0,
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use stabilizer::hardware::design_parameters::{self, DDS_SYSTEM_CLK};

    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        usb_terminal: SerialTerminal,
        network: NetworkUsers<Settings, Telemetry, 3>,

        settings: Settings,
        telemetry: TelemetryBuffer,

        dds: pounder::dds_output::DdsOutput,
    }

    #[local]
    struct Local {
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        iir_state: [[iir::Vec5<f32>; IIR_CASCADE_LENGTH]; 2],
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        timestamper: pounder::timestamp::Timestamper,
        phase_offset: u16,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        // Configure the microcontroller
        let (mut stabilizer, pounder) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let pounder =
            pounder.expect("Fibre noise cancellation requires a Pounder");

        let flash = stabilizer.usb_serial.flash();
        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &flash.settings.broker,
            &flash.settings.id,
        );

        // todo: check streamformat for pounder data
        let generator = network.configure_streaming(StreamFormat::AdcDacData);

        let settings = Settings::default();

        let mut shared = Shared {
            usb_terminal: stabilizer.usb_serial,
            network,
            settings,
            telemetry: TelemetryBuffer::default(),
            dds: pounder.dds_output,
        };

        let mut local = Local {
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            iir_state: [[[0.; 5]; IIR_CASCADE_LENGTH]; 2],
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            timestamper: pounder.timestamper,
            phase_offset: 0x00,
        };

        let mut dds_profile = shared.dds.builder();

        // set both channels to the same phase
        dds_profile.update_channels(
            ad9959::Channel::ONE | ad9959::Channel::TWO,
            None,
            Some(local.phase_offset),
            None,
        );

        // aom frequency
        let ftw = pounder::dds_output::frequency_to_ftw(
            settings.aom_centre_f,
            design_parameters::DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .ok();
        dds_profile.update_channels(ad9959::Channel::ONE, ftw, None, None);

        // Mix down 2 * aom centre frequency with input APD signal
        let ftw = pounder::dds_output::frequency_to_ftw(
            2.0 * settings.aom_centre_f,
            design_parameters::DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .ok();
        dds_profile.update_channels(ad9959::Channel::TWO, ftw, None, None);

        dds_profile.write();

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
        start::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(stabilizer.systick))
    }

    #[task(priority = 1, local=[sampling_timer, timestamper])]
    fn start(c: start::Context) {
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();

        // todo: check if needed
        // Start sampling pounder ADCs and DACs
        c.local.timestamper.start();
    }

    /// Main DSP processing routine.
    ///
    /// # Note
    /// Processing time for the DSP application code is bounded by the following constraints:
    ///
    /// DSP application code starts after the ADC has generated a batch of samples and must be
    /// completed by the time the next batch of ADC samples has been acquired (plus the FIFO buffer
    /// time). If this constraint is not met, firmware will panic due to an ADC input overrun.
    ///
    /// The DSP application code must also fill out the next DAC output buffer in time such that the
    /// DAC can switch to it when it has completed the current buffer. If this constraint is not met
    /// it's possible that old DAC codes will be generated on the output and the output samples will
    /// be delayed by 1 batch.
    ///
    /// Because the ADC and DAC operate at the same rate, these two constraints actually implement
    /// the same time bounds, meeting one also means the other is also met.
    // todo: binds?
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, iir_state, generator, phase_offset], shared=[settings, telemetry, dds], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            dds,
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            iir_state,
            generator,
            phase_offset,
        } = c.local;

        (settings, telemetry, dds).lock(|settings, telemetry, dds| {
            let digital_inputs =
                [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
            telemetry.digital_inputs = digital_inputs;

            let hold = settings.force_hold
                || (digital_inputs[1] && settings.allow_hold);

            let mut dds_profile = dds.builder();

            let power_in: f32 = 0.0;
            (adc0, adc1).lock(|adc0, adc1| {
                let adc_samples = [adc0, adc1];

                // Preserve instruction and data ordering w.r.t. DMA flag access.
                fence(Ordering::SeqCst);

                // for 0 in 0..adc_samples.len() {
                adc_samples[0]
                    .iter()
                    .map(|ai| {
                        let power_in = f32::from(*ai as i16);

                        let iir_out = settings.iir_ch[0]
                            .iter()
                            .zip(iir_state[0].iter_mut())
                            .fold(power_in, |iir_accumulator, (ch, state)| {
                                ch.update(state, iir_accumulator, hold)
                            });

                        *phase_offset = (((iir_out * (1 << 14) as f32)
                            as i16
                            & 0x3FFFi16)
                            as u16
                            + *phase_offset)
                            & 0x3FFFu16;

                        dds_profile.update_channels(
                            ad9959::Channel::ONE,
                            None,
                            Some(*phase_offset),
                            None,
                        );
                        dds_profile.update_channels(
                            ad9959::Channel::TWO,
                            None,
                            Some(2 * (*phase_offset) & 0x3FFF),
                            None,
                        );

                        dds_profile.write();
                    })
                    .last();
                // }

                // // Stream the data.
                // const N: usize = BATCH_SIZE * core::mem::size_of::<i16>();
                // generator.add(|buf| {
                //     for (data, buf) in adc_samples
                //         .iter()
                //         .chain(dac_samples.iter())
                //         .zip(buf.chunks_exact_mut(N))
                //     {
                //         let data = unsafe {
                //             core::slice::from_raw_parts(
                //                 data.as_ptr() as *const MaybeUninit<u8>,
                //                 N,
                //             )
                //         };
                //         buf.copy_from_slice(data)
                //     }
                //     N * 4
                // });

                generator.add(|buf| {
                    let power_data = unsafe {
                        core::slice::from_raw_parts(
                            power_in.to_ne_bytes().as_ptr()
                                as *const MaybeUninit<u8>,
                            4,
                        )
                    };
                    let phase_data = unsafe {
                        core::slice::from_raw_parts(
                            phase_offset.to_ne_bytes().as_ptr()
                                as *const MaybeUninit<u8>,
                            2,
                        )
                    };

                    buf[0..4].copy_from_slice(power_data);
                    buf[4..6].copy_from_slice(phase_data);

                    6 as usize
                });
                // // Update telemetry measurements.
                telemetry.adcs =
                    [AdcCode(adc_samples[0][0]), AdcCode(adc_samples[0][0])];

                // telemetry.dacs = [
                //     DacCode(dac_samples[0][0]),
                //     DacCode(dac_samples[1][0]),
                // ];

                // // Preserve instruction and data ordering w.r.t. DMA flag access.
                fence(Ordering::SeqCst);
            });
        });
    }

    // todo: No change?
    #[idle(shared=[network, usb_terminal])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged(_path) => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    // We can't sleep if USB is not in suspend.
                    if c.shared
                        .usb_terminal
                        .lock(|terminal| terminal.usb_is_suspended())
                    {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }

    // todo: Done?
    #[task(priority = 1, local=[afes], shared=[network, settings, dds])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c.shared.network.lock(|net| *net.miniconf.settings());
        c.shared.settings.lock(|current| *current = settings);

        c.local.afes.0.set_gain(settings.afe[0]);
        c.local.afes.1.set_gain(settings.afe[1]);

        let ftw_ch1 = pounder::dds_output::frequency_to_ftw(
            settings.aom_centre_f,
            DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .ok();
        let ftw_ch2 = pounder::dds_output::frequency_to_ftw(
            2.0 * settings.aom_centre_f,
            DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .ok();
        if ftw_ch1.is_none() || ftw_ch2.is_none() {
            log::warn!("Failed to set desired aom centre frequency");
        } else {
            c.shared.dds.lock(|dds| {
                let mut dds_profile = dds.builder();
                dds_profile.update_channels(
                    ad9959::Channel::ONE,
                    ftw_ch1,
                    None,
                    None,
                );
                dds_profile.update_channels(
                    ad9959::Channel::TWO,
                    ftw_ch2,
                    None,
                    None,
                );
                dds_profile.write();
            });
        }
        let target = settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));
    }

    // todo: fix pounder telemetry structures -- non-critical
    #[task(priority = 1, shared=[network, settings, telemetry], local=[cpu_temp_sensor])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period) = c
            .shared
            .settings
            .lock(|settings| (settings.afe, settings.telemetry_period));

        c.shared.network.lock(|net| {
            net.telemetry.publish(&telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            ))
        });

        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    // todo: no change
    #[task(priority = 1, shared=[usb_terminal])]
    fn usb(mut c: usb::Context) {
        // Handle the USB serial terminal.
        c.shared.usb_terminal.lock(|usb| usb.process());

        // Schedule to run this task every 10 milliseconds.
        usb::spawn_after(10u64.millis()).unwrap();
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
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
