//! # Fibre Noise Cancellation
//!
//! Pounder samples the error signal input and mixes it down with a DDS at
//! 2*aom_f (IN0/IN1). It passes it to Stabilizer for digital filtering
//! (normally a PI application) where it is read at a fixed rate. The filter
//! output is integrated into the phase offset of the DDS output (OUT0/OUT1)
//! at aom_f driving the AOM. Both Pounder channels are available as independent
//! FNC channels.
//!
//! Currently samples at 200 kHz, i.e. once in 5 µs.
//!
//! ## Features
//! * up to 200 kHz rate, timed sampling
//! * Run-time filter configuration
//! * Input/phase offset data streaming
//! * f32 IIR math
//! * Generic biquad (second order) IIR filter
//! * Anti-windup
//! * Derivative kick avoidance
//!
//! ## Settings
//! Refer to the [Fnc] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [stabilizer::telemetry::Telemetry] for information about telemetry reported by this
//! application. Pounder telemetry is included.
//!
//! ## Stream
//! This application streams raw ADC samples and DDS phase offset words over UDP, using the
//! [stream::Format::AdcDacData] format where the DAC codes are replaced by the 14 bit phase offset
//! words of OUT0/OUT1. Refer to [stream] for more information.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use miniconf::Tree;

use idsp::iir::{self, pid::Units};

use platform::{AppSettings, NetSettings};
use serde::{Deserialize, Serialize};
use stabilizer::{
    convert::{AdcCode, Gain},
    fnc::PounderFncSettings,
};

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The number of samples in each batch process
//
// Each batch requires a DDS profile update over QSPI. Larger batches reduce
// the phase update rate. Smaller sample periods stall the QSPI.
const BATCH_SIZE: usize = 1;

// The number of 100MHz timer ticks between each sample. Currently set to 5 us
// corresponding to a 200 kHz sampling rate.
const SAMPLE_TICKS: u32 = 500;
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * stabilizer::design_parameters::TIMER_PERIOD;

// The filter input is in ADC units, the output is a phase increment in turns per sample.
const UNITS: Units<f32> = Units {
    t: SAMPLE_PERIOD,
    x: AdcCode::VOLT_PER_LSB,
    y: 1.0,
};

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    fnc: Fnc,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            fnc: Fnc::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            fnc: Fnc::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename = "BiquadReprTree"))]
pub struct BiquadRepr {
    /// Biquad parameters
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.repr)]
    _typ: (),
    repr: iir::repr::BiquadRepr<f32, f32>,
}

impl Default for BiquadRepr {
    fn default() -> Self {
        let mut i = iir::BiquadClamp::from(iir::Biquad::IDENTITY);
        i.min = -i16::MAX as _;
        i.max = i16::MAX as _;
        Self {
            _typ: (),
            repr: iir::repr::BiquadRepr::Raw(i),
        }
    }
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Default)]
pub enum Run {
    #[default]
    /// Run
    Run,
    /// Hold
    Hold,
    /// Hold controlled by corresponding digital input
    External,
}

impl Run {
    fn run(&self, di: bool) -> bool {
        match self {
            Self::Run => true,
            Self::Hold => false,
            Self::External => di,
        }
    }
}

/// An FNC channel: Pounder IN/OUT pair and ADC input
#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Channel {
    /// Analog Front End (AFE) gain.
    #[tree(with=miniconf::leaf)]
    gain: Gain,
    /// Biquad
    biquad: [BiquadRepr; IIR_CASCADE_LENGTH],
    /// Run/Hold behavior
    ///
    /// On hold, the phase offset continues to advance at the last rate.
    #[tree(with=miniconf::leaf)]
    run: Run,
    /// Pounder DDS and attenuator settings
    pounder: PounderFncSettings,
}

impl Channel {
    fn build(&self) -> Active {
        Active {
            state: Default::default(),
            run: self.run,
            biquad: self
                .biquad
                .each_ref()
                .map(|biquad| biquad.repr.build(&UNITS)),
        }
    }
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct Fnc {
    /// Channel configuration
    ch: [Channel; 2],
    /// Telemetry output period in seconds.
    #[tree(with=miniconf::leaf)]
    telemetry_period: f32,
    /// Target IP and port for UDP streaming.
    ///
    /// Can be multicast.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,
}

impl Default for Fnc {
    fn default() -> Self {
        Self {
            telemetry_period: 10.0,
            stream: Default::default(),
            ch: Default::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Active {
    run: Run,
    biquad: [iir::BiquadClamp<f32, f32>; IIR_CASCADE_LENGTH],
    state: [iir::DirectForm1<f32>; IIR_CASCADE_LENGTH],
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
        .insert("title".to_string(), "Stabilizer fnc".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC]))]
mod app {
    use super::*;
    use arbitrary_int::u14;
    use core::num::Wrapping;
    use core::sync::atomic::{Ordering, fence};
    use dsp_process::SplitProcess;
    use fugit::ExtU32 as _;
    use rtic_monotonics::Monotonic;

    use stabilizer::{
        hardware::{
            self, DigitalInput0, DigitalInput1, Pgia, SerialTerminal,
            SystemTimer, Systick, UsbDevice,
            adc::{Adc0Input, Adc1Input},
            hal,
            net::{NetworkState, NetworkUsers},
            pounder::{self, PounderDevices, dds_output::DdsOutput},
            setup::Mezzanine,
            timers::SamplingTimer,
        },
        telemetry::TelemetryBuffer,
    };
    use stream::FrameGenerator;

    /// Pounder (input, output) channels for each FNC channel
    const CHANNELS: [(pounder::Channel, pounder::Channel); 2] = [
        (pounder::Channel::In0, pounder::Channel::Out0),
        (pounder::Channel::In1, pounder::Channel::Out1),
    ];

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Fnc>,
        settings: Settings,
        active: [Active; 2],
        telemetry: TelemetryBuffer,
        pounder: PounderDevices,
        dds_output: DdsOutput,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: [Pgia; 2],
        adcs: (Adc0Input, Adc1Input),
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (stabilizer, mezzanine, _eem) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let Mezzanine::Pounder(pounder) = mezzanine else {
            panic!("Fibre noise cancellation requires a Pounder");
        };

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
            active: stabilizer.settings.fnc.ch.each_ref().map(|a| a.build()),
            telemetry: TelemetryBuffer::default(),
            settings: stabilizer.settings,
            pounder: pounder.pounder,
            dds_output: pounder.dds_output,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.sampling_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
        };

        // Enable ADC events
        local.adcs.0.start();
        local.adcs.1.start();

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
        // Start sampling ADCs.
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// The filter output of each channel is a phase increment (in turns) that is accumulated into
    /// the phase offset of the respective Pounder output DDS channel.
    #[task(
        binds=DMA1_STR4,
        local=[digital_inputs, adcs, generator, phase: [u16; 2] = [0; 2]],
        shared=[active, telemetry, dds_output],
        priority=3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::SharedResources {
            active,
            telemetry,
            dds_output,
            ..
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            generator,
            phase,
            ..
        } = c.local;

        (active, telemetry, dds_output).lock(
            |active, telemetry, dds_output| {
                let di =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                telemetry.digital_inputs = di;

                (adc0, adc1).lock(|adc0, adc1| {
                    // Preserve instruction and data ordering w.r.t. DMA flag access before and after.
                    fence(Ordering::SeqCst);
                    let adc: [&[u16; BATCH_SIZE]; 2] = [
                        (**adc0).try_into().unwrap(),
                        (**adc1).try_into().unwrap(),
                    ];
                    let mut pow = [[0u16; BATCH_SIZE]; 2];

                    let mut builder = dds_output.builder();
                    for (((((adc, pow), active), phase), di), (_, out)) in adc
                        .into_iter()
                        .zip(pow.iter_mut())
                        .zip(active.iter_mut())
                        .zip(phase.iter_mut())
                        .zip(di)
                        .zip(CHANNELS)
                    {
                        for (adc, pow) in adc.iter().zip(pow.iter_mut()) {
                            let x = f32::from(*adc as i16);
                            let y = active
                                .biquad
                                .iter()
                                .zip(active.state.iter_mut())
                                .fold(x, |y, (ch, state)| {
                                    if active.run.run(di) {
                                        ch.process(state, y)
                                    } else {
                                        iir::Biquad::<f32>::HOLD
                                            .process(state, y)
                                    }
                                });

                            *phase = phase.wrapping_add(
                                ad9959::phase_to_pow(y).0.value(),
                            ) & 0x3FFF;
                            *pow = *phase;
                        }
                        builder.push(
                            out.into(),
                            None,
                            Some(Wrapping(u14::new(*phase))),
                            None,
                        );
                    }
                    dds_output.write(builder);

                    telemetry.adcs = [AdcCode(adc[0][0]), AdcCode(adc[1][0])];

                    const N: usize = BATCH_SIZE * size_of::<u16>();
                    generator.add(|buf| {
                        [adc[0], adc[1], &pow[0], &pow[1]]
                            .into_iter()
                            .zip(buf.chunks_exact_mut(N))
                            .map(|(data, buf)| {
                                buf.copy_from_slice(bytemuck::cast_slice(data))
                            })
                            .count()
                            * N
                    });

                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.fnc))
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

    #[task(priority = 1, local=[afes], shared=[network, settings, active, pounder, dds_output])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.local.afes[0].set_gain(settings.fnc.ch[0].gain);
            c.local.afes[1].set_gain(settings.fnc.ch[1].gain);

            let b = settings.fnc.ch.each_ref().map(|ch| {
                (ch.run, ch.biquad.each_ref().map(|b| b.repr.build(&UNITS)))
            });
            c.shared.active.lock(|active| {
                for (a, b) in active.iter_mut().zip(b) {
                    (a.run, a.biquad) = b;
                }
            });

            // DDS and in particular attenuator updates are slow. Keep the locks short to
            // allow `process` to preempt.
            for (ch, (inp, out)) in settings.fnc.ch.iter().zip(CHANNELS) {
                match ch.pounder.dds_words() {
                    Ok([(ftw_in, acr_in), (ftw_out, acr_out)]) => {
                        c.shared.dds_output.lock(|dds_output| {
                            let mut builder = dds_output.builder();
                            builder.push(
                                inp.into(),
                                Some(ftw_in),
                                None,
                                Some(acr_in),
                            );
                            builder.push(
                                out.into(),
                                Some(ftw_out),
                                None,
                                Some(acr_out),
                            );
                            dds_output.write(builder);
                        });
                    }
                    Err(err) => {
                        log::warn!("Failed to update Pounder DDS: {:?}", err)
                    }
                }

                for (channel, attenuation) in [
                    (inp, ch.pounder.attenuation_in),
                    (out, ch.pounder.attenuation_out),
                ] {
                    c.shared.pounder.lock(|pounder| {
                        if let Err(err) =
                            pounder.set_attenuation(channel, attenuation)
                        {
                            log::warn!(
                                "Failed to update Pounder attenuation: {:?}",
                                err
                            );
                        }
                    });
                }
            }

            c.shared
                .network
                .lock(|net| net.direct_stream(settings.fnc.stream));
        });
    }

    #[task(priority = 1, shared=[network, settings, telemetry, pounder], local=[cpu_temp_sensor])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            let telemetry =
                c.shared.telemetry.lock(|telemetry| telemetry.clone());

            let (gains, telemetry_period) =
                c.shared.settings.lock(|settings| {
                    (
                        settings.fnc.ch.each_ref().map(|ch| ch.gain),
                        settings.fnc.telemetry_period,
                    )
                });

            let mut telemetry = telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            );
            telemetry.pounder =
                Some(c.shared.pounder.lock(|pounder| pounder.get_telemetry()));

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

    #[task(binds = SPI2, priority = 4)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 4)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
    }
}
