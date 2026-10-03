//! # 674 nm laser lock (`l674`)
//!
//! Cascaded dual IIR filter for locking an M-Squared SolsTiS laser to a cavity: channel 0
//! drives the fast PZT from the error signal on ADC0, and channel 1 drives the slow PZT from
//! the *output* of channel 0 (rather than from ADC1). ADC1 carries the cavity transmission,
//! which is used for lock detection.
//!
//! ## Features
//! * `dual-iir` processing at 781.25 kHz with two cascaded biquads per channel, channel 1
//!   filtering the channel 0 output
//! * Gain ramp of the error signal when the lock is switched on
//! * Lock detection on ADC1 with a holdoff, reported on LVDS6 of the EEM connector, in the
//!   telemetry, and as the filtered transmission at `lock_detect/adc1_filtered` (read-only)
//! * Auxiliary TTL output on LVDS7 (`aux_ttl_out`), e.g. for holding an external lock
//! * Signal sources, run/hold, streaming and telemetry as `dual-iir`
//!
//! ## Settings
//! Refer to the [L674] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stream] for more information.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use miniconf::Tree;

use dsp_process::SplitProcess;
use idsp::iir::{self, pid::Units};

use platform::{AppSettings, NetSettings};
use serde::{Deserialize, Serialize};
use signal_generator::{self, Source};
use stabilizer::convert::{AdcCode, DacCode, Gain};
use stabilizer::l674::{GainRamp, LockDetect, LockDetectConfig};

// The number of cascaded IIR biquads per channel.
const IIR_CASCADE_LENGTH: usize = 2;

// The number of samples in each batch process
const BATCH_SIZE: usize = 8;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;
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
    l674: L674,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            l674: L674::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            l674: L674::default(),
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

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize, Default)]
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

/// A channel of the cascade: channel 0 filters ADC0 and drives DAC0 (fast PZT), channel 1
/// filters the channel 0 output and drives DAC1 (slow PZT).
#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Channel {
    /// Analog Front End (AFE) gain.
    ///
    /// The channel 1 gain is that of ADC1 (the cavity transmission).
    #[tree(with=miniconf::leaf)]
    gain: Gain,
    /// Biquads, in the order they are applied
    biquad: [BiquadRepr; IIR_CASCADE_LENGTH],
    /// Run/Hold behavior
    #[tree(with=miniconf::leaf)]
    run: Run,
    /// Signal generator configuration to add to the DAC0/DAC1 outputs
    source: signal_generator::Config,
}

impl Channel {
    fn build(&self) -> Result<Active, signal_generator::Error> {
        Ok(Active {
            source: self
                .source
                .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip())
                .unwrap(),
            state: Default::default(),
            run: self.run,
            biquad: self
                .biquad
                .each_ref()
                .map(|biquad| biquad.repr.build(&UNITS)),
        })
    }
}

/// A settings leaf which can be read but not written.
mod read_only {
    pub use miniconf::deny::{deserialize_by_key, probe_by_key};
    pub use miniconf::leaf::{
        SCHEMA, mut_any_by_key, ref_any_by_key, serialize_by_key,
    };
}

/// Lock detection on the cavity transmission (ADC1).
///
/// The lock is reported on LVDS6 (and in the telemetry) once the transmission has stayed
/// above the threshold for the reset time, and the report is withdrawn as soon as one sample
/// is below the threshold.
#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct LockDetectSettings {
    /// Transmission threshold, in volts at the ADC1 input (before the AFE gain).
    #[tree(with=miniconf::leaf)]
    threshold: f32,
    /// Time the transmission has to stay above the threshold for the lock to be reported, in
    /// seconds.
    #[tree(with=miniconf::leaf)]
    reset_time: f32,
    /// The low-pass filtered transmission (time constant about 10 ms), in volts at the ADC1
    /// input.
    ///
    /// Read-only: a request to set it is rejected. Request the value (an empty payload with
    /// a response topic) to poll the transmission, e.g. for relocking.
    #[tree(with=read_only)]
    adc1_filtered: f32,
}

impl Default for LockDetectSettings {
    fn default() -> Self {
        Self {
            threshold: 0.0,
            reset_time: 0.0,
            adc1_filtered: 0.0,
        }
    }
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct L674 {
    /// Channel configuration
    ch: [Channel; 2],
    /// Trigger both signal sources
    #[tree(with=miniconf::leaf)]
    trigger: bool,
    /// Time over which the error signal (ADC0) gain is ramped from zero to one after the
    /// lock is switched on, in seconds (zero for no ramp).
    ///
    /// The lock counts as switched on when the first biquad of either channel gets a nonzero
    /// `b0` coefficient (e.g. a PI controller instead of zero coefficients).
    #[tree(with=miniconf::leaf)]
    gain_ramp_time: f32,
    /// Lock detection on the cavity transmission
    lock_detect: LockDetectSettings,
    /// State of the auxiliary TTL output on LVDS7 of the EEM connector.
    #[tree(with=miniconf::leaf)]
    aux_ttl_out: bool,
    /// Telemetry output period in seconds.
    #[tree(with=miniconf::leaf)]
    telemetry_period: f32,
    /// Target IP and port for UDP streaming.
    ///
    /// Can be multicast.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,
}

impl Default for L674 {
    fn default() -> Self {
        Self {
            ch: Default::default(),
            trigger: false,
            gain_ramp_time: 0.0,
            lock_detect: Default::default(),
            aux_ttl_out: false,
            telemetry_period: 10.0,
            stream: Default::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Active {
    run: Run,
    biquad: [iir::BiquadClamp<f32, f32>; IIR_CASCADE_LENGTH],
    state: [iir::DirectForm1<f32>; IIR_CASCADE_LENGTH],
    source: Source,
}

/// The lock-specific DSP state, shared between the processing and the settings update.
#[derive(Clone, Debug, Default)]
pub struct Lock {
    ramp: GainRamp,
    detect_config: LockDetectConfig,
    detect: LockDetect,
}

/// The telemetry reported by this application (at `<prefix>/telemetry`, every
/// `telemetry_period`).
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
    /// The low-pass filtered cavity transmission, in volts at the ADC1 input (see
    /// `lock_detect/adc1_filtered`).
    pub adc1_filtered: f32,
    /// Whether the lock is detected (the state of LVDS6).
    pub locked: bool,
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
        .insert("title".to_string(), "Stabilizer l674".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC]))]
mod app {
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering, fence};
    use fugit::ExtU32 as _;
    use rtic_monotonics::Monotonic;

    use stabilizer::{
        hardware::{
            self, DigitalInput0, DigitalInput1, Eem, Pgia, SerialTerminal,
            SystemTimer, Systick, UsbDevice,
            adc::{Adc0Input, Adc1Input},
            dac::{Dac0Output, Dac1Output},
            hal,
            net::{NetworkState, NetworkUsers},
            timers::SamplingTimer,
        },
        telemetry::TelemetryBuffer,
    };
    use stream::FrameGenerator;

    /// LVDS6 of the EEM connector: the lock detect output.
    type LockDetectOutput = hal::gpio::gpiod::PD3<hal::gpio::Output>;
    /// LVDS7 of the EEM connector: the auxiliary TTL output.
    type AuxTtlOutput = hal::gpio::gpiod::PD4<hal::gpio::Output>;

    /// The filtered transmission (ADC1 codes, as `f32` bits) and the lock detect state, as
    /// of the last batch, for the lower priority tasks (read-only setting and telemetry).
    static ADC1_FILTERED: AtomicU32 = AtomicU32::new(0);
    static LOCKED: AtomicBool = AtomicBool::new(false);

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<L674>,
        settings: Settings,
        active: [Active; 2],
        lock: Lock,
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
        lock_output: Option<LockDetectOutput>,
        aux_output: Option<AuxTtlOutput>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (stabilizer, _mezzanine, eem) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        // The lock detect and auxiliary TTL outputs are on the EEM connector.
        let (lock_output, aux_output) = match eem {
            Eem::Gpio(gpio) => {
                let mut lock_output = gpio.lvds6;
                lock_output.set_low();
                let mut aux_output = gpio.lvds7;
                aux_output.set_low();
                (Some(lock_output), Some(aux_output))
            }
            _ => {
                log::error!(
                    "EEM GPIO not available: lock detect and auxiliary TTL outputs disabled"
                );
                (None, None)
            }
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
            active: stabilizer
                .settings
                .l674
                .ch
                .each_ref()
                .map(|a| a.build().unwrap()),
            lock: Lock::default(),
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
            lock_output,
            aux_output,
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

    /// Process one biquad of a channel (holding its output if the channel is held).
    #[inline]
    fn biquad(
        biquad: &iir::BiquadClamp<f32, f32>,
        state: &mut iir::DirectForm1<f32>,
        x: f32,
        run: bool,
    ) -> f32 {
        if run {
            biquad.process(state, x)
        } else {
            iir::Biquad::<f32>::HOLD.process(state, x)
        }
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// On top of the `dual-iir` processing, the channel 0 input is scaled by the gain ramp,
    /// channel 1 filters the channel 0 output, and the lock detection runs on the ADC1
    /// samples.
    #[task(
        binds=DMA1_STR4,
        local=[
            digital_inputs, adcs, dacs, generator, lock_output,
            source: [[i16; BATCH_SIZE]; 2] = [[0; BATCH_SIZE]; 2]],
        shared=[active, lock, telemetry],
        priority=3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::SharedResources {
            active,
            lock,
            telemetry,
            ..
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            generator,
            lock_output,
            source,
            ..
        } = c.local;

        (active, lock, telemetry).lock(|active, lock, telemetry| {
            (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                // Preserve instruction and data ordering w.r.t. DMA flag access before and after.
                fence(Ordering::SeqCst);
                let adc: [&[u16; BATCH_SIZE]; 2] = [
                    (**adc0).try_into().unwrap(),
                    (**adc1).try_into().unwrap(),
                ];
                let dac: [&mut [u16; BATCH_SIZE]; 2] =
                    [(*dac0).try_into().unwrap(), (*dac1).try_into().unwrap()];

                let [run0, run1] = [
                    active[0].run.run(telemetry.digital_inputs[0]),
                    active[1].run.run(telemetry.digital_inputs[1]),
                ];
                let (fast, slow) = active.split_at_mut(1);
                let (fast, slow) = (&mut fast[0], &mut slow[0]);
                let mut locked = lock.detect.locked();

                for i in 0..BATCH_SIZE {
                    let adc1_sample = adc[1][i] as i16;

                    // Lock detection, with the output pin following the state at once.
                    let now_locked =
                        lock.detect.update(&lock.detect_config, adc1_sample);
                    if now_locked != locked {
                        locked = now_locked;
                        if let Some(pin) = lock_output.as_mut() {
                            if locked {
                                pin.set_high();
                            } else {
                                pin.set_low();
                            }
                        }
                    }

                    // Channel 0: the error signal through the gain ramp and the cascade.
                    let x = f32::from(adc[0][i] as i16) * lock.ramp.step();
                    let y =
                        biquad(&fast.biquad[0], &mut fast.state[0], x, run0);
                    let y0 =
                        biquad(&fast.biquad[1], &mut fast.state[1], y, run0);

                    // Channel 1 filters the channel 0 output.
                    let y1 = slow
                        .biquad
                        .iter()
                        .zip(slow.state.iter_mut())
                        .fold(y0, |y, (b, s)| biquad(b, s, y, run1));

                    let to_dac = |y: f32, source: i16| -> u16 {
                        // Note(unsafe): The filter limits must ensure that the value is in range.
                        // The truncation introduces 1/2 LSB distortion.
                        let y: i16 = unsafe { y.to_int_unchecked() };
                        DacCode::from(y.saturating_add(source)).0
                    };
                    dac[0][i] = to_dac(y0, source[0][i]);
                    dac[1][i] = to_dac(y1, source[1][i]);
                }
                telemetry.adcs = [AdcCode(adc[0][0]), AdcCode(adc[1][0])];
                telemetry.dacs = [DacCode(dac[0][0]), DacCode(dac[1][0])];

                const N: usize = BATCH_SIZE * size_of::<i16>();
                generator.add(|buf| {
                    [adc[0], adc[1], dac[0], dac[1]]
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
            *source = active.each_mut().map(|ch| {
                core::array::from_fn(|_| (ch.source.next().unwrap() >> 16) as _)
            });
            telemetry.digital_inputs =
                [digital_inputs.0.is_high(), digital_inputs.1.is_high()];

            LOCKED.store(lock.detect.locked(), Ordering::Relaxed);
            ADC1_FILTERED
                .store(lock.detect.filtered().to_bits(), Ordering::Relaxed);
        });
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings).lock(
                |net, settings| {
                    let settings = &mut settings.l674;
                    // Refresh the read-only transmission reading before handling requests.
                    settings.lock_detect.adc1_filtered =
                        f32::from_bits(ADC1_FILTERED.load(Ordering::Relaxed))
                            * AdcCode::VOLT_PER_LSB
                            / settings.ch[1].gain.gain();
                    net.update(settings)
                },
            ) {
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

    #[task(priority = 1, local=[afes, aux_output, lock_enabled: bool = false], shared=[network, settings, active, lock])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            let settings = &mut settings.l674;

            c.local.afes[0].set_gain(settings.ch[0].gain);
            c.local.afes[1].set_gain(settings.ch[1].gain);

            if settings.trigger {
                settings.trigger = false;
                let s = settings.ch.each_ref().map(|ch| {
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
            let b = settings.ch.each_ref().map(|ch| {
                (ch.run, ch.biquad.each_ref().map(|b| b.repr.build(&UNITS)))
            });
            // The lock is on if the first biquad of either channel does anything.
            let lock_enabled =
                b.iter().any(|(_, biquad)| biquad[0].coeff.ba[0] != 0.0);
            c.shared.active.lock(|active| {
                for (a, b) in active.iter_mut().zip(b) {
                    (a.run, a.biquad) = b;
                }
            });

            let detect_config = LockDetectConfig::new(
                settings.lock_detect.threshold,
                settings.ch[1].gain.gain(),
                settings.lock_detect.reset_time,
                SAMPLE_PERIOD,
            );
            if let Err(err) = &detect_config {
                log::warn!("{}: {}", err, settings.lock_detect.threshold);
            }
            c.shared.lock.lock(|lock| {
                if let Ok(config) = detect_config {
                    lock.detect_config = config;
                }
                if lock_enabled != *c.local.lock_enabled {
                    *c.local.lock_enabled = lock_enabled;
                    let ramp_time = if lock_enabled {
                        log::info!(
                            "Lock enabled, ramping the gain over {} s",
                            settings.gain_ramp_time
                        );
                        settings.gain_ramp_time
                    } else {
                        0.0
                    };
                    lock.ramp.start(ramp_time, SAMPLE_PERIOD);
                }
            });

            if let Some(pin) = c.local.aux_output.as_mut() {
                if settings.aux_ttl_out {
                    pin.set_high();
                } else {
                    pin.set_low();
                }
            }

            c.shared
                .network
                .lock(|net| net.direct_stream(settings.stream));
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
                        settings.l674.ch.each_ref().map(|ch| ch.gain),
                        settings.l674.telemetry_period,
                    )
                });

            let base = telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            );
            let telemetry = Telemetry {
                adcs: base.adcs,
                dacs: base.dacs,
                digital_inputs: base.digital_inputs,
                cpu_temp: base.cpu_temp,
                adc1_filtered: f32::from_bits(
                    ADC1_FILTERED.load(Ordering::Relaxed),
                ) * AdcCode::VOLT_PER_LSB
                    / gains[1].gain(),
                locked: LOCKED.load(Ordering::Relaxed),
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
