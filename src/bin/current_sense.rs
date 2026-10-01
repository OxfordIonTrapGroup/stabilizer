//! # Dual IIR with mains-synchronised harmonic feedforward
//!
//! The `current_sense` application is a variant of `dual-iir` for the current sense mezzanine. Stabilizer
//! samples the inputs at a fixed rate, digitally filters the data, and then generates filtered
//! output signals on the respective channel outputs. On channel 0, a harmonic feedforward waveform
//! synchronised to the mains is added to the input.
//!
//! ## Features
//! * Two indpenendent channels
//! * up to 800 kHz rate, timed sampling
//! * Run-time filter configuration
//! * Input/Output data streaming
//! * f32 IIR math
//! * Generic biquad (second order) IIR filter
//! * Anti-windup
//! * Derivative kick avoidance
//! * Harmonic feedforward on channel 0, phase-locked to a mains reference applied to DI0
//! * Frontend DC offset via the current sense board offset DAC
//! * Feedback offset current via auxiliary (MCU-internal) DAC output 0
//!
//! ## Mains synchronisation
//! The rising edges of the mains reference on DI0 are timestamped in hardware. A reciprocal PLL
//! ([idsp::RPLL]), advanced once per sample batch, tracks the mains phase from these timestamps.
//! The feedforward waveform is evaluated for every sample directly from the PLL phase, keeping all
//! harmonics coherent with the reference.
//!
//! The feedforward is only available on channel 0, matching the single-channel current sense
//! hardware; channel 1 is a regular `dual-iir` channel. Conversely, as DI0 is taken by the mains
//! reference, the external run/hold control is only available on channel 1.
//!
//! ## Settings
//! Refer to the [CurrentSense] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [stabilizer::telemetry::Telemetry] for information about telemetry reported by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stream] for more information.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use core::num::Wrapping;

use miniconf::{Leaf, Tree};

use dsp_process::SplitProcess;
use idsp::{
    RPLL, RPLLConfig,
    iir::{self, pid::Units},
};

use platform::{AppSettings, NetSettings};
use serde::{Deserialize, Serialize};
use signal_generator::{self, Source};
use stabilizer::convert::{AdcCode, DacCode, Gain};
use stabilizer::harmonics::{Harmonic, HarmonicWaveform};

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The logarithm of the number of samples in each batch process. This corresponds with 2^3 samples
// per batch = 8 samples
const BATCH_SIZE_LOG2: u32 = 3;
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u32 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * stabilizer::design_parameters::TIMER_PERIOD;

const UNITS: Units<f32> = Units {
    t: SAMPLE_PERIOD,
    x: AdcCode::VOLT_PER_LSB,
    y: DacCode::VOLT_PER_LSB,
};

// The logarithm of the number of timestamp timer ticks between PLL updates (one per batch).
const PLL_DT2: u8 = (SAMPLE_TICKS_LOG2 + BATCH_SIZE_LOG2) as u8;

// The range of PLL settling time exponents supported by the RPLL arithmetic.
const PLL_TC_MIN: u8 = PLL_DT2 + 1;
const PLL_TC_MAX: u8 = 32;

// Minimum spacing between accepted mains timestamps, in timestamp timer ticks.
// Captures closer than this to the previous accepted one are treated as glitches.
const MIN_TIMESTAMP_SPACING_TICKS: u32 =
    (0.005 / stabilizer::design_parameters::TIMER_PERIOD) as u32;

// Feedback offset current per auxiliary DAC output voltage (A/V), fixed by the current sense
// board design (independent of the shunt chosen for the particular application).
const FEEDBACK_OFFSET_CURRENT_PER_VOLT: f32 = 0.0235;

// Number of harmonics (fundamental included) in the feedforward waveform.
const MAX_HARMONICS: usize = 5;

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    current_sense: CurrentSense,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            current_sense: CurrentSense::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            current_sense: CurrentSense::default(),
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
    ///
    /// Not available on channel 0, as digital input 0 is taken by the mains reference.
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

/// A ADC-DAC channel
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
    /// `External` is not available on channel 0, as digital input 0 is taken by the mains
    /// reference; it is replaced by `Hold` there.
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

/// Amplitude and phase of one harmonic of the feedforward waveform.
#[derive(Copy, Clone, Debug, Default, Serialize, Deserialize)]
pub struct FeedforwardHarmonic {
    /// Amplitude in volts, referred to the ADC input.
    pub amp: f32,
    /// Phase in turns (any value, wrapped modulo 1).
    pub phase_turns: f32,
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct CurrentSense {
    /// Channel configuration
    ch: [Channel; 2],
    /// Trigger both signal sources
    #[tree(with=miniconf::leaf)]
    trigger: bool,
    /// Harmonic feedforward waveform added to the channel 0 input.
    ///
    /// The waveform is `sum(amp[n] * sin(2π * ((n + 1) * theta + phase_turns[n])))` over all
    /// `n`, where `theta` is the mains phase in turns tracked by the PLL (zero at the reference
    /// edge). The index `n` selects the harmonic of order `n + 1`, i.e. the fundamental comes
    /// first.
    feedforward_harmonics: [Leaf<FeedforwardHarmonic>; MAX_HARMONICS],
    /// Mains PLL time constants.
    ///
    /// `1 << pll_tc[n]` is the settling time in 100 MHz timestamp timer ticks, where index 0
    /// selects the frequency lock settling time and index 1 the phase lock settling time
    /// (usually one less than the former). The frequency settling time must be longer than
    /// one mains period (`pll_tc[0] >= 22` at 50 Hz). Values outside of the supported range
    /// from 11 to 32 are clamped.
    pll_tc: [u8; 2],
    /// Frontend DC offset voltage, set via the current sense board offset DAC.
    ///
    /// Voltage in volts (clamped to the DAC output range from 0 V to 2.5 V).
    #[tree(with=miniconf::leaf)]
    frontend_offset: f32,
    /// Feedback offset current, generated from auxiliary DAC output 0.
    ///
    /// The auxiliary DAC output voltage is converted to a current at 23.5 mA/V by the current
    /// sense board, giving a range of 0 to ~48 mA for the 0 V to 2.048 V DAC output range.
    ///
    /// Current in amperes (clamped to the output range).
    #[tree(with=miniconf::leaf)]
    feedback_offset: f32,
    /// Telemetry output period in seconds.
    #[tree(with=miniconf::leaf)]
    telemetry_period: f32,
    /// Target IP and port for UDP streaming.
    ///
    /// Can be multicast.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,
}

impl Default for CurrentSense {
    fn default() -> Self {
        Self {
            ch: Default::default(),
            trigger: false,

            // No feedforward on start up.
            feedforward_harmonics: Default::default(),

            // Frequency and phase settling time (log2 timestamp timer ticks):
            // 2^24 ticks ≈ 168 ms and 2^23 ticks ≈ 84 ms at 100 MHz.
            //
            // Rationale for default choice: grid frequency changes slowly, so trigger
            // jitter dominates for faster loops. The current settings give the lowest
            // phase noise for a few tens of µs jitter (as measured on 2025-05-16
            // between a Tektronix scope, Sinara line_trigger and the legacy trigger
            // box in lab one with presumably different filters, where the presumably
            // different filters hopefully would mean that noise doesn't entirely
            // common-mode out of the comparison). The largest-slew GB grid events seem
            // to have been around 0.125 Hz/s, giving a tracking error of ~0.6°, which
            // is still ~25 dB cancellation at the 5th harmonic. The initial lock on
            // boot-up takes ~1 s.
            pll_tc: [24, 23],

            // No frontend DC offset
            frontend_offset: 0.0,

            // 11 mA feedback offset current.
            feedback_offset: 0.011,

            telemetry_period: 10.0,
            stream: Default::default(),
        }
    }
}

impl CurrentSense {
    /// Replace settings that are not supported by valid ones.
    ///
    /// `Run::External` on channel 0 is replaced by `Run::Hold`, and PLL time constants outside
    /// of the range supported by the RPLL are clamped.
    fn validate(&mut self) {
        if self.ch[0].run == Run::External {
            log::warn!(
                "External run/hold control not available on channel 0, holding"
            );
            self.ch[0].run = Run::Hold;
        }

        for tc in self.pll_tc.iter_mut() {
            let clamped = (*tc).clamp(PLL_TC_MIN, PLL_TC_MAX);
            if clamped != *tc {
                log::warn!(
                    "PLL time constant {} out of range, clamped to {}",
                    *tc,
                    clamped
                );
                *tc = clamped;
            }
        }
    }

    /// Derive the mains feedforward from the (validated) settings.
    fn build_feedforward(&self) -> Feedforward {
        Feedforward {
            pll: RPLLConfig {
                dt2: PLL_DT2,
                shift_frequency: self.pll_tc[0],
                shift_phase: self.pll_tc[1],
            },
            waveform: HarmonicWaveform::new(
                self.feedforward_harmonics.each_ref().map(|harmonic| {
                    Harmonic::new(
                        harmonic.amp * AdcCode::LSB_PER_VOLT,
                        harmonic.phase_turns,
                    )
                }),
            ),
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

/// The mains feedforward for channel 0
#[derive(Clone, Debug)]
pub struct Feedforward {
    /// Mains PLL configuration
    pll: RPLLConfig,
    /// Feedforward waveform as a function of the mains phase, in ADC codes
    waveform: HarmonicWaveform<MAX_HARMONICS>,
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
        .insert("title".to_string(), "Stabilizer current_sense".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC]))]
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
            aux_dac::{AuxDac0, AuxDacOutput},
            current_sense_dac::CurrentSenseDac,
            dac::{Dac0Output, Dac1Output},
            hal,
            input_stamper::InputStamper,
            net::{NetworkState, NetworkUsers},
            setup::Mezzanine,
            timers::SamplingTimer,
        },
        telemetry::TelemetryBuffer,
    };
    use stream::FrameGenerator;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<CurrentSense>,
        settings: Settings,
        active: [Active; 2],
        feedforward: Feedforward,
        telemetry: TelemetryBuffer,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        timestamper: InputStamper,
        afes: [Pgia; 2],
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        pll: RPLL,
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        current_sense_dac: Option<CurrentSenseDac>,
        aux_dac: AuxDac0,
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
                BATCH_SIZE,
                SAMPLE_TICKS,
            );

        let current_sense_dac = match mezzanine {
            Mezzanine::CurrentSense(dac) => Some(dac),
            Mezzanine::Pounder(_) => None,
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

        // Set the initial output before enabling to avoid glitching through 0 V. Aux DAC 1 is
        // not used by the current sense board, so is left disabled.
        let aux_dac = {
            let mut dac = stabilizer.aux_dacs.0;
            dac.set_voltage(
                stabilizer.settings.current_sense.feedback_offset
                    / FEEDBACK_OFFSET_CURRENT_PER_VOLT,
            );
            dac.enable()
        };

        stabilizer.settings.current_sense.validate();

        let shared = Shared {
            usb: stabilizer.usb,
            network,
            active: stabilizer
                .settings
                .current_sense
                .ch
                .each_ref()
                .map(|a| a.build().unwrap()),
            feedforward: stabilizer.settings.current_sense.build_feedforward(),
            telemetry: TelemetryBuffer::default(),
            settings: stabilizer.settings,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.sampling_timer,
            digital_inputs: stabilizer.digital_inputs,
            timestamper: stabilizer.input_stamper,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            pll: RPLL::default(),
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            current_sense_dac,
            aux_dac,
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

        // Start recording mains reference timestamps on DI0.
        stabilizer.timestamp_timer.start();
        local.timestamper.start();

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
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// On top of the `dual-iir` processing, this tracks the mains reference on DI0 with a PLL and
    /// adds the harmonic feedforward waveform evaluated from the PLL phase to the ADC0 samples.
    #[task(
        binds=DMA1_STR4,
        local=[
            digital_inputs, adcs, dacs, generator, timestamper, pll,
            last_timestamp: Option<u32> = None,
            source: [[i16; BATCH_SIZE]; 2] = [[0; BATCH_SIZE]; 2]],
        shared=[active, feedforward, telemetry],
        priority=3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::SharedResources {
            active,
            feedforward,
            telemetry,
            ..
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            generator,
            timestamper,
            pll,
            last_timestamp,
            source,
            ..
        } = c.local;

        (active, feedforward, telemetry).lock(
            |active, feedforward, telemetry| {
                // Fetch the latest capture of the mains reference edge (if any arrived since the
                // previous batch). Timestamps from capture overflows are ignored, as are captures
                // too close to the previously accepted one (glitches).
                let timestamp = timestamper
                    .latest_timestamp()
                    .unwrap_or(None)
                    .filter(|timestamp| {
                        last_timestamp.is_none_or(|last| {
                            timestamp.wrapping_sub(last)
                                >= MIN_TIMESTAMP_SPACING_TICKS
                        })
                    });
                if timestamp.is_some() {
                    *last_timestamp = timestamp;
                }

                // Advance the PLL by one batch. The resulting phase is the reference phase at
                // the start of this batch (zero at the reference edge) and the frequency is the
                // phase increment per batch.
                let reference = feedforward
                    .pll
                    .process(pll, timestamp.map(|t| Wrapping(t as i32)));
                let sample_frequency =
                    (reference.step.0 as u32 >> BATCH_SIZE_LOG2) as i32;

                // Evaluate the feedforward waveform (in ADC codes) at each sample of the batch.
                let feedforward: [f32; BATCH_SIZE] = feedforward
                    .waveform
                    .sample_batch(reference.state.0, sample_frequency);

                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    // Preserve instruction and data ordering w.r.t. DMA flag access before and after.
                    fence(Ordering::SeqCst);
                    let adc: [&[u16; BATCH_SIZE]; 2] = [
                        (**adc0).try_into().unwrap(),
                        (**adc1).try_into().unwrap(),
                    ];
                    let mut dac: [&mut [u16; BATCH_SIZE]; 2] = [
                        (*dac0).try_into().unwrap(),
                        (*dac1).try_into().unwrap(),
                    ];

                    // The feedforward is only added to the channel 0 input.
                    for (((((adc, dac), active), di), source), feedforward) in
                        adc.into_iter()
                            .zip(dac.iter_mut())
                            .zip(active.iter_mut())
                            .zip(telemetry.digital_inputs)
                            .zip(source.iter())
                            .zip([feedforward, [0.0; BATCH_SIZE]])
                    {
                        for (((adc, dac), source), feedforward) in adc
                            .iter()
                            .zip(dac.iter_mut())
                            .zip(source)
                            .zip(feedforward)
                        {
                            let x = f32::from(*adc as i16) + feedforward;
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

                            // Note(unsafe): The filter limits must ensure that the value is in range.
                            // The truncation introduces 1/2 LSB distortion.
                            let y: i16 = unsafe { y.to_int_unchecked() };
                            *dac = DacCode::from(y.saturating_add(*source)).0;
                        }
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
                    core::array::from_fn(|_| {
                        (ch.source.next().unwrap() >> 16) as _
                    })
                });
                telemetry.digital_inputs =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
            },
        );
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.current_sense))
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

    #[task(priority = 1, local=[afes, current_sense_dac, aux_dac], shared=[network, settings, active, feedforward])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            let settings = &mut settings.current_sense;
            settings.validate();

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
            c.shared.active.lock(|active| {
                for (a, b) in active.iter_mut().zip(b) {
                    (a.run, a.biquad) = b;
                }
            });

            let feedforward = settings.build_feedforward();
            c.shared.feedforward.lock(|f| *f = feedforward);

            if let Some(dac) = c.local.current_sense_dac.as_mut() {
                dac.write_voltage(settings.frontend_offset);
            }
            c.local.aux_dac.set_voltage(
                settings.feedback_offset / FEEDBACK_OFFSET_CURRENT_PER_VOLT,
            );

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
                        settings.current_sense.ch.each_ref().map(|ch| ch.gain),
                        settings.current_sense.telemetry_period,
                    )
                });

            let telemetry = telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            );

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
