//! # Dual IIR with mains-synchronised harmonic feedforward
//!
//! The `current_sense` application is a low-latency variant of `dual-iir` for the current sense
//! mezzanine. Stabilizer samples the inputs at a fixed rate, digitally filters each sample as
//! soon as it is available, and then generates filtered output signals on the respective channel
//! outputs. On channel 0, a harmonic feedforward waveform synchronised to the mains is added to
//! the input.
//!
//! ## Features
//! * Two independent channels
//! * 781.25 kHz rate, timed sampling
//! * Per-sample processing for low input-to-output latency
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
//! ## Sample processing
//! Each sample is processed as in [Dual IIR, low latency](../dual_iir_ll/index.html), which
//! also describes the timing, the limits and the timing telemetry. The feedforward adds an
//! addition to the latency critical path of channel 0. Its value for the next sample is
//! computed after the DAC codes have been written.
//!
//! ## Mains synchronisation
//! The rising edges of the mains reference on DI0 are timestamped in hardware. The timestamp
//! timer is started together with the sampling timer, so the time of each sample is known in
//! its ticks. A reciprocal PLL ([idsp::RPLL]) tracks the mains phase from the timestamps. It is
//! updated on a grid of 10.24 µs ([stabilizer::mains]) by a lower priority task, which catches
//! up on it every 64 samples (81.92 µs). That task then expands the feedforward waveform around
//! an upcoming sample to second order in the sample index
//! ([stabilizer::harmonics::Expansion]), and the sample processing evaluates this expansion for
//! each sample. The harmonics are thus coherent with the reference, and the phase applies to the
//! sampling instant of each sample. The expansion is accurate to better than 1e-4 of the
//! amplitude of the 5th harmonic.
//!
//! The feedforward is only available on channel 0, matching the single-channel current sense
//! hardware; channel 1 is a regular `dual-iir` channel. Conversely, as DI0 is taken by the mains
//! reference, the external run/hold control is only available on channel 1. It responds once per
//! batch of eight samples.
//!
//! ## Settings
//! Refer to the [CurrentSense] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry](stabilizer::dual_iir::Telemetry) for information about telemetry reported
//! by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP in batches of eight samples. Refer to
//! [stream] for more information.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use miniconf::{Leaf, Tree};

use idsp::{RPLLConfig, iir::pid::Units};

use platform::{AppSettings, NetSettings};
use serde::{Deserialize, Serialize};
use stabilizer::{
    convert::{AdcCode, DacCode},
    dual_iir::{Active, Channel, Run, Times},
    harmonics::{Expansion, Harmonic, HarmonicWaveform},
};

// The number of samples in each streamed batch
const BATCH_SIZE: usize = 8;

// Timer ticks at 100 MHz: 128 ticks give 781.25 kHz. See `dual-iir-ll` for the limits.
// Non-power-of-two periods are supported.
const SAMPLE_TICKS: u32 = 128;
const _: () = assert!(
    SAMPLE_TICKS >= 100 && SAMPLE_TICKS <= u16::MAX as u32,
    "current_sense requires 100..=65535 timer ticks per sample"
);
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * stabilizer::design_parameters::TIMER_PERIOD;

const UNITS: Units<f32> = Units {
    t: SAMPLE_PERIOD,
    x: AdcCode::VOLT_PER_LSB,
    y: DacCode::VOLT_PER_LSB,
};

// The logarithm of the number of timestamp timer ticks between PLL updates (10.24 µs).
const PLL_DT2: u8 = 10;

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

// The number of samples between updates of the mains PLL and the feedforward expansion (eight
// batches, 81.92 µs). Updating every batch costs about 9 % of the CPU time, which the stream
// and the network then lack. Over this span, the expansion stays accurate to about 4e-5 of the
// amplitude of the 5th harmonic (see `HarmonicWaveform::expand()`).
const FEEDFORWARD_UPDATE: u32 = 8 * BATCH_SIZE as u32;

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
    ///
    /// `External` run/hold control is not available on channel 0, as digital input 0 is taken
    /// by the mains reference; it is replaced by `Hold` there.
    ch: [Channel; 2],
    /// Trigger both signal sources
    #[tree(with=miniconf::leaf)]
    trigger: bool,
    /// Harmonic feedforward waveform added to the channel 0 input.
    ///
    /// The waveform is `sum(amp[n] * sin(2π * ((n + 1) * theta + phase_turns[n])))` over all
    /// `n`, where `theta` is the mains phase in turns tracked by the PLL (zero at the reference
    /// edge) at the time of the sample. The index `n` selects the harmonic of order `n + 1`,
    /// i.e. the fundamental comes first.
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

/// The mains feedforward for channel 0
#[derive(Clone, Debug)]
pub struct Feedforward {
    /// Mains PLL configuration
    pll: RPLLConfig,
    /// Feedforward waveform as a function of the mains phase, in ADC codes
    waveform: HarmonicWaveform<MAX_HARMONICS>,
}

/// The state of the sample processing routine.
///
/// It is shared with the batch, settings update and telemetry tasks.
pub struct State {
    ch: [Active; 2],
    /// The feedforward waveform, expanded around a recent sample by the batch task.
    feedforward: Expansion,
    /// The feedforward for the next channel 0 sample in ADC LSB.
    next_feedforward: f32,
    times: Times,
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
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC]))]
mod app {
    use super::*;
    use core::sync::atomic::{AtomicBool, Ordering, fence};
    use cortex_m::peripheral::DWT;
    use fugit::ExtU32 as _;
    use rtic_monotonics::Monotonic;

    use stabilizer::{
        dual_iir::Telemetry,
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
            timers::{SamplingTimer, TimestampTimer},
        },
        mains::MainsPll,
        telemetry::TelemetryBuffer,
    };
    use stream::{FrameGenerator, Pop, SampleRing};

    // The most recent samples, handed from `process` to `batch` and `telemetry`.
    static RING: SampleRing<512> = SampleRing::new();

    // The digital input states, sampled by `batch` once per batch.
    static DI: [AtomicBool; 2] =
        [AtomicBool::new(false), AtomicBool::new(false)];

    /// The time of the sample with the given index in timestamp timer ticks.
    ///
    /// The timestamp timer is started together with the sampling timer (see `start`), so this
    /// is exact up to a constant offset.
    fn sample_time(index: u32) -> u32 {
        index.wrapping_mul(SAMPLE_TICKS)
    }

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<CurrentSense>,
        settings: Settings,
        state: State,
        feedforward: Feedforward,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        timestamp_timer: TimestampTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        timestamper: InputStamper,
        mains: MainsPll,
        afes: [Pgia; 2],
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
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
                env!("CARGO_BIN_NAME"),
                hardware::Sampling::LowLatency,
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
            state: State {
                ch: stabilizer
                    .settings
                    .current_sense
                    .ch
                    .each_ref()
                    .map(|ch| Active::new(ch, &UNITS)),
                feedforward: Default::default(),
                next_feedforward: 0.0,
                times: Default::default(),
            },
            feedforward: stabilizer.settings.current_sense.build_feedforward(),
            settings: stabilizer.settings,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.sampling_timer,
            timestamp_timer: stabilizer.timestamp_timer,
            digital_inputs: stabilizer.digital_inputs,
            timestamper: stabilizer.input_stamper,
            mains: MainsPll::new(MIN_TIMESTAMP_SPACING_TICKS),
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
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

        // Record mains reference timestamps on DI0. The timestamp timer starts counting with
        // the sampling timer.
        local.timestamper.start();

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
        start::spawn().unwrap();

        (shared, local)
    }

    #[task(priority = 1, local=[sampling_timer, timestamp_timer])]
    async fn start(c: start::Context) {
        Systick::delay(100.millis()).await;
        // Start sampling ADCs and DACs. The timestamp timer starts at the same time, so that
        // the time of each sample is known (see `sample_time()`).
        cortex_m::interrupt::free(|_| {
            c.local.timestamp_timer.start();
            c.local.sampling_timer.start();
        });
    }

    /// Main DSP processing routine.
    ///
    /// Runs once per sample, as soon as the ADC1 sample is available in its SPI receive FIFO,
    /// as in `dual-iir-ll`. On top of that, the feedforward is added to the ADC0 sample. Its
    /// value is computed from the expansion provided by `batch` after the DAC codes have been
    /// written, for the next sample.
    ///
    /// # Note
    /// Processing should complete within one sample period. Samples that arrive earlier wait
    /// in the ADC SPI receive FIFOs; at the end of each batch, a pending sample is counted as
    /// late. If those FIFOs overflow, firmware will panic due to an ADC SPI error.
    #[task(binds = SPI3, local = [adcs, dacs], shared = [state], priority = 3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(mut c: process::Context) {
        let start = DWT::cycle_count();

        let (adc0, adc1) = c.local.adcs;
        let (dac0, dac1) = c.local.dacs;

        c.shared.state.lock(|state| {
            let State {
                ch: [ch0, ch1],
                feedforward,
                next_feedforward,
                times,
            } = state;

            // The latency critical part: write each DAC code as soon as it is available.
            // ADC0 is sampled before ADC1, whose interrupt this is.
            let adc0 = adc0.read();
            let (dac0_code, sample0) =
                ch0.process(&DI[0], f32::from(adc0 as i16) + *next_feedforward);
            dac0.write(dac0_code);
            let adc1_code = adc1.read();
            let (dac1_code, sample1) =
                ch1.process(&DI[1], f32::from(adc1_code as i16));
            dac1.write(dac1_code);
            let output = DWT::cycle_count();

            // Prepare the filters for the next sample.
            ch0.update(sample0);
            ch1.update(sample1);

            // Hand the sample to the batch task and wake it once per batch.
            let count = RING.push([adc0, adc1_code, dac0_code, dac1_code]);
            let batch_end = count.is_multiple_of(BATCH_SIZE as u32);
            if batch_end {
                rtic::pend(hal::stm32::Interrupt::LTDC);
            }

            // The index of the next sample is the number of samples so far.
            *next_feedforward = feedforward.sample(count);

            ch0.update_source();
            ch1.update_source();

            // Detect SPI errors and backlog (e.g. after a settings update) once per batch.
            // An SPI error keeps the interrupt pending, so it is detected within a batch.
            let late = batch_end && adc1.check();

            times.update(
                output.wrapping_sub(start),
                DWT::cycle_count().wrapping_sub(start),
                late,
            );
        });
    }

    /// Track the mains phase, sample the digital inputs, and assemble stream frames from the
    /// samples handed over by `process`.
    ///
    /// Pended by `process` once per batch of samples. This keeps the PLL, the evaluation of the
    /// harmonics, the frame generation (allocation, header, copy, queueing) and the GPIO
    /// accesses out of the sample processing. The PLL and the harmonics are only updated every
    /// `FEEDFORWARD_UPDATE` samples.
    ///
    /// LTDC is a spare vector pended in software; the display peripheral is unused.
    /// A direct binding avoids async executor overhead. It runs below DSP and above
    /// network/USB. Wakeups can coalesce: the PLL catches up on its grid of timer ticks, and
    /// this task drains the sample ring.
    #[task(
        binds = LTDC,
        local = [
            digital_inputs, timestamper, mains, generator,
            tail: u32 = 0, next_update: u32 = 0],
        shared = [state, feedforward],
        priority = 2)]
    fn batch(mut c: batch::Context) {
        let batch::LocalResources {
            digital_inputs,
            timestamper,
            mains,
            generator,
            tail,
            next_update,
            ..
        } = c.local;

        let head = RING.head();
        if head.wrapping_sub(*next_update) as i32 >= 0 {
            *next_update = head.wrapping_add(FEEDFORWARD_UPDATE);

            // Read the number of samples before the timestamp: by then, all mains edges up to
            // the latest processed sample have been captured. Timestamps from capture
            // overflows are ignored.
            fence(Ordering::SeqCst);
            if let Ok(Some(timestamp)) = timestamper.latest_timestamp() {
                mains.push(timestamp);
            }

            // Advance the PLL, and expand the feedforward waveform around the middle of the
            // samples it is used for until the next update: from the next sample to about
            // `FEEDFORWARD_UPDATE` samples later.
            let centre = head.wrapping_add(FEEDFORWARD_UPDATE / 2);
            let feedforward = c.shared.feedforward.lock(|feedforward| {
                let estimate = mains.advance(
                    &feedforward.pll,
                    sample_time(head.wrapping_sub(1)),
                );
                feedforward.waveform.expand(
                    centre,
                    estimate.phase_at(sample_time(centre)),
                    estimate.step(SAMPLE_TICKS as i32),
                )
            });
            c.shared.state.lock(|state| state.feedforward = feedforward);
        }

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

    #[task(priority = 1, local=[afes, current_sense_dac, aux_dac], shared=[network, settings, state, feedforward])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            let settings = &mut settings.current_sense;
            settings.validate();

            c.local.afes[0].set_gain(settings.ch[0].gain);
            c.local.afes[1].set_gain(settings.ch[1].gain);

            if settings.trigger {
                settings.trigger = false;
                let s =
                    settings.ch.each_ref().map(|ch| ch.build_source(&UNITS));
                c.shared.state.lock(|state| {
                    for (ch, s) in state.ch.iter_mut().zip(s) {
                        if let Ok(s) = s {
                            ch.set_source(s);
                        }
                    }
                });
            }
            let b = settings.ch.each_ref().map(|ch| ch.build_filter(&UNITS));
            c.shared.state.lock(|state| {
                for (a, (run, biquad)) in state.ch.iter_mut().zip(b) {
                    a.set_filter(run, biquad);
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

    #[task(priority = 1, shared=[network, settings, state], local=[cpu_temp_sensor, count: u32 = 0])]
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
                        settings.current_sense.ch.each_ref().map(|ch| ch.gain),
                        settings.current_sense.telemetry_period,
                    )
                });

            let telemetry = Telemetry::new(
                telemetry.finalize(
                    gains[0],
                    gains[1],
                    c.local.cpu_temp_sensor.get_temperature().unwrap(),
                ),
                times,
                count,
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
