//! # Dual IIR with Mains-Synchronised Harmonic Feedforward
//!
//! This application implements a dual-channel real-time DSP pipeline on Stabilizer.
//! Each channel samples analog input data at a fixed rate, applies cascaded IIR filtering,
//! optionally injects harmonic feedforward components via DDS, and generates filtered
//! output signals on the respective DAC outputs.
//!
//! In addition, the application includes a digital PLL that synchronizes the generated
//! fundamental frequency to the mains using hardware timestamp capture.
//!
//! ---
//!
//! ## Core Signal Path (per channel)
//!
//! ADC → Harmonic DDS Injection → Cascaded IIR Filter(s) → DAC
//!
//! - Batch-based DMA processing
//! - Deterministic latency
//! - f32 biquad filter implementation
//!
//! ---
//!
//! ## Mains Synchronisation
//!
//! - Hardware timestamp capture (TIM5) of the mains reference on DI0
//! - Reciprocal PLL ([`idsp::RPLL`]) advanced once per sample batch
//! - Fundamental phase and frequency taken from the PLL every batch
//! - Harmonics derived coherently as integer multiples of the fundamental phase
//!
//! ---
//!
//! ## Additional Features
//!
//! * Two independent channels
//! * Up to ~400–800 kHz sample rate (configuration dependent)
//! * Runtime IIR filter configuration
//! * Harmonic feedforward (up to `MAX_HARMONICS`)
//! * Optional DC offset via Current Sense DAC
//! * Feedback offset current via auxiliary (MCU-internal) DAC output 0
//! * Ethernet UDP data streaming (ADC + DAC)
//! * Telemetry publishing
//! * Deterministic DMA-driven timing
//!
//! ---
//!
//! ## Settings
//! Refer to the [`Settings`] structure for documentation of runtime-configurable
//! parameters including:
//! - AFE gain
//! - IIR coefficients
//! - Harmonic amplitudes and phases
//! - PLL time constants
//! - Current sense DAC offset
//! - Feedback offset current
//!
//! ## Telemetry
//! Refer to [`Telemetry`] for information about reported measurements.
//!
//! ## Livestreaming
//!
//! Raw ADC and DAC data are streamed over UDP.
//! See [`stabilizer::net::data_stream`] for details.
//! 
//! # Notes
//! Can only lock to one external phase reference and cannot independently phase-lock channel 0 and 1 to different triggers
//! Can not run two separate PLL for each channel
//! Can not run two separate PLL for each channel
//! Cannot run two different fundamental frequencies per channel while using one trigger
//! CANNOT have channel 0 locked to mains A and channel 1 locked to mains B NOR e.g. channel 0 free running and channel 1 phase locked
//! If want this - need two timestamp inputs, two independent frequency corr and two independent PLL loops
//!
//! Cannot mix between channels i.e ADC0/DAC1
//! 
//! ASSUME THAT THIS IS RUNNING ON 1 CHANNEL ONLY AT A TIME - BUT LEFT IN DUAL CHANNEL CAPABILITIES FOR POTENTIALLY EASIER ADAPTION IN THE FUTURE
//! AND SO CAN EASILY SWITCH BETWEEN CHANNELS IF FAULT WITH ONE
#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::sync::atomic::{fence, Ordering};
use core::usize;
use serde::{Deserialize, Serialize};
use fugit::ExtU64;
use mutex_trait::prelude::*;
use idsp::{iir, RPLL};
use stabilizer::app_utils::harmonic_dds::{BasicConfig};
use stabilizer::{
    app_utils::harmonic_dds::{HarmonicGenerator},
    hardware::{
        self,
        
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        aux_dac::{AuxDac0, AuxDacOutput},
        dac::{Dac0Output, Dac1Output, DacCode},
        hal,
        input_stamper::InputStamper,
        current_sense_dac::CurrentSenseDac,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, SerialTerminal, SystemTimer, Systick,
        UsbDevice, AFE0, AFE1,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Tree,
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};


// Maximum magnitude of a signed 16-bit signal (used for normalization/scaling)
const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// log2 of the number of samples processed per batch.
const BATCH_SIZE_LOG2: u32 = 3;

// Number of samples processed per batch.
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

// log2 of the number of 100 MHz timer ticks between samples.
// SAMPLE_TICKS = 2^SAMPLE_TICKS_LOG2
// Example: 8 → 256 ticks → 2.56 µs/sample → ~390.625 kHz sample rate.
// Note dual-iir was original 7 - changed to 8 as otherwise too fast for computing harmonics
const SAMPLE_TICKS_LOG2: u32 = 8;

// log2 of the number of timestamp timer ticks between PLL updates (one per batch).
const PLL_DT2: u32 = SAMPLE_TICKS_LOG2 + BATCH_SIZE_LOG2;

// Minimum spacing between accepted mains timestamps, in timestamp timer ticks.
// Captures closer than this to the previous accepted one are treated as glitches.
const MIN_TIMESTAMP_SPACING_TICKS: u32 =
    (0.005 / hardware::design_parameters::TIMER_PERIOD) as u32;

// Number of timer ticks between consecutive samples.
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

// Sampling period in seconds.
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

// Nominal mains frequency (Hz).
const MAINS_FREQUENCY: f32 = 50.0;

// Feedback offset current per auxiliary DAC output voltage (A/V), fixed by the current sense
// board design (independent of the shunt chosen for the particular application).
const FB_OFFSET_CURRENT_PER_VOLT: f32 = 0.0235;

// Maximum number of harmonics used in feedforward control.
const MAX_HARMONICS: usize = 5;

// Application-level harmonic parameters
//
// Represents the amplitude and phase of a single harmonic component
// This struct is specific to this application - define locally
#[derive(Copy, Clone, Debug, Serialize, Deserialize, Tree)]
pub struct HarmonicWaveParameters {
    // Harmonic amplitude in controller units
    // Uses f32 to allow fractional values
    pub amp: f32,
    // Harmonic phase in degree - can be [-360, 360) from UI
    pub phase: f32,
}
impl Default for HarmonicWaveParameters {
    fn default() -> Self {
        Self {
            // Zero amplitude  (disabled harmonic)
            amp: 0.0,
            // Zero phase offset
            phase: 0.0,
        }
    }
}


#[derive(Clone, Copy, Debug, Tree)]
pub struct Settings{
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
    afe: [Gain;2],

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    /// * `<m>` specifies which cascade to configure. `<m>` := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// See [iir::Biquad]
    #[tree(depth(2))]
    iir_ch: [[iir::Biquad<f32>; IIR_CASCADE_LENGTH];2],

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

    /// Specifies the config for signal generators to add on to DAC0/DAC1 outputs.
    ///
    /// # Path
    /// `harmonic_wave_parameters/<n>`
    /// 
    /// * `<n>` specifies which harmonic order to configure. `<n>` := [0,1,2,3,4]
    ///
    ///
    /// # Value
    /// See [HarmonicWaveParameters]
    #[tree(depth(2))]
    harmonic_wave_parameters: [[HarmonicWaveParameters;MAX_HARMONICS];2],

    /// DC voltage offset applied via Current Sense DAC
    /// 
    /// This value is written directly to the `CurrentSenseDac`:
    /// 
    /// `dac.write_voltage(v_offset)`
    /// 
    /// Used to shift the analog output baseline
    /// 
    /// # Path
    /// `v_offset`
    /// 
    /// # Value
    /// Voltage in volts (clamped internally to Current Sense DAC range)
    v_offset: f32,

    /// Feedback offset current, generated from auxiliary DAC output 0.
    ///
    /// The auxiliary DAC output voltage is converted to a current at
    /// [FB_OFFSET_CURRENT_PER_VOLT] by the current sense board, giving a range of
    /// 0 to ~48 mA for the 0 V to 2.048 V DAC output range.
    ///
    /// # Path
    /// `fb_offset`
    ///
    /// # Value
    /// Current in amperes (clamped to the output range).
    fb_offset: f32,

    /// Mains PLL time constants.
    ///
    /// # Path
    /// `pll_tc/<n>`
    ///
    /// * `<n>` := 0 selects the frequency lock settling time.
    /// * `<n>` := 1 selects the phase lock settling time (usually one less than `<n>` := 0).
    ///
    /// # Value
    /// The settling time exponent: `1 << pll_tc[n]` is the settling time in 100 MHz
    /// timestamp timer ticks. The frequency settling time must be longer than one mains
    /// period (`pll_tc[0] >= 22` at 50 Hz). Both must exceed the batch period
    /// exponent ([PLL_DT2]) and be at most `PLL_DT2 + 31`. Out-of-range values are clamped.
    pll_tc: [u32; 2],
}

impl Default for Settings{
    fn default() -> Self {
        // Unity-gain identity biquad
        let mut i = iir::Biquad::IDENTITY;
        i.set_min(-SCALE);
        i.set_max(SCALE);

        Self{
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[i; IIR_CASCADE_LENGTH]; 2],
            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream_target: StreamTarget::default(),

            // No harmonic injection on start up
            harmonic_wave_parameters:[[HarmonicWaveParameters::default(); MAX_HARMONICS], [HarmonicWaveParameters::default(); MAX_HARMONICS]],

            // No DC offset
            v_offset: 0.0,

            // 11 mA feedback offset current.
            fb_offset: 0.011,

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
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {

    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        /// USB device stack (used in idle + USB task)
        usb: UsbDevice,
        /// Network stack and configuration interface (Miniconf).
        network: NetworkUsers<Settings, Telemetry, 3>,
        /// Runtime application settings (updated via network).
        settings: Settings,
        /// Telemetry buffer updated in DSP task and published periodically.
        telemetry: TelemetryBuffer,
        /// Harmonic DDS generators (one per channel).
        ///
        /// Updated in:
        /// - DSP task (sample generation)
        /// - PLL task (frequency adjustment)
        /// - Settings update task (reconfiguration)
        harmonic_generators: [HarmonicGenerator<MAX_HARMONICS>;2],
    }



    #[local]
    struct Local {
        /// USB serial terminal handler.
        usb_terminal: SerialTerminal,
        /// Hardware timer driving ADC/DAC sampling.
        sampling_timer: SamplingTimer,
        /// Digital inputs (DI0, DI1).
        digital_inputs: (DigitalInput0, DigitalInput1),
        /// Analog Front-End gain controls.
        afes: (AFE0, AFE1),
        /// ADC input interfaces.
        adcs: (Adc0Input, Adc1Input),
        /// DAC output interfaces.
        dacs: (Dac0Output, Dac1Output),
        /// Input timestamp capture for mains synchronisation.
        timestamper: InputStamper,
        /// IIR filter state memory:
        /// [channel][cascade_stage][delay_elements]
        iir_state: [[[f32; 4]; IIR_CASCADE_LENGTH]; 2],
        /// UDP frame generator for livestreaming.
        generator: FrameGenerator,
        /// Internal CPU temperature sensor.
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        /// Optional Current Sense DAC driver (present if board detected).
        current_sense_dac: Option<CurrentSenseDac>,
        /// Auxiliary DAC output setting the feedback offset current.
        aux_dac: AuxDac0,
        /// Reciprocal PLL tracking the mains reference from TIM5 timestamps.
        pll: RPLL,
        /// Last accepted mains timestamp (TIM5 capture), used for glitch rejection.
        last_ts: Option<u32>,
    }

    /// System init
    /// 
    /// Responsisble for:
    ///     - Configure hardware peripherals
    ///     - Init network stack and streaming
    ///     - Create harmonic DDS generators
    ///     - Create the mains PLL
    ///     - Start background tasks and interrupts
    /// 
    /// No real-time DSP runs until `start` task enables sampling timer
    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {

        // Create system time source used by network stack and scheduling.
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        // Perform board-level hardware setup.
        // Depending on detected hardware, SPI1 may be configured
        // for Pounder or Current Sense DAC.
        let (mut stabilizer, _pounder, current_sense_dac) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let device_settings = stabilizer.usb_serial.settings();
        // Load default application settings (may be overridden via network).
        let application_settings = Settings::default();

        // Initialize networking (Miniconf + telemetry + streaming).
        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &device_settings.broker,
            &device_settings.id,
            stabilizer.metadata,
            application_settings,
        );

        // Configure UDP livestream format (ADC + DAC samples).
        let generator = network.configure_streaming(StreamFormat::AdcDacData);
        
        // Initialize harmonic DDS generators for each channel.
        // Fundamental starts at MAINS_FREQUENCY.
        // Phases converted from degrees → cycles.  
        let harmonic_generators_arr = core::array::from_fn(|channel| {
            let basic_cfg = BasicConfig::<MAX_HARMONICS> {
                        zero_order_frequency: MAINS_FREQUENCY,
                        amplitude: core::array::from_fn(|i| {
                                    application_settings.harmonic_wave_parameters[channel][i].amp
                                }),
                        phase: core::array::from_fn(|i| {application_settings.harmonic_wave_parameters[channel][i].phase/360.0}),
            };

            let cfg = basic_cfg.try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE).unwrap();

            HarmonicGenerator::new(cfg)
        });

        // Shared resources accessed across RTIC tasks.
        let shared = Shared {
            usb: stabilizer.usb,
            network,
            settings: application_settings,
            telemetry: TelemetryBuffer::default(),
            harmonic_generators: harmonic_generators_arr,
        };
        // Local task-owned resources (not shared across tasks).
        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            timestamper: stabilizer.timestamper,
            iir_state: [[[0.; 4]; IIR_CASCADE_LENGTH]; 2],
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            current_sense_dac,
            aux_dac: {
                // Set the initial output before enabling to avoid glitching through 0 V. Aux DAC 1
                // is not used by the current sense board, so is left disabled.
                let mut dac = stabilizer.aux_dacs.0;
                dac.set_voltage(application_settings.fb_offset / FB_OFFSET_CURRENT_PER_VOLT);
                dac.enable()
            },
            pll: RPLL::new(PLL_DT2),
            last_ts: None,
        };

        // Apply initial DC offset to Current Sense DAC (if present).
        if let Some(dac) = local.current_sense_dac.as_mut() {
            dac.write_voltage(application_settings.v_offset);
        }

        // Enable ADC and DAC peripherals (DMA-driven).
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();


        // Schedule background tasks.
        // Order does not matter here since sampling has not started yet.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
        start::spawn_after(100.millis()).unwrap();
        
        // Start recording mains reference timestamps on DI0 (PLL input).
        stabilizer.timestamp_timer.start();
        local.timestamper.start();

        (shared, local, init::Monotonics(stabilizer.systick))
    }

    //Start the sampling ADCs and DACS Timers
    #[task(priority = 1, local=[sampling_timer])]
    fn start(c: start::Context){
        c.local.sampling_timer.start();
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
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator, timestamper, pll, last_ts], shared=[settings, telemetry, harmonic_generators], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context){

        // Shared and local resources
        let process::SharedResources {
            settings,
            telemetry,
            harmonic_generators,
        } = c.shared;        
        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
            timestamper,
            pll,
            last_ts,
        } = c.local;

        // Real-time DSP work must be bounded; lock shared resources only for the minimum time.
        // Acquire shared locks and run the per-batch pipeline.
        (settings, telemetry, harmonic_generators).lock(
            |settings, telemetry, harmonic_generators| {
                
                // Snapshot digital inputs (DI0, DI1) and publish to telemetry.
                let digital_inputs = [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                telemetry.digital_inputs = digital_inputs;

                // Compute hold state used by the filter stage:
                // - force_hold overrides everything
                // - otherwise DI1 (when allow_hold is enabled) will hold filter state
                let hold = settings.force_hold
                    || (digital_inputs[1] && settings.allow_hold);

                // Mains synchronisation.
                //
                // Fetch the latest TIM5 capture of the mains reference edge (if any arrived
                // since the previous batch). Timestamps from capture overflows are ignored, as
                // are captures too close to the previously accepted one (glitches).
                let timestamp = timestamper
                    .latest_timestamp()
                    .unwrap_or(None)
                    .filter(|ts| {
                        last_ts.map_or(true, |last| {
                            ts.wrapping_sub(last) >= MIN_TIMESTAMP_SPACING_TICKS
                        })
                    });
                if timestamp.is_some() {
                    *last_ts = timestamp;
                }

                // Advance the PLL by one batch. The returned phase is the reference phase at
                // the start of this batch (zero at the reference edge) and the frequency is
                // in units of 1 << 32 per batch.
                let (pll_phase, pll_frequency) = pll.update(
                    timestamp.map(|t| t as i32),
                    settings.pll_tc[0],
                    settings.pll_tc[1],
                );
                let phase_increment = (pll_frequency >> BATCH_SIZE_LOG2) as i32;

                // Re-anchor every harmonic generator to the tracked fundamental. The
                // generators then advance sample-by-sample within the batch.
                for harmonic_generator in harmonic_generators.iter_mut() {
                    harmonic_generator.set_fundamental(pll_phase, phase_increment);
                }

                // Lock ADC/DAC DMA buffers (local hardware resources) for the duration of processing.   
                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
 
                    // References to the per-channel sample buffers (each is a batch of i16 samples).
                    let adc_samples = [adc0, adc1];
                    let dac_samples = [dac0, dac1];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);


                    // For each channel, mix DDS harmonic feedforward with ADC sample,
                    // pass result through cascaded IIR(s), and write to DAC buffer.
                    for channel in 0..adc_samples.len() {
                        // Iterate sample-by-sample for this batch. We zip:
                        //  - input ADC samples (ai)
                        //  - mutable DAC output slots (di)
                        //  - a mutable harmonic generator for this channel

                        adc_samples[channel]
                        .iter()
                        .zip(dac_samples[channel].iter_mut())
                        .zip(&mut harmonic_generators[channel])
                        .for_each(|((ai, di), harmonic)|{

                            let ai_i16 = *ai as i16;

                            // harmonic: i16 feedforward sample produced by DDS iterator
                            // Mix feedforward with ADC input (saturating add to avoid overflow).
                            let mixed = ai_i16.saturating_add(harmonic);
                            
                            let x = f32::from(mixed);
                            // Apply cascaded IIR stages. If `hold` is active, use the HOLD filter
                            // (which returns the previous output) instead of the configured stage.
                            //
                            // Each stage is applied in sequence, feeding the next stage.
                            let y = settings.iir_ch[channel].iter().zip(iir_state[channel].iter_mut()).fold(x, |yi, (ch, state)|{
                                let filter = if hold { &iir::Biquad::HOLD} else { ch };
                                filter.update(state, yi)
                            });

                            // Convert filtered f32 back to i16. `to_int_unchecked` is used for
                            // performance; ensure upstream filters clamp outputs to safe range.
                            let y_i16: i16 = unsafe{ y.to_int_unchecked() };

                            // Store converted code into the DAC DMA buffer (driver expects DacCode raw).
                            *di = DacCode::from(y_i16).0;
                        });
                
                    }

                    // Add batch to UDP stream buffer:
                    // Copy adc0, adc1, dac0, dac1 (in that order) into the provided `buf`.
                    const N: usize = BATCH_SIZE * core::mem::size_of::<i16>();
                    generator.add(|buf| {
                        // copy adc0, adc1, dac0, dac1 in that order from adc_samples/dac_samples arrays
                        for (data, buf) in adc_samples
                            .iter()
                            .chain(dac_samples.iter())
                            .zip(buf.chunks_exact_mut(N))

                        {
                            let data = unsafe {
                                core::slice::from_raw_parts(
                                    data.as_ptr() as *const MaybeUninit<u8>,
                                    N,
                                )
                            };
                            buf.copy_from_slice(data)
                        }
                        // Return number of bytes written for all four buffers
                        N * 4
                    });
                    
                    // Update snapshot telemetry (first sample of each buffer)
                    telemetry.adcs = [
                        AdcCode(adc_samples[0][0]),
                        AdcCode(adc_samples[1][0]),
                        
                    ];
                    telemetry.dacs = [
                        DacCode(dac_samples[0][0]),
                        DacCode(dac_samples[1][0]),

                    ];
                    // Ensure memory/ordering before releasing locks so DMA flags are observed consistently.
                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    /// Idle task.
    ///
    /// Polls the network for updates and spawns a settings update
    /// task when configuration changes are detected.
    ///
    /// Enters low-power sleep (WFI) when USB is suspended.
    #[idle(shared=[network, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                // New settings arrived via network — apply them.
                NetworkState::SettingsChanged(_path) => {
                    // Spawn RTIC task to apply the settings and run as priority 1
                    settings_update::spawn().unwrap()
                }
                // If network has updated internal state but no settings change - just keep running
                NetworkState::Updated => {}
                // Nothing new from the network
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
    
    

    // Apply configuration received from the network (Miniconf).
    //
    // Responsibilities:
    // - Pull the latest settings from the network-miniconf
    // - Update local copy of settings
    // - Apply hardware changes (AFE gains, Current Sense DAC, feedback offset aux DAC)
    // - Reconfigure harmonic DDS generators (phases are made relative to fundamental)
    // - Update livestream target
    #[task(priority = 1, local=[afes, current_sense_dac, aux_dac], shared=[network, settings, harmonic_generators])]
    fn settings_update(mut c: settings_update::Context) {
        // Read configuration from network miniconf and store in shared settings.
        let mut settings = c.shared.network.lock(|net| *net.miniconf.settings());

        // Keep the PLL time constants within the range supported by `RPLL::update`:
        // both shifts must exceed the batch period exponent and stay within 31 of it.
        for tc in settings.pll_tc.iter_mut() {
            let clamped = (*tc).clamp(PLL_DT2 + 1, PLL_DT2 + 31);
            if clamped != *tc {
                log::warn!("PLL time constant {} out of range, clamped to {}", *tc, clamped);
                *tc = clamped;
            }
        }

        c.shared.settings.lock(|current| *current = settings);
        
        // Apply AFE gain settings.
        c.local.afes.0.set_gain(settings.afe[0]);
        c.local.afes.1.set_gain(settings.afe[1]);

        // Apply DC offset to Current Sense DAC if present.
        if let Some(dac) = c.local.current_sense_dac.as_mut() {
            dac.write_voltage(settings.v_offset);
        }

        // Apply feedback offset current via the auxiliary DAC.
        c.local.aux_dac.set_voltage(settings.fb_offset / FB_OFFSET_CURRENT_PER_VOLT);

        // Update harmonic DDS generators.
        // Phases in the UI are specified in degrees. We convert them to cycles
        // (0.0..1.0) and make each harmonic phase relative to the fundamental
        // (harmonic[0]) so the UI can set a per-harmonic offset.
        let harmonic_parameters = &settings.harmonic_wave_parameters;
        
        for channel in 0..harmonic_parameters.len(){

            let fundamental_phase = harmonic_parameters[channel][0].phase / 360.0;

            // Build BasicConfig with amplitudes and phases (relative to fundamental).
            let basic_cfg = BasicConfig::<MAX_HARMONICS> {
                    zero_order_frequency: MAINS_FREQUENCY,
                    amplitude: core::array::from_fn(|i| {
                                harmonic_parameters[channel][i].amp
                            }),
                    phase: core::array::from_fn(|i| {
                        // Convert degrees to cycles and subtract the fundamental to produce a relative phase.
                        let phi = harmonic_parameters[channel][i].phase / 360.0;
                        let mut rel = phi - fundamental_phase;
                        // Wrap into [0.0, 1.0)
                        if rel >= 1.0 {
                            rel -= 1.0;
                        }
                        if rel < 0.0 {
                            rel += 1.0;
                        }
                        rel
                       
                    }),
                };
            
            // Convert user-facing BasicConfig into the internal fixed-point Config.
            match basic_cfg.try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE) {
                Ok(config)=> {c.shared.harmonic_generators.lock(|harmonic_generator| harmonic_generator[channel].update_waveform(config));}
                Err(err) => log::error!(
                    "Failed to update harmonic generation on channel{}: {:?}",channel,
                    err
                ),
            }
        }

        // Update the streaming target (UDP) based on settings.
        let target = settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));
    }


    //Telemetry update
    #[task(priority = 1, shared=[network, settings, telemetry], local=[cpu_temp_sensor])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period) = 
            (c.shared.settings).lock(|settings| { 
                (
                    settings.afe,
                    settings.telemetry_period,
                )
            });

        c.shared.network.lock(|net| {
            net.telemetry.publish(&telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
                None,
            ))
        });

        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }


    //USB UPDATE    
    #[task(priority = 1, shared=[usb], local=[usb_terminal])]
    fn usb(mut c: usb::Context) {
        // Handle the USB serial terminal.
        c.shared.usb.lock(|usb| {
            usb.poll(&mut [c.local.usb_terminal.interface_mut().inner_mut()]);
        });

        c.local.usb_terminal.process().unwrap();

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

    #[task(binds = SPI1, priority = 4)]
    fn spi1(_: spi1::Context) {
        panic!("Current Sense DAC error");
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
