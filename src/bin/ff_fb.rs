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
//! - Hardware timestamp capture (TIM5)
//! - Digital phase-locked loop (PI controller)
//! - Frequency correction with integrator clamping (anti-windup)
//! - Harmonics derived from tracked fundamental frequency
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
//! - PLL gains (Kp, Ki)
//! - Current sense DAC offset
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
use idsp::iir;
use stabilizer::app_utils::harmonic_dds::{BasicConfig};
use stabilizer::{
    app_utils::harmonic_dds::{HarmonicGenerator},
    hardware::{
        self,
        
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
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

// Number of samples processed per batch.
const BATCH_SIZE: usize = 8;

// log2 of the number of 100 MHz timer ticks between samples.
// SAMPLE_TICKS = 2^SAMPLE_TICKS_LOG2
// Example: 8 → 256 ticks → 2.56 µs/sample → ~390.625 kHz sample rate.
// Note dual-iir was original 7 - changed to 8 as otherwise too fast for computing harmonics
const SAMPLE_TICKS_LOG2: u8 = 8;

// Number of timer ticks between consecutive samples.
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

// Sampling period in seconds.
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

// Nominal mains frequency (Hz).
const MAINS_FREQUENCY: f32 = 50.0;

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

    /// Proportional gain (Kp) for mains phase-locked loop (PLL)
    /// 
    /// Used in frequency correction:
    /// `freq = MAINS_FREQUENCY + frequency_corr + kp_alpha * phase_error`
    /// 
    /// Controls instantaneous response of PLL
    /// 
    /// # Path
    /// `kp_alpha`
    kp_alpha: f32,
    
    /// Integral gain (Ki) for the mains PLL.
    ///
    /// Used to accumulate long-term phase error:
    /// `frequency_corr += ki_alpha * phase_error`
    ///
    /// The integrator term is clamped to prevent windup.
    ///
    /// Controls steady-state frequency tracking accuracy.
    ///
    /// # Path
    /// `ki_alpha`
    ki_alpha: f32,
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
            
            // No kp or ki for PLL
            kp_alpha: 0.0,
            ki_alpha: 0.0,
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
        /// Last valid mains timestamp (TIM5 capture).
        last_ts: Option<u32>,
        /// PLL integrator state (frequency correction term).
        frequency_corr: f32,
        /// Current DDS fundamental frequency (Hz).
        dds_frequency: f32,
        
    }

    /// System init
    /// 
    /// Responsisble for:
    ///     - Configure hardware peripherals
    ///     - Init network stack and streaming
    ///     - Create harmonic DDS generators
    ///     - Init PLL state
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
            last_ts: None,
            frequency_corr: 0.0,
            dds_frequency: MAINS_FREQUENCY,
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
        
        // Enable TIM5 interrupt for mains capture (PLL input).
        stabilizer.timestamp_timer.start();
        local.timestamper.start();
        unsafe {
            cortex_m::peripheral::NVIC::unmask(
                stabilizer::hardware::hal::stm32::Interrupt::TIM5
            );
        }

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
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[settings, telemetry, harmonic_generators], priority=3)]
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
    
    

    // TIM5 capture interrupt — mains synchronisation (digital PLL).
    //
    // Behaviour:
    // - Reads the latest timestamp captured from the mains input (via `timestamper`).
    // - Rejects random fast captures (jitter).
    // - Back-propagates the DDS phase to the exact capture instant.
    // - Computes a wrapped phase error in cycles ([-0.5, 0.5] = ±180°).
    // - Applies a PI correction (integrator clamped) and updates DDS frequency.
    // - Updates each channel's harmonic generator frequency and applies a global
    //   phase offset (used to align DDS phase to the measured edge).
    //
    #[task(binds = TIM5, priority =2, local=[timestamper, frequency_corr, dds_frequency, last_ts], shared=[settings, harmonic_generators])]
    fn mains_sync(mut c: mains_sync::Context){
        
        // Process all pending capture timestamps.
        //
        // A single TIM5 interrupt may correspond to multiple captured edges
        // (e.g. if captures occur faster than this handler runs).
        //
        // `latest_timestamp()` returns one timestamp at a time, so we loop
        // until no more timestamps are available to ensure the capture FIFO
        // is empty before exiting the interrupt.
        loop{
            match c.local.timestamper.latest_timestamp() {

                Ok(Some(ts)) => {

                    // Reject very close captures (likely jitter or noise).
                    if let Some(last) = *c.local.last_ts {
                        let dt = ts.wrapping_sub(last);
                        // Minimum allowed time between valid captures (5 ms here) - may want to change
                        let min_ticks = (0.005 / hardware::design_parameters::TIMER_PERIOD) as u32;
                        if dt < min_ticks {
                            // If too close to previous timestamp ignore as jitter.
                            continue;
                        }
                    }
                    
                    // Time difference from capture to now (in seconds).
                    let now_ticks = unsafe { (*stm32h7xx_hal::stm32::TIM5::ptr()).cnt.read().bits() };
                    let dt_ticks = now_ticks.wrapping_sub(ts);
                    let dt_s = dt_ticks as f32 * hardware::design_parameters::TIMER_PERIOD;
                    
                    // Read PLL gains atomically from shared settings.
                    let (kp, ki) = c.shared.settings.lock(|s| (s.kp_alpha, s.ki_alpha));

                    // Update all harmonic generators (one per channel)
                    c.shared.harmonic_generators.lock(|gens| {
                        for ch in 0..gens.len(){
                            // Get generator state: current fractional phase (cycles) and
                            // phase increment per sample (in cycles/sample).
                            let (phase_now, phase_increase_per_sample) = gens[ch].get_current_state();
                            
                            // Estimate how many samples have elapsed between the captured
                            // timestamp and 'now', then back-propagate the phase to the edge.
                            let samples_elapsed = dt_s / SAMPLE_PERIOD; 
                            
                            
                            // Phase error: desired phase at edge is 0.0 cycles.
                            let mut phase_at_edge = phase_now - phase_increase_per_sample * samples_elapsed;
                            
                            // Wrap phase value into [0.0, 1.0)
                            if phase_at_edge >= 1.0 {
                                phase_at_edge -= 1.0;
                            }
                            else if phase_at_edge < 0.0 {
                                phase_at_edge += 1.0;
                            }

                            // This was our phase at the actual timestamp!
                            let mut phase_error = 0.0 - phase_at_edge;

                            // Wrap phase error into [-0.5, 0.5] cycles or -180 to 180
                            if phase_error > 0.5 {
                                phase_error -= 1.0;
                            }
                            if phase_error < -0.5 {
                                phase_error += 1.0;
                            }

                            // PI: integrate (Ki) then compute proportional action (Kp).
                            *c.local.frequency_corr += ki * phase_error;

                            // Clamp integrator to prevent windup
                            let MAX_FREQ_CORR = 1.0;
                            if *c.local.frequency_corr > MAX_FREQ_CORR {
                                *c.local.frequency_corr = MAX_FREQ_CORR;
                            }
                            if *c.local.frequency_corr < -MAX_FREQ_CORR {
                                *c.local.frequency_corr = -MAX_FREQ_CORR;
                            }
                            
                            // New frequency = nominal mains + integrator + proportional correction.
                            let freq = MAINS_FREQUENCY + *c.local.frequency_corr + kp * phase_error;
                            // Store applied frequency for diagnostics and update generator.
                            *c.local.dds_frequency = freq;
                            gens[ch].set_base_frequency(freq, SAMPLE_PERIOD);
                            // Apply a fixed global phase offset (0.5 cycles => 180°).
                            // This is intentional phase alignment — adjust if your hardware expects a different phase.
                            gens[ch].set_global_phase_offset(0.5);

                        }
                    });  
                    // Record last valid timestamp.   
                    *c.local.last_ts = Some(ts);
                }
                // No timestamp available — exit loop and return from interrupt.
                Ok(None) => break,
                // Overcapture condition: we received overflowed timestamps; log and record.
                Err(Some(ts)) => {
                    log::warn!("Overcapture detected, ts={}", ts);
                    *c.local.last_ts = Some(ts);
                }
                // No timestamp and no error — exit.
                Err(None) => break,
            
            }

        }
     
    }

    // Apply configuration received from the network (Miniconf).
    //
    // Responsibilities:
    // - Pull the latest settings from the network-miniconf
    // - Update local copy of settings
    // - Apply hardware changes (AFE gains, Current Sense DAC)
    // - Reconfigure harmonic DDS generators (phases are made relative to fundamental)
    // - Update livestream target
    #[task(priority = 1, local=[afes, current_sense_dac], shared=[network, settings, harmonic_generators])]
    fn settings_update(mut c: settings_update::Context) {
        // Read configuration from network miniconf and store in shared settings.
        let settings = c.shared.network.lock(|net| *net.miniconf.settings());
        c.shared.settings.lock(|current| *current = settings);
        
        // Apply AFE gain settings.
        c.local.afes.0.set_gain(settings.afe[0]);
        c.local.afes.1.set_gain(settings.afe[1]);

        // Apply DC offset to Current Sense DAC if present.
        if let Some(dac) = c.local.current_sense_dac.as_mut() {
            dac.write_voltage(settings.v_offset);
        }
        
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
