// Cascaded IIR filter setup, where the output of channel 0 is used as the input
// to channel 1 (e.g. to drive both the slow and fast PZTs of a M-Squared SolsTiS
// laser).
//
// ADC1 can be summed to the first ADC0 IIR filter input or output for testing.

// #![deny(warnings)]
#![no_std]
#![no_main]

use core::sync::atomic::{fence, Ordering};

use fugit::ExtU64;
use mutex_trait::prelude::*;
use minimq::{QoS, Retain, Property};

// Replace `hal` by `stabilizer::hardware::hal`
// `CycleCounterClock` -> `Systick`
use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        hal,
        timers::SamplingTimer,
        SystemTimer, Systick, AFE0, AFE1,
        EemDigitalOutput1,
    },
    net::{
        miniconf::Miniconf,
        serde::{Serialize, Deserialize},
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

use core::fmt::Write;
use heapless::String;
use log::{info, warn};

use idsp::{iir, Lowpass};

// The logarithm of the number of samples in each batch process. This corresponds with 2^3 samples
// per batch = 8 samples
const SAMPLE_TICKS_LOG2: u8 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

// The logarithm of the number of 100MHz timer ticks between each sample. This corresponds with a
// sampling period of 2^7 = 128 ticks. At 100MHz, 10ns per tick, this corresponds to a sampling
// period of 1.28 uS or 781.25 KHz.
const BATCH_SIZE_LOG2: u8 = 3;
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

const SCALE: f32 = i16::MAX as _;

const IIR_CASCADE_LENGTH: usize = 2;

/// Amount of bits to shift ADC1 samples before feeding into averaging low-pass filter.
/// 15 might be possible also (would need to look more closely at saturation behaviour).
const ADC1_LOWPASS_SHIFT: u8 = 14;

/// log2 of time constant of ADC1 lowpass filter in sample units, about 10 ms.
const ADC1_LOWPASS_LOG2_TC: u32 = 13;

const ADC1_FILTERED_TOPIC: &str = "read_adc1_filtered";

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Miniconf)]
pub enum ADC1Routing {
    Ignore,
    SumWithADC0,
    SumWithIIR0Output,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Miniconf, PartialEq)]
pub enum LockMode {
    Disabled,
    RampPassThrough,
    Enabled,
}

#[derive(Debug, Copy, Clone, Default, Deserialize, Miniconf)]
pub struct Gains {
    proportional: f32,
    integral: f32,
}

#[derive(Debug, Copy, Clone, Default, Deserialize, Miniconf)]
pub struct Notch {
    frequency: f32,
    quality_factor: f32,
}

#[derive(Debug, Copy, Clone, Default, Deserialize, Miniconf)]
pub struct LockDetectConfig {
    adc1_threshold: f32,
    reset_time: f32,
}

#[derive(Debug, Copy, Clone, Deserialize, Miniconf)]
pub struct Settings {
    lock_mode: LockMode,
    gain_ramp_time: f32,

    fast_gains: Gains,

    fast_notch: Notch,
    fast_notch_enable: bool,

    slow_gains: Gains,
    slow_enable: bool,

    adc1_routing: ADC1Routing,
    lock_detect: LockDetectConfig,

    aux_ttl_out: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            lock_mode: LockMode::Disabled,
            gain_ramp_time: 0.0,
            fast_gains: Default::default(),
            fast_notch: Default::default(),
            fast_notch_enable: false,
            slow_gains: Default::default(),
            slow_enable: false,
            adc1_routing: ADC1Routing::Ignore,
            lock_detect: Default::default(),
            aux_ttl_out: false,
        }
    }
}

pub struct GainRampState {
    current: f32,
    increment: f32,
}

pub struct LockDetectState {
    decrement: u32,
    counter: u32,
    threshold: i16,
    ///< in units of ADC1 codes
    pin: hal::gpio::gpiod::PD3<hal::gpio::Output<hal::gpio::PushPull>>,
}


#[rtic::app(device=stabilizer::hardware::hal::stm32, peripherals=true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
        telemetry: TelemetryBuffer,

        iir_ch: [[iir::IIR<f32>; IIR_CASCADE_LENGTH]; 2],
        adc1_routing: ADC1Routing,
        gain_ramp: GainRampState,
        adc1_filtered: i32,
        lock_detect: LockDetectState,
    }

    #[local]
    struct Local {
        sampling_timer: SamplingTimer,
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),

        // Format: iir_state[ch][cascade-no][coeff]
        iir_state: [[iir::Vec5<f32>; IIR_CASCADE_LENGTH]; 2],
        current_mode: LockMode,
        adc1_filter: Lowpass<4>,
        aux_ttl_out: EemDigitalOutput1,

        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        // Configure the microcontroller
        let (stabilizer, _pounder) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("10.255.6.4")
                .parse()
                .unwrap(),
        );

        // Set up lock detect GPIO output pins, which are specific to this
        // configuration.
        let (mut lock_detect_output, aux_ttl_out) = match stabilizer.eem_gpio {
            Some(eem_gpio) => (eem_gpio.lvds6, eem_gpio.lvds7),
            None => panic!("Pounder detected; GPIO pins not usable."),
        };
        lock_detect_output.set_low();
        let lock_detect = LockDetectState {
            decrement: 1,
            counter: 0,
            threshold: i16::MAX,
            pin: lock_detect_output,
        };

        let shared = Shared {
            network,
            telemetry: TelemetryBuffer::default(),
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],
            adc1_routing: ADC1Routing::Ignore,
            gain_ramp: GainRampState { current: 1.0, increment: 0.0 },
            adc1_filtered: 0,
            lock_detect,
        };

        let mut local = Local {
            sampling_timer: stabilizer.adc_dac_timer,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            iir_state: [[[0.; 5]; IIR_CASCADE_LENGTH]; 2],
            current_mode: LockMode::Disabled,
            adc1_filter: Lowpass::default(),
            aux_ttl_out,
            cpu_temp_sensor: stabilizer.temperature_sensor,
        };

        // We hard-code gains here to save some complexity w.r.t. converting the
        // lock detect threshold from volts to codes.
        local.afes.0.set_gain(Gain::G1);
        local.afes.1.set_gain(Gain::G10);

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // Spawn a settings and telemetry update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        start::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(stabilizer.systick))
    }

    #[task(priority = 1, local=[sampling_timer])]
    fn start(c: start::Context) {
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
    }

    /// Main DSP processing routine for Stabilizer.
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
    #[task(binds=DMA1_STR4, shared=[iir_ch, adc1_routing, adc1_filtered, gain_ramp, lock_detect, telemetry],
           local=[adcs, dacs, iir_state, adc1_filter], priority=2)]
    fn process(c: process::Context) {
        let process::SharedResources {
            iir_ch,
            adc1_routing,
            adc1_filtered,
            gain_ramp,
            lock_detect,
            telemetry,
        } = c.shared;

        let process::LocalResources {
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            adc1_filter,
        } = c.local;

        fn to_dac(val: f32) -> u16 {
            // Note(unsafe): The filter limits ensure that the value is in range.
            // The truncation introduces 1/2 LSB distortion.
            let y: i16 = unsafe { val.to_int_unchecked() };
            // Convert to DAC code
            y as u16 ^ 0x8000
        }

        (iir_ch, adc1_routing, adc1_filtered, gain_ramp, lock_detect, telemetry).lock(
            |iir_ch, adc1_routing, adc1_filtered, gain_ramp, lock_detect, telemetry| {
                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    let adc_samples = [adc0, adc1];
                    let dac_samples = [dac0, dac1];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                    for sample_idx in 0..adc_samples[0].len() {
                        let adc1_int = adc_samples[1][sample_idx] as i16;
                        *adc1_filtered = adc1_filter.update(
                            (adc1_int as i32) << ADC1_LOWPASS_SHIFT,
                            ADC1_LOWPASS_LOG2_TC,
                        );

                        {
                            // Lock detect.
                            let ld = &mut *lock_detect;
                            if adc1_int < ld.threshold {
                                ld.pin.set_low();
                                ld.counter = u32::MAX;
                            } else if ld.counter > 0 {
                                ld.counter = ld.counter.saturating_sub(ld.decrement);
                                if ld.counter == 0 {
                                    ld.pin.set_high();
                                }
                            }
                        }

                        // Cascaded PID controllers.
                        let adc1_float = f32::from(adc1_int);
                        let mut x = f32::from(adc_samples[0][sample_idx] as i16)
                            * gain_ramp.current;
                        if let ADC1Routing::SumWithADC0 = adc1_routing {
                            x += adc1_float;
                        }

                        let y0 = {
                            let mut y = iir_ch[0][0]
                                .update(&mut iir_state[0][0], x, false);
                            if let ADC1Routing::SumWithIIR0Output = adc1_routing
                            {
                                y += adc1_float;
                            }
                            iir_ch[0][1]
                                .update(&mut iir_state[0][1], y, false)
                        };
                        dac_samples[0][sample_idx] = to_dac(y0);

                        let y1 = {
                            let y = iir_ch[1][0]
                                .update(&mut iir_state[1][0], y0, false);
                            iir_ch[1][1]
                                .update(&mut iir_state[1][1], y, false)
                        };
                        dac_samples[1][sample_idx] = to_dac(y1);

                        if gain_ramp.current < 1.0 {
                            gain_ramp.current += gain_ramp.increment;
                            if gain_ramp.current > 1.0 {
                                gain_ramp.current = 1.0;
                            }
                        }
                    }

                    // Update telemetry measurements.
                    telemetry.adcs = [
                        AdcCode(adc_samples[0][0]),
                        AdcCode(adc_samples[1][0]),
                    ];

                    telemetry.dacs = [
                        DacCode(dac_samples[0][0]),
                        DacCode(dac_samples[1][0]),
                    ];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    #[idle(shared=[network, adc1_filtered])]
    fn idle(mut c: idle::Context) -> ! {
        info!("Starting idle task...");

        let mut subscribed = false;
        let mut adc1_filtered = c.shared.adc1_filtered;
        let adc1_filtered_topic = c.shared.network.lock(|net| {
            net.telemetry.as_ref().unwrap().topic(ADC1_FILTERED_TOPIC)
        });

        loop {
            match c.shared.network.lock(|net| {
                // Take ownership of telemetry.
                let mut telemetry = net.telemetry.take().unwrap();

                // Skips the telemetry update because we took ownership.
                let settings_result = net.update();
                let mqtt_result = {
                    let mqtt = telemetry.mqtt();

                    if !subscribed {
                        match mqtt.client.subscribe(&adc1_filtered_topic, &[]) {
                            Ok(_) => {
                                subscribed = true;
                                info!("Subscribed to ADC1_FILTERED_TOPIC");
                            },
                            Err(_) => {}
                        }
                    }

                    mqtt.poll(|client, topic, _message, properties| {
                        let mut payload_buf: String<64> = String::new();

                        let payload = if topic == adc1_filtered_topic {
                            let filtered_int: i32 = adc1_filtered.lock(|a| *a);
                            // 16 signed ADC bits (set to 1V full-range).
                            const FULL_RANGE: i32 = 1 << (15 + ADC1_LOWPASS_SHIFT);
                            write!(
                                &mut payload_buf,
                                "{}",
                                (filtered_int as f32) / (FULL_RANGE as f32)
                            ).unwrap();
                            info!("Received ADC1_FILTERED_TOPIC");
                            payload_buf.as_bytes()
                        } else {
                            warn!("Unexpected topic");
                            "Unexpected topic".as_bytes()
                        };

                        let topic_property = properties
                            .iter()
                            .find(|&prop| matches!(*prop, Property::ResponseTopic(_)));
                        // Make a best-effort attempt to send the response. If we get a failure,
                        // we may have disconnected or the peer provided an invalid topic to
                        // respond to. Ignore the failure in these cases.
                        if let Some(Property::ResponseTopic(response_topic)) = topic_property {
                            // Send back any correlation data with the response.
                            let response_properties = properties
                                .iter()
                                .find(|&prop| matches!(*prop, Property::CorrelationData(_)))
                                .map(core::slice::from_ref)
                                .unwrap_or(&[]);

                            client.publish(
                                response_topic,
                                payload,
                                QoS::AtMostOnce,
                                Retain::NotRetained,
                                response_properties)
                            .ok();
                        } else {
                            warn!("No response topic");
                        }
                    })
                };
                // Move telemetry back into NetworkUsers.
                net.telemetry.replace(telemetry);

                if let Err(_) = mqtt_result {
                    if subscribed {
                        warn!("MQTT broker connection lost.");
                    }
                    subscribed = false;
                }
                settings_result
            }) {
                NetworkState::SettingsChanged(_path) => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, shared=[iir_ch, gain_ramp, lock_detect, adc1_routing, network], local=[current_mode, aux_ttl_out])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c.shared.network.lock(|net| *net.miniconf.settings());

        let clk: hal::time::MegaHertz =
            hardware::design_parameters::TIMER_FREQUENCY;
        let sample_freq = (clk.to_Hz() as f32) / (SAMPLE_TICKS as f32);
        let freq_factor = 2.0 / sample_freq;

        c.shared.iir_ch.lock(|iir| {
            match settings.lock_mode {
                LockMode::Disabled => {
                    iir[0][0].set_pi(0.0, 0.0, 0.0).unwrap();
                    iir[1][0].set_pi(0.0, 0.0, 0.0).unwrap();
                }
                LockMode::RampPassThrough => {
                    // Gain 5 gives approximately ±10 V when driven using the
                    // Vescent servo box ramp.
                    iir[0][0].set_pi(5.0, 0.0, 0.0).unwrap();
                    iir[1][0].set_pi(0.0, 0.0, 0.0).unwrap();
                }
                LockMode::Enabled => {
                    // Negative sign in fast branch to match AOM lock; both PZTs
                    // have same sign.
                    let fast_p = {
                        // KLUDGE: For whatever reason, copysign()-ing the I gain
                        // doesn't work for signed zero, so lower-bound P gain to
                        // just above zero.
                        let mut p = settings.fast_gains.proportional;
                        if p == 0.0 {
                            p = f32::MIN_POSITIVE;
                        }
                        p
                    };
                    iir[0][0]
                        .set_pi(
                            -fast_p,
                            freq_factor * settings.fast_gains.integral,
                            0.0,
                        )
                        .unwrap();
                    iir[1][0]
                        .set_pi(
                            settings.slow_gains.proportional,
                            freq_factor * settings.slow_gains.integral,
                            0.0,
                        )
                        .unwrap();
                }
            }
        });
        if settings.lock_mode != *c.local.current_mode {
            c.shared.gain_ramp.lock(|gr| {
                if settings.lock_mode == LockMode::Enabled
                    && settings.gain_ramp_time > 0.0
                {
                    gr.current = 0.0;
                    gr.increment =
                        1.0 / (sample_freq * settings.gain_ramp_time);
                } else {
                    gr.current = 1.0;
                    gr.increment = 0.0;
                }
            });
        }
        *c.local.current_mode = settings.lock_mode;

        {
            let threshold = settings.lock_detect.adc1_threshold * 32768.0;
            if threshold < -32768.0 || threshold > 32767.0 {
                warn!("Ignoring invalid lock detect threshold: {}", threshold);
            } else {
                c.shared.lock_detect.lock(|state| {
                    state.threshold = threshold as i16;
                });
            }

            let mut reset_samples =
                settings.lock_detect.reset_time * sample_freq;
            if reset_samples < 1.0 {
                warn!(
                    "Lock detect reset time too small, clamping: {}",
                    settings.lock_detect.reset_time
                );
                reset_samples = 1.0;
            }
            let mut decrement = ((u32::MAX as f32) / reset_samples) as u32;
            if decrement == 0 {
                warn!(
                    "Lock detect reset time too large, clamping: {}",
                    settings.lock_detect.reset_time
                );
                decrement = 1;
            }
            c.shared.lock_detect.lock(|state| {
                state.decrement = decrement;
            });
        }

        c.shared.iir_ch.lock(|iir| {
            if settings.fast_notch_enable {
                iir[0][1]
                    .set_notch(
                        freq_factor * settings.fast_notch.frequency,
                        settings.fast_notch.quality_factor,
                    )
                    .unwrap();
            } else {
                iir[0][1].set_scale(1.0).unwrap();
            }
        });

        c.shared.iir_ch.lock(|iir| {
            // Second IIR on slow PZT unused - optional low-pass filter?
            iir[1][1].set_scale(1.0).unwrap();
        });

        let iir = c.shared.iir_ch.lock(|iir| *iir);
        {
            info!("IIR settings:");
            info!(" [0][0]: {:?}", iir[0][0]);
            info!(" [0][1]: {:?}", iir[0][1]);
            info!(" [1][0]: {:?}", iir[1][0]);
            info!(" [1][1]: {:?}", iir[1][1]);
        }

        c.shared
            .adc1_routing
            .lock(|r| *r = settings.adc1_routing);

        if settings.aux_ttl_out {
            c.local.aux_ttl_out.set_high();
        } else {
            c.local.aux_ttl_out.set_low();
        }
    }

    #[task(priority = 1, shared=[network, telemetry], local=[cpu_temp_sensor, afes])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        c.shared.network.lock(|net| {
            net.telemetry.as_mut().unwrap().publish(&telemetry.finalize(
                c.local.afes.0.get_gain(),
                c.local.afes.1.get_gain(),
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            ))
        });

        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after(10.secs())
            .unwrap();
    }
}
