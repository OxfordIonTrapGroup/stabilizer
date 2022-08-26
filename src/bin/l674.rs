// Cascaded IIR filter setup, where the output of channel 0 is used as the input
// to channel 1 (e.g. to drive both the slow and fast PZTs of a M-Squared SolsTiS
// laser).
//
// ADC1 can be summed to the first ADC0 IIR filter input or output for testing.

#![deny(warnings)]
#![no_std]
#![no_main]

use core::sync::atomic::{fence, Ordering};

use fugit::ExtU64;
use mutex_trait::prelude::*;
use minimq::{QoS, Retain, Property};

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

use idsp::{iir, Lowpass, abs};

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;
const SAMPLE_PERIOD: f32 = SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

// The number of samples in each batch process
const BATCH_SIZE: usize = 8;

const SCALE: f32 = i16::MAX as _;

const IIR_CASCADE_LENGTH: usize = 2;

const ADC1_FILTERED_TOPIC: &str = "read_adc1_filtered";

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Miniconf)]
pub enum ADC1Routing {
    Ignore,
    SumWithADC0,
    SumWithIIR0Output,
}


#[derive(Debug, Copy, Clone, Deserialize, Miniconf)]
pub struct Settings {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    afe: [Gain; 2],

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    /// * <m> specifies which cascade to configure. <m> := [0, 1]
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    iir_ch: [[iir::IIR<f32>; IIR_CASCADE_LENGTH]; 2],

    /// Configure the gain ramp time.
    gain_ramp_time: f32,

    /// Configure routing of ADC1 signal.
    ///
    /// # Path
    /// `adc1_routing`
    ///
    /// # Value
    /// Any of the variants of [ADC1Routing] enclosed in double quotes.
    adc1_routing: ADC1Routing,

    /// Specifies the ADC1 input voltage threshold in volts.
    ///
    /// # Path
    /// `ld_threshold`
    ld_threshold: f32,
    ld_reset_time: f32,

    aux_ttl_out: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [Gain::G1, Gain::G10],
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],
            gain_ramp_time: 0.0,
            adc1_routing: ADC1Routing::Ignore,
            ld_threshold: 0.0,
            ld_reset_time: 0.0,
            aux_ttl_out: false,
        }
    }
}

pub struct GainRampState {
    current: f32,
    increment: f32,
}

impl GainRampState {
    pub fn update(&mut self) {
        if self.current < 1.0 {
            self.current += self.increment;
            if self.current > 1.0 {
                self.current = 1.0;
            }
        }
    }

    pub fn reset(&mut self, increment: f32) {
        self.increment = increment;
        self.current = if increment > 0.0 { 0.0 } else { 1.0 };
    }

    // Not a member function to be able to execute this before locking the object.
    pub fn prepare_reset(ramp_time: f32) -> f32 {
        if ramp_time > 0.0 { SAMPLE_PERIOD / ramp_time } else { 0.0 }
    }
}

pub struct LockDetectState {
    adc1_filtered: i32,
    decrement: u32,
    counter: u32,
    threshold: i16,
    ///< in units of ADC1 codes
    adc1_filter: Lowpass<4>,
    pin: hal::gpio::gpiod::PD3<hal::gpio::Output<hal::gpio::PushPull>>,
}

impl LockDetectState {
    /// Amount of bits to shift ADC1 samples before feeding into averaging low-pass filter.
    /// 15 might be possible also (would need to look more closely at saturation behaviour).
    const LOWPASS_SHIFT: u8 = 14;

    /// log2 of time constant of ADC1 lowpass filter in sample units, about 10 ms.
    const LOWPASS_LOG2_TC: u32 = 13;

    pub fn update(&mut self, sample: i16) {
        // Lock detect.
        self.adc1_filtered = self.adc1_filter.update(
            (sample as i32) << Self::LOWPASS_SHIFT,
            Self::LOWPASS_LOG2_TC,
        );
        if sample < self.threshold {
            self.pin.set_low();
            self.counter = u32::MAX;
        } else if self.counter > 0 {
            self.counter = self.counter.saturating_sub(self.decrement);
            if self.counter == 0 {
                self.pin.set_high();
            }
        }
    }

    pub fn reset(&mut self, threshold: Option<i16>, decrement: u32) {
        self.threshold = threshold.unwrap_or(self.threshold);
        self.decrement = decrement;
    }

    // Not a member function to be able to execute this before locking the object.
    pub fn prepare_reset(threshold: f32, reset_time: f32) -> (Option<i16>, u32) {
        let threshold_opt = if let Ok(thres) = AdcCode::try_from(threshold) {
            Some(i16::from(thres))
        } else {
            warn!("Ignoring invalid lock detect threshold: {}", threshold);
            None
        };

        let mut reset_samples = reset_time / SAMPLE_PERIOD;
        if reset_samples < 1.0 {
            warn!("Lock detect reset time too small, clamping: {}", reset_time);
            reset_samples = 1.0;
        }
        let mut decrement = ((u32::MAX as f32) / reset_samples) as u32;
        if decrement == 0 {
            warn!("Lock detect reset time too large, clamping: {}", reset_time);
            decrement = 1;
        }

        (threshold_opt, decrement)
    }

    pub fn get_filtered(&self) -> AdcCode {
        AdcCode::from((self.adc1_filtered >> Self::LOWPASS_SHIFT) as u16)
    }
}

#[rtic::app(device=stabilizer::hardware::hal::stm32, peripherals=true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,

        settings: Settings,
        telemetry: TelemetryBuffer,

        gain_ramp: GainRampState,
        lock_detect: LockDetectState,
    }

    #[local]
    struct Local {
        sampling_timer: SamplingTimer,
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),

        iir_state: [[iir::Vec5<f32>; IIR_CASCADE_LENGTH]; 2],
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
            adc1_filtered: 0,
            decrement: 1,
            counter: 0,
            threshold: i16::MAX,
            adc1_filter: Lowpass::default(),
            pin: lock_detect_output,
        };

        let shared = Shared {
            network,
            settings: Settings::default(),
            telemetry: TelemetryBuffer::default(),
            gain_ramp: GainRampState { current: 1.0, increment: 0.0 },
            lock_detect,
        };

        let mut local = Local {
            sampling_timer: stabilizer.adc_dac_timer,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            iir_state: [[[0.; 5]; IIR_CASCADE_LENGTH]; 2],
            aux_ttl_out,
            cpu_temp_sensor: stabilizer.temperature_sensor,
        };

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
    #[task(binds=DMA1_STR4, shared=[settings, telemetry, gain_ramp, lock_detect],
           local=[adcs, dacs, iir_state], priority=2)]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            gain_ramp,
            lock_detect,
        } = c.shared;

        let process::LocalResources {
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
        } = c.local;

        fn to_dac(val: f32) -> u16 {
            // Note(unsafe): The filter limits ensure that the value is in range.
            // The truncation introduces 1/2 LSB distortion.
            let y: i16 = unsafe { val.to_int_unchecked() };
            // Convert to DAC code
            DacCode::from(y).0
        }

        (settings, telemetry, gain_ramp, lock_detect).lock(
            |settings, telemetry, gain_ramp, lock_detect| {
                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                    adc0.iter()
                        .zip(adc1.iter())
                        .zip(dac0.iter_mut())
                        .zip(dac1.iter_mut())
                        .map(|(((a0, a1), d0), d1)| {
                            let adc1_int = *a1 as i16;

                            lock_detect.update(adc1_int);

                            // Cascaded PID controllers.
                            let adc1_float = f32::from(adc1_int);
                            let mut x = f32::from(*a0 as i16) * gain_ramp.current;
                            if let ADC1Routing::SumWithADC0 = settings.adc1_routing {
                                x += adc1_float;
                            }

                            let y0 = {
                                let mut y = settings.iir_ch[0][0]
                                    .update(&mut iir_state[0][0], x, false);
                                if let ADC1Routing::SumWithIIR0Output = settings.adc1_routing
                                {
                                    y += adc1_float;
                                }
                                settings.iir_ch[0][1].update(&mut iir_state[0][1], y, false)
                            };
                            *d0 = to_dac(y0);

                            let y1 = {
                                let y = settings.iir_ch[1][0]
                                    .update(&mut iir_state[1][0], y0, false);
                                settings.iir_ch[1][1].update(&mut iir_state[1][1], y, false)
                            };
                            *d1 = to_dac(y1);

                            gain_ramp.update();
                        })
                        .last();

                    // Update telemetry measurements.
                    telemetry.adcs = [AdcCode(adc0[0]), AdcCode(adc1[0])];
                    telemetry.dacs = [DacCode(dac0[0]), DacCode(dac1[0])];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    #[idle(shared=[network, lock_detect, settings])]
    fn idle(mut c: idle::Context) -> ! {
        info!("Starting idle task...");

        let mut subscribed = false;
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
                        if let Ok(_) = mqtt.client.subscribe(&adc1_filtered_topic, &[]) {
                            subscribed = true;
                            info!("Subscribed to ADC1_FILTERED_TOPIC");
                        }
                    }

                    mqtt.poll(|client, topic, _message, properties| {
                        let mut payload_buf: String<64> = String::new();

                        let payload = if topic == adc1_filtered_topic {
                            let afe1_gain = c.shared.settings.lock(|settings| settings.afe[1]);
                            let adc1_filtered = f32::from(
                                c.shared.lock_detect.lock(|ld| ld.get_filtered())
                            ) / afe1_gain.as_multiplier();
                            write!(&mut payload_buf, "{}", adc1_filtered).unwrap();
                            payload_buf.as_bytes()
                        } else {
                            warn!("Unexpected topic");
                            "Unexpected topic".as_bytes()
                        };

                        let property = properties
                            .iter()
                            .find(|&prop| {
                                matches!(*prop, Property::ResponseTopic(_))
                            });
                        // Make a best-effort attempt to send the response.
                        // If we get a failure, we may have disconnected or the
                        // peer provided an invalid topic to respond to.
                        // Ignore the failure in these cases.
                        if let Some(Property::ResponseTopic(topic)) = property {
                            // Send back any correlation data with the response.
                            let response_properties = properties
                                .iter()
                                .find(|&prop| {
                                    matches!(*prop, Property::CorrelationData(_))
                                })
                                .map(core::slice::from_ref)
                                .unwrap_or(&[]);

                            client.publish(
                                topic,
                                payload,
                                QoS::AtMostOnce,
                                Retain::NotRetained,
                                response_properties)
                            .ok();
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
                },
                NetworkState::Updated => {},
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, shared=[network, settings, gain_ramp, lock_detect],
           local=[aux_ttl_out, afes])]
    fn settings_update(mut c: settings_update::Context) {

        fn lock_enabled(iir: [[iir::IIR<f32>; IIR_CASCADE_LENGTH]; 2]) -> bool {
            const EPS: f32 = f32::EPSILON;
            abs(iir[0][0].get_k()) < EPS && abs(iir[1][0].get_k()) < EPS
        }

        let settings = c.shared.network.lock(|net| *net.miniconf.settings());

        let en_before = c.shared.settings.lock(|current| {
            let en_before = lock_enabled(current.iir_ch);
            *current = settings;
            en_before
        });
        let en_after = lock_enabled(settings.iir_ch);

        // AFE Gains
        c.local.afes.0.set_gain(settings.afe[0]);
        c.local.afes.1.set_gain(settings.afe[1]);

        // IIR gain ramp
        if en_before != en_after {
            let ramp_time = if en_after {
                info!("Activating gain ramp");
                settings.gain_ramp_time
            } else {
                0.0
            };
            let increment = GainRampState::prepare_reset(ramp_time);
            c.shared.gain_ramp.lock(|gr| gr.reset(increment));
        }

        // Lock detect
        let (threshold, decrement) = LockDetectState::prepare_reset(settings.ld_threshold,
                                                                    settings.ld_reset_time);
        c.shared.lock_detect.lock(|ld| ld.reset(threshold, decrement));

        // Print IIR settings
        let iir = c.shared.settings.lock(|settings| settings.iir_ch);
        {
            info!("IIR settings:");
            info!(" [0][0]: {:?}", iir[0][0]);
            info!(" [0][1]: {:?}", iir[0][1]);
            info!(" [1][0]: {:?}", iir[1][0]);
            info!(" [1][1]: {:?}", iir[1][1]);
        }

        // Set auxiliary output TTL
        if settings.aux_ttl_out {
            c.local.aux_ttl_out.set_high();
        } else {
            c.local.aux_ttl_out.set_low();
        }
    }

    #[task(priority = 1, shared=[settings, network, telemetry], local=[cpu_temp_sensor])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let gains = c.shared.settings.lock(|settings| settings.afe);

        c.shared.network.lock(|net| {
            net.telemetry.as_mut().unwrap().publish(&telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            ))
        });

        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after(10.secs()).unwrap();
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
}
