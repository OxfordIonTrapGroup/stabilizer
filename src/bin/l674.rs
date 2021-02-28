// Cascaded IIR filter setup, where the output of channel 0 is used as the input
// to channel 1 (e.g. to drive both the slow and fast PZTs of a M-Squared SolsTiS
// laser).
//
// ADC1 can be summed to the first ADC0 IIR filter input or output for testing.

#![deny(warnings)]
#![no_std]
#![no_main]

use stm32h7xx_hal as hal;

use stabilizer::hardware;

use miniconf::{
    embedded_nal::{IpAddr, Ipv4Addr},
    minimq, MqttInterface, StringSet,
};
use serde::Deserialize;

use dsp::iir;
use hardware::{
    Adc0Input, Adc1Input, AfeGain, CycleCounter, Dac0Output, Dac1Output,
    NetworkStack, AFE0, AFE1,
};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 2;

#[derive(Clone, Copy, Debug, Deserialize, StringSet)]
pub enum ADC1Routing {
    Ignore,
    SumWithADC0,
    SumWithIIR0Output
}

#[derive(Debug, Deserialize, StringSet)]
pub struct Settings {
    afe: [AfeGain; 2],
    iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
    adc1_routing: ADC1Routing,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [AfeGain::G1, AfeGain::G1],
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],
            adc1_routing: ADC1Routing::Ignore,
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        mqtt_interface:
            MqttInterface<Settings, NetworkStack, minimq::consts::U256>,
        clock: CycleCounter,

        // Format: iir_state[ch][cascade-no][coeff]
        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
        #[init([[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2])]
        iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],

        #[init(ADC1Routing::Ignore)]
        adc1_routing: ADC1Routing,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let mqtt_interface = {
            let mqtt_client = {
                let broker = IpAddr::V4(Ipv4Addr::new(10, 34, 16, 1));
                minimq::MqttClient::new(
                    broker,
                    "stabilizer",
                    stabilizer.net.stack,
                )
                .unwrap()
            };

            MqttInterface::new(mqtt_client, "stabilizer", Settings::default())
                .unwrap()
        };

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            mqtt_interface,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            clock: stabilizer.cycle_counter,
        }
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
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, iir_state, iir_ch, adc1_routing], priority=2)]
    fn process(c: process::Context) {
        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        fn to_dac(val: f32) -> u16 {
            // Note(unsafe): The filter limits ensure that the value is in range.
            // The truncation introduces 1/2 LSB distortion.
            let y = unsafe { val.to_int_unchecked::<i16>() };
            // Convert to DAC code
            y as u16 ^ 0x8000
        }

        for sample_idx in 0..adc_samples[0].len() {
            let adc1 = f32::from(adc_samples[1][sample_idx] as i16);
            let mut x = f32::from(adc_samples[0][sample_idx] as i16);
            if let ADC1Routing::SumWithADC0 = c.resources.adc1_routing {
                x += adc1;
               }

            let y0 = {
                let mut y = c.resources.iir_ch[0][0].update(&mut c.resources.iir_state[0][0], x);
                if let ADC1Routing::SumWithIIR0Output = c.resources.adc1_routing {
                    y += adc1;
                }
                c.resources.iir_ch[0][1].update(&mut c.resources.iir_state[0][1], y)
            };
            dac_samples[0][sample_idx] = to_dac(y0);

            let y1 = {
                let y = c.resources.iir_ch[1][0].update(&mut c.resources.iir_state[1][0], y0);
                c.resources.iir_ch[1][1].update(&mut c.resources.iir_state[1][1], y)
            };
            dac_samples[1][sample_idx] = to_dac(y1);
        }
    }

    #[idle(resources=[mqtt_interface, clock], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        let clock = c.resources.clock;

        loop {
            let sleep = c.resources.mqtt_interface.lock(|interface| {
                !interface.network_stack().poll(clock.current_ms())
            });

            match c
                .resources
                .mqtt_interface
                .lock(|interface| interface.update().unwrap())
            {
                miniconf::Action::Continue => {
                    if sleep {
                        cortex_m::asm::wfi();
                    }
                }
                miniconf::Action::CommitSettings => {
                    c.spawn.settings_update().unwrap()
                }
            }
        }
    }

    #[task(priority = 1, resources=[mqtt_interface, afes, iir_ch, adc1_routing])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = &c.resources.mqtt_interface.settings;
        c.resources.iir_ch.lock(|iir| *iir = settings.iir_ch);
        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);
        c.resources.adc1_routing.lock(|r| *r = settings.adc1_routing);
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 3)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI3, priority = 3)]
    fn spi3(_: spi3::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI4, priority = 3)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 output error");
    }

    #[task(binds = SPI5, priority = 3)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 output error");
    }

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};
