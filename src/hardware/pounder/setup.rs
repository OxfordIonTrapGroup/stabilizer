use super::{dds_output, hrtimer, timestamp, PounderDevices, QspiInterface};
use crate::hardware::{
    delay, design_parameters, mezzanine::Resources as MezzanineResources,
    timers,
};
use stm32h7xx_hal::{self as hal, gpio::Speed, prelude::*};

pub struct Devices {
    pub pounder: PounderDevices,
    pub dds_output: dds_output::DdsOutput,
    #[cfg(not(feature = "pounder_v1_0"))]
    pub timestamper: timestamp::Timestamper,
}

/// Configure the Pounder hardware for operation.
///
/// # Returns
/// `Some(devices)` if Pounder is detected, where `devices` contains all of the hardware
/// interfaces in a disabled state.
pub fn setup(
    resources: MezzanineResources,
    adc_dac_timer: &mut timers::SamplingTimer,
    batch_size: usize,
    sample_ticks: u32,
) -> Option<Devices> {
    let mut delay = delay::AsmDelay::new(resources.core_clocks.c_ck().to_Hz());

    // Measure the Pounder PGOOD output to detect if pounder is present on Stabilizer.
    let pounder_pgood = resources.gpio_header_pins.pb13.into_pull_down_input();
    delay.delay_ms(2u8);
    if !pounder_pgood.is_high() {
        return None;
    }
    log::info!("Found Pounder");

    let i2c1 = {
        let sda = resources
            .gpio_header_pins
            .pb7
            .into_alternate()
            .set_open_drain();
        let scl = resources
            .gpio_header_pins
            .pb8
            .into_alternate()
            .set_open_drain();
        let i2c1 = resources.devices.i2c1.i2c(
            (scl, sda),
            400.kHz(),
            resources.recs.i2c1,
            &resources.core_clocks,
        );

        shared_bus::new_atomic_check!(hal::i2c::I2c<hal::stm32::I2C1> = i2c1)
            .unwrap()
    };

    let spi = {
        let mosi = resources.gpio_header_pins.pd7.into_alternate();
        let miso = resources.gpio_header_pins.pa6.into_alternate();
        let sck = resources.gpio_header_pins.pg11.into_alternate();

        let config = hal::spi::Config::new(hal::spi::Mode {
            polarity: hal::spi::Polarity::IdleHigh,
            phase: hal::spi::Phase::CaptureOnSecondTransition,
        });

        // The maximum frequency of this SPI must be limited due to capacitance on the MISO
        // line causing a long RC decay.
        resources.devices.spi1.spi(
            (sck, miso, mosi),
            config,
            5.MHz(),
            resources.recs.spi1,
            &resources.core_clocks,
        )
    };

    let pwr0 = resources
        .shared_adcs
        .adc1
        .create_channel(resources.cpu_adc_dac_pins.pf11.into_analog());
    let pwr1 = resources
        .shared_adcs
        .adc2
        .create_channel(resources.cpu_adc_dac_pins.pf14.into_analog());
    let aux_adc0 = resources
        .shared_adcs
        .adc3
        .create_channel(resources.cpu_adc_dac_pins.pf3.into_analog());
    let aux_adc1 = resources
        .shared_adcs
        .adc3
        .create_channel(resources.cpu_adc_dac_pins.pf4.into_analog());

    let (ad9959, pg7) = {
        let qspi_interface = {
            // Instantiate the QUADSPI pins and peripheral interface.
            let qspi_pins = {
                let _ncs = resources
                    .gpio_header_pins
                    .pc11
                    .into_alternate::<9>()
                    .speed(Speed::VeryHigh);

                let clk = resources
                    .gpio_header_pins
                    .pb2
                    .into_alternate()
                    .speed(Speed::VeryHigh);
                let io0 = resources
                    .gpio_header_pins
                    .pe7
                    .into_alternate()
                    .speed(Speed::VeryHigh);
                let io1 = resources
                    .gpio_header_pins
                    .pe8
                    .into_alternate()
                    .speed(Speed::VeryHigh);
                let io2 = resources
                    .gpio_header_pins
                    .pe9
                    .into_alternate()
                    .speed(Speed::VeryHigh);
                let io3 = resources
                    .gpio_header_pins
                    .pe10
                    .into_alternate()
                    .speed(Speed::VeryHigh);

                (clk, io0, io1, io2, io3)
            };

            let qspi = resources.devices.quadspi.bank2(
                qspi_pins,
                design_parameters::POUNDER_QSPI_FREQUENCY.convert(),
                &resources.core_clocks,
                resources.recs.qspi,
            );

            QspiInterface::new(qspi).unwrap()
        };

        #[cfg(not(feature = "pounder_v1_0"))]
        let reset_pin = resources.gpio_header_pins.pg6.into_push_pull_output();
        #[cfg(feature = "pounder_v1_0")]
        let reset_pin = resources.gpio_header_pins.pa0.into_push_pull_output();

        let mut io_update =
            resources.gpio_header_pins.pg7.into_push_pull_output();

        // Delay to allow the pounder DDS reference clock to fully start up. The exact startup
        // time is not specified, but bench testing indicates it usually comes up within
        // 200-300uS. We do a larger delay to ensure that it comes up and is stable before
        // using it.
        delay.delay_ms(10u32);

        let mut ad9959 = ad9959::Ad9959::new(
            qspi_interface,
            reset_pin,
            &mut io_update,
            &mut delay,
            ad9959::Mode::FourBitSerial,
            design_parameters::DDS_REF_CLK.to_Hz() as f32,
            design_parameters::DDS_MULTIPLIER,
        )
        .unwrap();

        ad9959.self_test().unwrap();

        // Return IO_Update
        (ad9959, io_update.into_analog())
    };

    let dds_output = {
        let io_update_trigger = {
            let _io_update = pg7.into_alternate::<2>().speed(Speed::VeryHigh);

            // Configure the IO_Update signal for the DDS.
            let mut hrtimer = hrtimer::HighResTimerE::new(
                resources.devices.hrtim_time,
                resources.devices.hrtim_master,
                resources.devices.hrtim_common,
                resources.core_clocks,
                resources.recs.hrtim,
            );

            // IO_Update occurs after a fixed delay from the QSPI write. Note that the timer
            // is triggered after the QSPI write, which can take approximately 120nS, so
            // there is additional margin.
            hrtimer.configure_single_shot(
                hrtimer::Channel::Two,
                design_parameters::POUNDER_IO_UPDATE_DELAY,
                design_parameters::POUNDER_IO_UPDATE_DURATION,
            );

            // Ensure that we have enough time for an IO-update every batch.
            let sample_frequency = {
                design_parameters::TIMER_FREQUENCY.to_Hz() as f32
                    / sample_ticks as f32
            };

            let sample_period = 1.0 / sample_frequency;
            assert!(
                sample_period * batch_size as f32
                    > design_parameters::POUNDER_IO_UPDATE_DELAY
            );

            hrtimer
        };

        let (qspi, config) = ad9959.freeze();
        dds_output::DdsOutput::new(qspi, io_update_trigger, config)
    };

    #[cfg(not(feature = "pounder_v1_0"))]
    let timestamper = {
        log::info!("Assuming Pounder v1.1 or later");
        let etr_pin = resources.gpio_header_pins.pa0.into_alternate();

        // The frequency in the constructor is dont-care, as we will modify the period + clock
        // source manually below.
        let tim8 = resources.devices.tim8.timer(1.kHz(), resources.recs.tim8, &resources.core_clocks);
        let mut timestamp_timer = timers::PounderTimestampTimer::new(tim8);

        // Pounder is configured to generate a 500MHz reference clock, so a 125MHz sync-clock is
        // output. As a result, dividing the 125MHz sync-clk provides a 31.25MHz tick rate for
        // the timestamp timer. 31.25MHz corresponds with a 32ns tick rate.
        // This is less than fCK_INT/3 of the timer as required for oversampling the trigger.
        timestamp_timer.set_external_clock(timers::Prescaler::Div4);
        timestamp_timer.start();

        // Set the timer to wrap at the u16 boundary to meet the PLL periodicity.
        // Scale and wrap before or after the PLL.
        timestamp_timer.set_period_ticks(u16::MAX);
        let tim8_channels = timestamp_timer.channels();

        timestamp::Timestamper::new(
            timestamp_timer,
            tim8_channels.ch1,
            adc_dac_timer,
            etr_pin,
            batch_size,
        )
    };

    Some(Devices {
        pounder: PounderDevices::new(
            i2c1.acquire_i2c(),
            spi,
            (pwr0, pwr1),
            (aux_adc0, aux_adc1),
        )
        .unwrap(),
        dds_output,
        timestamper,
    })
}
