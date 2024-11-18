use stm32h7xx_hal as hal;

use super::shared_adc::SharedAdc;

pub struct GpioHeaderPins {
    pub pa0: hal::gpio::gpioa::PA0,
    pub pa6: hal::gpio::gpioa::PA6,
    pub pb2: hal::gpio::gpiob::PB2,
    pub pb7: hal::gpio::gpiob::PB7,
    pub pb8: hal::gpio::gpiob::PB8,
    pub pb13: hal::gpio::gpiob::PB13,
    pub pc11: hal::gpio::gpioc::PC11,
    pub pd7: hal::gpio::gpiod::PD7,
    pub pe7: hal::gpio::gpioe::PE7,
    pub pe8: hal::gpio::gpioe::PE8,
    pub pe9: hal::gpio::gpioe::PE9,
    pub pe10: hal::gpio::gpioe::PE10,
    pub pg6: hal::gpio::gpiog::PG6,
    pub pg7: hal::gpio::gpiog::PG7,
    pub pg11: hal::gpio::gpiog::PG11,
}

pub struct CpuAdcDacHeaderPins {
    pub pf3: hal::gpio::gpiof::PF3,
    pub pf4: hal::gpio::gpiof::PF4,
    pub pf11: hal::gpio::gpiof::PF11,
    pub pf14: hal::gpio::gpiof::PF14,
}

pub struct Devices {
    pub hrtim_common: hal::stm32::HRTIM_COMMON,
    pub hrtim_master: hal::stm32::HRTIM_MASTER,
    pub hrtim_time: hal::stm32::HRTIM_TIME,
    pub i2c1: hal::stm32::I2C1,
    pub quadspi: hal::stm32::QUADSPI,
    pub spi1: hal::stm32::SPI1,
    pub tim8: hal::stm32::TIM8,
}

pub struct Recs {
    pub i2c1: hal::rcc::rec::I2c1,
    pub hrtim: hal::rcc::rec::Hrtim,
    pub qspi: hal::rcc::rec::Qspi,
    pub spi1: hal::rcc::rec::Spi1,
    pub tim8: hal::rcc::rec::Tim8,
}

pub struct SharedAdcs {
    pub adc1: &'static SharedAdc<hal::stm32::ADC1>,
    pub adc2: &'static SharedAdc<hal::stm32::ADC2>,
    pub adc3: &'static SharedAdc<hal::stm32::ADC3>,
}

pub struct Resources {
    pub core_clocks: hal::rcc::CoreClocks,
    pub cpu_adc_dac_pins: CpuAdcDacHeaderPins,
    pub gpio_header_pins: GpioHeaderPins,
    pub devices: Devices,
    pub recs: Recs,
    pub shared_adcs: SharedAdcs,
}
