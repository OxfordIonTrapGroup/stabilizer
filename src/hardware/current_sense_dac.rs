use stm32h7xx_hal as hal;
use hal::{
    prelude::*,
};
use super::dac::DacCode;

pub struct CurrentSenseDac{
    spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
    cs: hal::gpio::Pin<'G', 10, hal::gpio::Output<hal::gpio::PushPull>>,
}


impl CurrentSenseDac{
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
        mut cs: hal::gpio::Pin<'G', 10, hal::gpio::Output<hal::gpio::PushPull>>,
    ) -> Self {
        cs.set_high();
        Self {spi, cs}
    }
    pub fn write_raw(&mut self, value: u16){
        let bytes = value.to_be_bytes();
        self.cs.set_low();
        self.spi.write(&bytes).unwrap();
        self.cs.set_high();
    }
    pub fn write_voltage(&mut self, v:f32){
        let code = DacCode::try_from(v).unwrap().0;
        self.write_raw(code);
    }
}