use stm32h7xx_hal as hal;
use hal::{
    prelude::*,
};

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
        log::info!("Writing value {}", value);
        //log::info!("Writing bytes {}", &bytes);
        match self.spi.write(&bytes) {
            Ok(_) => {}
            Err(e) => {
                log::error!("SPI write error: {:?}", e);
            }
        }
        self.cs.set_high();
    }
    pub fn write_voltage(&mut self, v:f32){

        const DAC_MAX: f32 = u16::MAX as f32;
        const VREF: f32 = 2.5;
        if v.is_nan(){
            log::error!("NaN voltage requested");
        }

        // Next need to clamp the code to a safe range
        let v_clamped = v.clamp(0.0, VREF);
        let scaled = (v_clamped / VREF) * DAC_MAX;
        let code = (scaled + 0.5) as u16;
        self.write_raw(code);
        
        
        // let code = DacCode::try_from(v).unwrap().0;
        // self.write_raw(code);s
    }
}