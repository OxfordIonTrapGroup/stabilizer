use stm32h7xx_hal as hal;
use hal::{
    prelude::*,
};

/// SPI driver for current sense board DAC
/// 
/// - Communicates over SPI1
/// - Uses dedicated chip-select (cs) GPIO
/// - Accepts 0-2.5V input range
/// - Outputs 16 bit DAC codes
pub struct CurrentSenseDac{
    spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
    cs: hal::gpio::Pin<'G', 10, hal::gpio::Output<hal::gpio::PushPull>>,
}


impl CurrentSenseDac{
    /// Create a new DAC driver instance
    /// 
    /// The chip select pin is driven high (inactive) on init
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
        mut cs: hal::gpio::Pin<'G', 10, hal::gpio::Output<hal::gpio::PushPull>>,
    ) -> Self {
        cs.set_high(); // Ensure DAC is deselected
        Self {spi, cs}
    }

    /// Write a raw 16 bit DAC code over SPI
    /// 
    /// The value is transmitted MSB first
    pub fn write_raw(&mut self, value: u16){
        let bytes = value.to_be_bytes();
        self.cs.set_low(); // Select DAC
        match self.spi.write(&bytes) {
            Ok(_) => {}
            Err(e) => {
                log::error!("SPI write error: {:?}", e);
            }
        }
        self.cs.set_high(); // Deselect DAC
    }

    /// Write a voltage to DAC
    /// 
    /// - Input ragne 0.0 V to 2.5 V
    /// - Values outside range are clamped
    /// - NaN inputs logged as errors
    pub fn write_voltage(&mut self, v:f32){

        // 16-bit full scale
        const DAC_MAX: f32 = u16::MAX as f32; 
        // DAC reference voltage
        const VREF: f32 = 2.5;
        
        
        if v.is_nan(){
            log::error!("NaN voltage requested");
        }

        // Clamp voltage to valid DAC input range
        let v_clamped = v.clamp(0.0, VREF);

        // Scale voltage proportionally to 16-bit DAC range
        let scaled = (v_clamped / VREF) * DAC_MAX;

        // Round to nearest integer code
        let code = (scaled + 0.5) as u16;
        
        self.write_raw(code);
        
    }
}