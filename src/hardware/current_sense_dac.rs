//! Current sense board offset DAC
//!
//! The current sense mezzanine carries a 16-bit unipolar DAC with a 2.5 V reference, which sets
//! the DC offset of the analog output. It is written to via SPI1 (shared with Pounder, which
//! cannot be present at the same time) with a dedicated GPIO as chip select.
use super::hal::{self, prelude::*};

/// SPI driver for the current sense board offset DAC.
pub struct CurrentSenseDac {
    spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
    cs: hal::gpio::gpiog::PG10<hal::gpio::Output>,
}

impl CurrentSenseDac {
    /// Output voltage corresponding to the full-scale code, given by the DAC reference.
    pub const FULL_SCALE_VOLTAGE: f32 = 2.5;

    /// Create a new DAC driver instance.
    ///
    /// The chip select pin is driven high (inactive).
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI1, hal::spi::Enabled, u8>,
        mut cs: hal::gpio::gpiog::PG10<hal::gpio::Output>,
    ) -> Self {
        cs.set_high();
        Self { spi, cs }
    }

    /// Write a raw 16-bit DAC code, MSB first.
    pub fn write_raw(&mut self, value: u16) {
        self.cs.set_low();
        if let Err(e) = self.spi.write(&value.to_be_bytes()) {
            log::error!("SPI write error: {:?}", e);
        }
        self.cs.set_high();
    }

    /// Set the output voltage.
    ///
    /// Values outside of `[0, FULL_SCALE_VOLTAGE]` are clamped; NaN is logged as an error and
    /// mapped to 0 V.
    pub fn write_voltage(&mut self, voltage: f32) {
        if voltage.is_nan() {
            log::error!("NaN voltage requested");
        }
        let max = u16::MAX as f32;
        // Float-to-int casts saturate, and map NaN to 0.
        let code = (voltage.clamp(0.0, Self::FULL_SCALE_VOLTAGE)
            * (max / Self::FULL_SCALE_VOLTAGE)
            + 0.5) as u16;
        self.write_raw(code);
    }
}
