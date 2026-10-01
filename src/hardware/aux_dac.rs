//! Auxiliary DAC outputs
//!
//! The MCU-internal 12-bit DAC (DAC1) provides two slow auxiliary analog outputs on PA4 (channel 0)
//! and PA5 (channel 1), complementing the auxiliary ADC inputs. The outputs are referenced to the
//! external 2.048 V analog reference (VREF+), and are operated with the internal output buffer
//! enabled.
//!
//! [super::setup::setup] hands out both channels in a calibrated, but disabled state (i.e. with the
//! pins in high-impedance analog mode, as after reset). Applications that use them need to enable
//! them first.
use super::hal::{self, traits::DacOut};

/// Auxiliary DAC channel 0 (PA4), disabled.
pub type AuxDac0Disabled = hal::dac::C1<hal::stm32::DAC, hal::dac::Disabled>;

/// Auxiliary DAC channel 1 (PA5), disabled.
pub type AuxDac1Disabled = hal::dac::C2<hal::stm32::DAC, hal::dac::Disabled>;

/// Auxiliary DAC channel 0 (PA4), enabled with output buffer.
pub type AuxDac0 = hal::dac::C1<hal::stm32::DAC, hal::dac::Enabled>;

/// Auxiliary DAC channel 1 (PA5), enabled with output buffer.
pub type AuxDac1 = hal::dac::C2<hal::stm32::DAC, hal::dac::Enabled>;

/// Output voltage corresponding to the full-scale code, given by the 2.048 V analog reference.
pub const FULL_SCALE_VOLTAGE: f32 = 2.048;

/// Largest (full-scale) output code of the 12-bit DAC.
const MAX_CODE: u16 = (1 << 12) - 1;

/// Voltage-based output interface for the auxiliary DAC channels.
pub trait AuxDacOutput: DacOut<u16> {
    /// Set the output voltage.
    ///
    /// Values outside of `[0, FULL_SCALE_VOLTAGE]` are clamped (with a warning logged); NaN is
    /// mapped to 0 V.
    ///
    /// # Returns
    /// The nominal output voltage corresponding to the DAC code actually set.
    fn set_voltage(&mut self, voltage: f32) -> f32 {
        if !(0.0..=FULL_SCALE_VOLTAGE).contains(&voltage) {
            log::warn!(
                "Aux DAC voltage {} V out of range [0, {}] V, clamping",
                voltage,
                FULL_SCALE_VOLTAGE
            );
        }
        let max = MAX_CODE as f32;
        // Float-to-int casts saturate, and map NaN to 0.
        let code =
            (voltage * (max / FULL_SCALE_VOLTAGE) + 0.5).clamp(0.0, max) as u16;
        self.set_value(code);
        code as f32 * (FULL_SCALE_VOLTAGE / max)
    }
}

impl<T: DacOut<u16>> AuxDacOutput for T {}
