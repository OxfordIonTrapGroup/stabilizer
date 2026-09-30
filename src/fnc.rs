//! Fibre noise cancellation (FNC) Pounder settings, see `bin/fnc.rs`.
use core::num::Wrapping;

use ad9959::{Acr, amplitude_to_acr, frequency_to_ftw};
use miniconf::Tree;

use crate::design_parameters::DDS_SYSTEM_CLK;

const DEFAULT_AOM_FREQUENCY: f32 = 80e6;

/// Pounder DDS and attenuator settings for one FNC channel
#[derive(Clone, Copy, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct PounderFncSettings {
    /// Frequency of the DDS output driving the double-pass AOM in Hz
    ///
    /// Range [0, 250 MHz]
    pub frequency_dds_out: f32,

    /// Frequency of the DDS mixing down the photodiode beat note in Hz
    ///
    /// Usually twice `frequency_dds_out`. Range [0, 250 MHz]
    pub frequency_dds_in: f32,

    /// Amplitude of the DDS output driving the AOM relative to full scale (10 dBm)
    ///
    /// Range [0, 1]
    pub amplitude_dds_out: f32,

    /// Amplitude of the DDS output mixing down the error signal relative to full scale (10 dBm)
    ///
    /// Range [0, 1]
    pub amplitude_dds_in: f32,

    /// Attenuation of the output channel driving the AOM in dB
    ///
    /// Range [0, 31.5] in steps of 0.5
    pub attenuation_out: f32,

    /// Attenuation of the input channel from the photodiode in dB
    ///
    /// Range [0, 31.5] in steps of 0.5
    pub attenuation_in: f32,
}

impl Default for PounderFncSettings {
    fn default() -> Self {
        Self {
            frequency_dds_out: DEFAULT_AOM_FREQUENCY,
            frequency_dds_in: 2.0 * DEFAULT_AOM_FREQUENCY,
            amplitude_dds_out: 0.1,
            amplitude_dds_in: 0.1,
            attenuation_out: 31.5,
            attenuation_in: 31.5,
        }
    }
}

impl PounderFncSettings {
    /// Get the DDS frequency tuning words and amplitude control registers.
    ///
    /// # Returns
    /// `[(ftw_in, acr_in), (ftw_out, acr_out)]`
    pub fn dds_words(
        &self,
    ) -> Result<[(Wrapping<i32>, Acr); 2], ad9959::Error> {
        let sysclk = DDS_SYSTEM_CLK.to_Hz() as f32;
        Ok([
            (
                frequency_to_ftw(self.frequency_dds_in, sysclk)?,
                amplitude_to_acr(self.amplitude_dds_in)?,
            ),
            (
                frequency_to_ftw(self.frequency_dds_out, sysclk)?,
                amplitude_to_acr(self.amplitude_dds_out)?,
            ),
        ])
    }
}
