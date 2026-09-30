//! Run-time configuration of the Pounder DDS clocking, DDS channels, and attenuators.
use core::num::Wrapping;

use ad9959::{
    Acr, amplitude_to_acr, frequency_to_ftw, phase_to_pow, validate_clocking,
};
use arbitrary_int::{u5, u14};
use miniconf::Tree;

use crate::design_parameters::{DDS_MULTIPLIER, DDS_REF_CLK};

/// DDS channel waveform configuration
#[derive(Clone, Copy, Debug, Default, PartialEq, Tree)]
#[tree(meta(doc, typename))]
pub struct DdsChannelConfig {
    /// Output frequency in Hz
    pub frequency: f32,
    /// Phase offset in turns
    pub phase_offset: f32,
    /// Normalized amplitude in [0, 1]
    pub amplitude: f32,
}

/// A fully defined DDS profile in machine units
#[derive(Clone, Copy, Debug)]
pub struct Profile {
    /// Frequency tuning word (AD9959 CFTW0)
    pub ftw: Wrapping<i32>,
    /// Phase offset word (AD9959 CPOW0)
    pub pow: Wrapping<u14>,
    /// Amplitude control register (AD9959 ACR)
    pub acr: Acr,
}

impl DdsChannelConfig {
    /// Convert to machine units given the DDS system clock frequency in Hz.
    pub fn profile(
        &self,
        system_clock_frequency: f32,
    ) -> Result<Profile, ad9959::Error> {
        Ok(Profile {
            ftw: frequency_to_ftw(self.frequency, system_clock_frequency)?,
            pow: phase_to_pow(self.phase_offset),
            acr: amplitude_to_acr(self.amplitude)?,
        })
    }
}

/// Pounder channel configuration
#[derive(Clone, Copy, Debug, PartialEq, Tree)]
#[tree(meta(doc, typename))]
pub struct ChannelConfig {
    /// DDS waveform
    pub dds: DdsChannelConfig,
    /// Attenuation in dB, [0, 31.5] in steps of 0.5
    pub attenuation: f32,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            dds: DdsChannelConfig::default(),
            attenuation: 31.5,
        }
    }
}

/// DDS clock configuration
#[derive(Clone, Copy, Debug, PartialEq, Tree)]
#[tree(meta(doc, typename))]
pub struct ClockConfig {
    /// Reference clock multiplier: 1 (disabled) or 4-20
    pub multiplier: u8,
    /// Reference clock frequency in Hz
    pub reference_clock_frequency: f32,
    /// Use the external reference clock input
    pub external_clock: bool,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            multiplier: DDS_MULTIPLIER.value(),
            reference_clock_frequency: DDS_REF_CLK.to_Hz() as f32,
            external_clock: false,
        }
    }
}

impl ClockConfig {
    /// The reference clock multiplier as a PLL divider value.
    pub fn multiplier(&self) -> Result<u5, ad9959::Error> {
        u5::try_new(self.multiplier).or(Err(ad9959::Error::Bounds))
    }

    /// Validate the configuration and compute the DDS system clock frequency in Hz.
    pub fn system_clock_frequency(&self) -> Result<f32, ad9959::Error> {
        validate_clocking(self.reference_clock_frequency, self.multiplier()?)
    }
}

/// Pounder DDS clocking, DDS channel, and attenuator configuration
#[derive(Clone, Copy, Debug, Default, PartialEq, Tree)]
#[tree(meta(doc, typename))]
pub struct PounderConfig {
    /// DDS clock configuration
    pub clock: ClockConfig,
    /// Input (mixer LO) channels IN0, IN1
    pub in_channel: [ChannelConfig; 2],
    /// Output channels OUT0, OUT1
    pub out_channel: [ChannelConfig; 2],
}
