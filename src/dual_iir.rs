//! Settings shared by the `dual-iir` and `dual-iir-ll` applications.

use idsp::iir::{self, pid::Units};
use miniconf::Tree;
use serde::{Deserialize, Serialize};

use crate::{convert::Gain, pounder_config::PounderConfig};

/// The number of cascaded IIR biquads per channel. Select 1 or 2!
pub const IIR_CASCADE_LENGTH: usize = 1;

/// The filter built from a [`BiquadRepr`].
pub type Biquad = iir::BiquadClamp<f32, f32>;

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename = "BiquadReprTree"))]
pub struct BiquadRepr {
    /// Biquad parameters
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.repr)]
    _typ: (),
    repr: iir::repr::BiquadRepr<f32, f32>,
}

impl Default for BiquadRepr {
    fn default() -> Self {
        let mut i = iir::BiquadClamp::from(iir::Biquad::IDENTITY);
        i.min = -i16::MAX as _;
        i.max = i16::MAX as _;
        Self {
            _typ: (),
            repr: iir::repr::BiquadRepr::Raw(i),
        }
    }
}

impl BiquadRepr {
    /// Build the filter for the given time and input/output units.
    pub fn build(&self, units: &Units<f32>) -> Biquad {
        self.repr.build(units)
    }
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Default)]
pub enum Run {
    #[default]
    /// Run
    Run,
    /// Hold
    Hold,
    /// Hold controlled by corresponding digital input
    External,
}

impl Run {
    /// Whether the filter runs given the state of the digital input.
    #[inline]
    pub fn run(&self, di: bool) -> bool {
        match self {
            Self::Run => true,
            Self::Hold => false,
            Self::External => di,
        }
    }
}

/// A ADC-DAC channel
#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Channel {
    /// Analog Front End (AFE) gain.
    #[tree(with=miniconf::leaf)]
    pub gain: Gain,
    /// Biquad
    pub biquad: [BiquadRepr; IIR_CASCADE_LENGTH],
    /// Run/Hold behavior
    #[tree(with=miniconf::leaf)]
    pub run: Run,
    /// Signal generator configuration to add to the DAC0/DAC1 outputs
    pub source: signal_generator::Config,
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct DualIir {
    /// Channel configuration
    pub ch: [Channel; 2],
    /// Trigger both signal sources
    #[tree(with=miniconf::leaf)]
    pub trigger: bool,
    /// Telemetry output period in seconds.
    #[tree(with=miniconf::leaf)]
    pub telemetry_period: f32,
    /// Target IP and port for UDP streaming.
    ///
    /// Can be multicast.
    #[tree(with=miniconf::leaf)]
    pub stream: stream::Target,
    /// Pounder DDS clock, DDS channel, and attenuator configuration.
    ///
    /// Only present if a Pounder is detected.
    pub pounder: Option<PounderConfig>,
}

impl Default for DualIir {
    fn default() -> Self {
        Self {
            telemetry_period: 10.0,
            trigger: false,
            stream: Default::default(),
            ch: Default::default(),
            pounder: None,
        }
    }
}
