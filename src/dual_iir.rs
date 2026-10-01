//! Settings shared by the `dual-iir` and `dual-iir-ll` applications.

use idsp::iir::{self, pid::Units};
use miniconf::Tree;
use num_traits::clamp;
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

/// The state of a [`Biquad`] with the input-independent part of its summing junction
/// precomputed.
///
/// A [`Biquad`] evaluates
///
/// `y0 = clamp(b0*x0 + b1*x1 + b2*x2 + a1*y1 + a2*y2 + u, min, max)`
///
/// when the input `x0` arrives. Everything but `b0*x0` is already known one sample earlier.
/// This splits the evaluation in two steps to minimize the delay from input to output:
///
/// * [`output()`](Self::output) computes `y0` from `x0` with a single multiplication, an
///   addition and the clamp.
/// * [`update()`](Self::update) then shifts `x0` and `y0` into the delay lines and precomputes
///   the remaining terms for the next sample. It can run after the output has been used.
///
/// The filter structure (direct form 1 with the clamped output fed back) is that of
/// [`Biquad`]. The results only differ by the rounding of the floating point additions due to
/// their different order.
#[derive(Clone, Debug, Default)]
pub struct Stage {
    /// The input and output delay lines, as for [`Biquad`].
    state: iir::DirectForm1<f32>,
    /// `b1*x1 + b2*x2 + a1*y1 + a2*y2 + u`
    partial: f32,
}

impl Stage {
    /// The output for the input `x0`.
    #[inline(always)]
    pub fn output(&self, biquad: &Biquad, x0: f32) -> f32 {
        clamp(
            biquad.coeff.ba[0] * x0 + self.partial,
            biquad.min,
            biquad.max,
        )
    }

    /// The output while holding: the previous output.
    #[inline(always)]
    pub fn hold(&self) -> f32 {
        self.state.y[0][0]
    }

    /// Shift the input `x0` and the output `y0` into the delay lines and precompute the
    /// input-independent terms for the next sample.
    #[inline(always)]
    pub fn update(&mut self, biquad: &Biquad, x0: f32, y0: f32) {
        self.state.x = [x0, self.state.x[0]];
        self.state.y[0] = [y0, self.state.y[0][0]];
        self.prepare(biquad);
    }

    /// Precompute the input-independent terms.
    ///
    /// To be called whenever the filter changes.
    #[inline(always)]
    pub fn prepare(&mut self, biquad: &Biquad) {
        let ba = &biquad.coeff.ba;
        let ([x1, x2], [y1, y2]) = (self.state.x, self.state.y[0]);
        // Two short independent sums instead of one long accumulation chain.
        self.partial =
            (ba[1] * x1 + ba[2] * x2) + ((ba[3] * y1 + ba[4] * y2) + biquad.u);
    }
}

#[cfg(test)]
mod test {
    extern crate std;

    use super::*;
    use dsp_process::SplitProcess;

    struct Rng(u32);

    impl Rng {
        /// Uniform in [-1, 1)
        fn next(&mut self) -> f32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            (self.0 as i32) as f32 / (1u32 << 31) as f32
        }
    }

    fn biquad(ba: [f32; 5], u: f32, limit: f32) -> Biquad {
        let mut biquad = Biquad::from(iir::Biquad::from(ba));
        biquad.u = u;
        biquad.min = -limit;
        biquad.max = limit;
        biquad
    }

    /// The [`Biquad`] in double precision.
    fn exact(filter: &Biquad, state: &mut [f64; 4], x0: f64) -> f64 {
        let ba = filter.coeff.ba.map(f64::from);
        let [x1, x2, y1, y2] = *state;
        let y0 = (ba[0] * x0
            + ba[1] * x1
            + ba[2] * x2
            + ba[3] * y1
            + ba[4] * y2
            + filter.u as f64)
            .clamp(filter.min as _, filter.max as _);
        *state = [x0, x1, y0, y1];
        y0
    }

    /// Run the idsp implementation and the split implementation side by side and return
    /// their maximum deviations from the double precision result.
    fn errors(filters: &[Biquad], samples: usize, hold: bool) -> [f32; 2] {
        let mut rng = Rng(0x1234_5678);
        let mut reference = iir::DirectForm1::<f32>::default();
        let mut stage = Stage::default();
        let mut double = [0f64; 4];
        let mut max = 0f32;
        let mut err = [0f32; 2];
        for (i, filter) in filters.iter().cycle().take(8).enumerate() {
            // Bumpless filter change
            stage.prepare(filter);
            for j in 0..samples {
                let x = (rng.next() * 3e4).round();
                let run = !hold || (i + j / 50) % 3 != 0;
                let (y_ref, y, y_exact) = if run {
                    (
                        filter.process(&mut reference, x),
                        stage.output(filter, x),
                        exact(filter, &mut double, x as _),
                    )
                } else {
                    let y = double[2];
                    double = [x as _, double[0], y, y];
                    (
                        iir::Biquad::<f32>::HOLD.process(&mut reference, x),
                        stage.hold(),
                        y,
                    )
                };
                stage.update(filter, x, y);
                assert!(y >= filter.min && y <= filter.max);
                err[0] = err[0].max((y_ref as f64 - y_exact).abs() as f32);
                err[1] = err[1].max((y as f64 - y_exact).abs() as f32);
                max = max.max(y.abs());
            }
        }
        // The test exercises a significant part of the output range.
        assert!(max > 1e3);
        err
    }

    #[test]
    fn proportional_is_exact() {
        let p = biquad([-0.75, 0., 0., 0., 0.], 100., 3e4);
        assert_eq!(errors(&[p], 1000, true), [0.; 2]);
    }

    #[test]
    fn accuracy() {
        // PI: kp = 0.5, ki*T = 1e-3, clamped
        let pi = biquad([0.5005, -0.4995, 0., 1., 0.], 0., 2e4);
        // PI with high proportional gain and slow integrator
        let pi_hi = biquad([20.000_01, -19.999_99, 0., 1., 0.], 0., 3e4);
        // Lowpass with complex poles
        let lp = biquad([0.02, 0.04, 0.02, 1.56, -0.64], -300., 3e4);
        // Lead/lag with offset, tight clamp
        let ll = biquad([3., -2.7, 0., 0.9, 0.], 1000., 5e3);
        for (name, filters, samples, hold) in [
            ("pi", &[pi.clone()][..], 100_000, false),
            ("pi_hi", &[pi_hi.clone()][..], 100_000, false),
            ("lp", &[lp.clone()][..], 100_000, false),
            ("ll", &[ll.clone()][..], 100_000, false),
            ("mixed", &[pi, lp, ll, pi_hi][..], 2000, true),
        ] {
            let [idsp, split] = errors(filters, samples, hold);
            std::println!("{name}: idsp {idsp:e}, split {split:e} LSB");
            // Both deviate from the exact result by accumulated rounding errors. The split
            // evaluation must be about as accurate as the idsp one.
            assert!(split <= 3. * idsp + 1e-2, "{name}: {split} vs {idsp}");
        }
    }
}
