//! Harmonic waveform synthesis.
//!
//! Evaluates a sum of harmonics of a common fundamental, each with its own amplitude and
//! phase, as a function of the fundamental phase. The evaluation is stateless, so the
//! waveform follows the source of that phase (e.g. an [`idsp::RPLL`] locked to an external
//! reference) without accumulating any drift.

use idsp::cossin;

/// A single harmonic, `amplitude * sin(order * theta + phase)`.
#[derive(Copy, Clone, Debug)]
pub struct Harmonic {
    /// Amplitude in output units.
    amplitude: f32,
    /// Phase offset, `1 << 32` per turn.
    phase: i32,
}

impl Harmonic {
    /// Create a harmonic from its `amplitude` (in output units) and `phase` offset in turns.
    pub fn new(amplitude: f32, phase: f32) -> Self {
        const TURN: f32 = (1u64 << 32) as f32;
        // Via i64 so that phases outside [-0.5, 0.5) turns wrap instead of saturating.
        let phase = (phase * TURN) as i64 as i32;
        Self { amplitude, phase }
    }
}

/// A sum of the first `N` harmonics of a fundamental.
#[derive(Copy, Clone, Debug)]
pub struct HarmonicWaveform<const N: usize> {
    /// `harmonics[k]` is harmonic order `k + 1`, i.e. the fundamental comes first.
    harmonics: [Harmonic; N],
}

impl<const N: usize> HarmonicWaveform<N> {
    /// Create a waveform from its harmonics, starting with the fundamental.
    pub const fn new(harmonics: [Harmonic; N]) -> Self {
        Self { harmonics }
    }

    /// Evaluate the waveform at fundamental phase `theta` (`1 << 32` per turn).
    pub fn sample(&self, theta: i32) -> f32 {
        self.harmonics
            .iter()
            .zip(1..)
            .map(|(harmonic, order)| {
                let phase =
                    theta.wrapping_mul(order).wrapping_add(harmonic.phase);
                harmonic.amplitude * (cossin(phase).1 as f32 * COSSIN_SCALE)
            })
            .sum()
    }

    /// Expand the waveform around the sample `centre`, where the fundamental phase
    /// (`1 << 32` per turn) is `theta` and advances by `step` per sample.
    ///
    /// The harmonics are only looked up once, at the centre; the samples around it are then
    /// obtained from the second-order expansion of the waveform (which comes for free, as the
    /// lookup yields both sine and cosine).
    ///
    /// For a harmonic of order `k`, the truncation error relative to its amplitude is at most
    /// `x^3 / 6`, where `x = k * j * step` (in radians) is the harmonic phase advance from the
    /// centre to the sample `j` samples away. This is intended for fundamentals that are slow
    /// compared to the sample rate: for instance, the 5th harmonic of 50 Hz sampled at
    /// 781.25 kHz gives `x = 16 mrad` at 8 samples from the centre and an error of `7e-7`,
    /// far below the accuracy of the lookup itself, so the result is as good as from calling
    /// [Self::sample] for each sample.
    pub fn expand(&self, centre: u32, theta: i32, step: i32) -> Expansion {
        // Value and first two derivatives w.r.t. the fundamental phase at the centre.
        let (mut y, mut dy, mut ddy) = (0.0, 0.0, 0.0);
        for (harmonic, order) in self.harmonics.iter().zip(1..) {
            let (cos, sin) =
                cossin(theta.wrapping_mul(order).wrapping_add(harmonic.phase));
            let k = order as f32;
            let sin = harmonic.amplitude * sin as f32;
            let cos = harmonic.amplitude * cos as f32;
            y += sin;
            dy += k * cos;
            ddy -= (k * k) * sin;
        }

        // Coefficients of the expansion in the sample index relative to the centre.
        const RADIAN_PER_LSB: f32 =
            core::f32::consts::TAU / (1u64 << 32) as f32;
        let step = step as f32 * RADIAN_PER_LSB;
        Expansion {
            centre,
            coefficients: [
                y * COSSIN_SCALE,
                dy * (COSSIN_SCALE * step),
                ddy * (0.5 * COSSIN_SCALE * step * step),
            ],
        }
    }
}

/// A waveform approximated by its second-order expansion around a sample.
///
/// See [HarmonicWaveform::expand].
#[derive(Copy, Clone, Debug, Default)]
pub struct Expansion {
    /// The index of the sample at the centre of the expansion (wrapping).
    centre: u32,
    /// The value at the centre, and the first and second order coefficients per sample.
    coefficients: [f32; 3],
}

impl Expansion {
    /// The waveform at the sample with the given index (wrapping).
    #[inline(always)]
    pub fn sample(&self, index: u32) -> f32 {
        let j = index.wrapping_sub(self.centre) as i32 as f32;
        let [c0, c1, c2] = self.coefficients;
        c0 + j * (c1 + j * c2)
    }
}

// cossin() output is scaled to 1 << 31 (to within 2e-5).
const COSSIN_SCALE: f32 = 1.0 / (1u64 << 31) as f32;

#[cfg(test)]
mod tests {
    use super::*;

    const QUARTER_TURN: i32 = 1 << 30;

    fn assert_close(actual: f32, expected: f32) {
        const TOLERANCE: f32 = 1e-4;
        assert!(
            (actual - expected) < TOLERANCE && (expected - actual) < TOLERANCE,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn fundamental() {
        let waveform = HarmonicWaveform::new([Harmonic::new(2.0, 0.0)]);
        assert_close(waveform.sample(0), 0.0);
        assert_close(waveform.sample(QUARTER_TURN), 2.0);
        assert_close(waveform.sample(i32::MIN), 0.0);
        assert_close(waveform.sample(-QUARTER_TURN), -2.0);
    }

    #[test]
    fn harmonic_order() {
        let silent = Harmonic::new(0.0, 0.0);
        let waveform =
            HarmonicWaveform::new([silent, silent, Harmonic::new(1.0, 0.0)]);
        // The third harmonic completes three quarter turns per quarter turn of the fundamental.
        assert_close(waveform.sample(QUARTER_TURN), -1.0);
        assert_close(waveform.sample(QUARTER_TURN / 3), 1.0);
    }

    #[test]
    fn phase_wraps() {
        // Phases beyond half a turn must wrap around rather than saturate.
        for phase in [-1.75, -0.75, 0.25, 1.25] {
            let waveform = HarmonicWaveform::new([Harmonic::new(1.0, phase)]);
            assert_close(waveform.sample(0), 1.0);
        }
        for phase in [-1.25, -0.25, 0.75, 1.75] {
            let waveform = HarmonicWaveform::new([Harmonic::new(1.0, phase)]);
            assert_close(waveform.sample(0), -1.0);
        }
    }

    #[test]
    fn superposition() {
        let silent = Harmonic::new(0.0, 0.0);
        let a = Harmonic::new(1.0, 0.1);
        let b = Harmonic::new(0.5, 0.6);
        let sum = HarmonicWaveform::new([a, b]);
        for theta in (-8..8).map(|i| i * (1 << 28)) {
            let a_only = HarmonicWaveform::new([a, silent]).sample(theta);
            let b_only = HarmonicWaveform::new([silent, b]).sample(theta);
            assert_close(sum.sample(theta), a_only + b_only);
        }
    }

    /// Largest deviation of the expansion from `sample()` over the samples up to `reach`
    /// samples from the centre, for expansions all around the fundamental period.
    fn max_expansion_error<const N: usize>(
        waveform: &HarmonicWaveform<N>,
        step: i32,
        reach: i32,
    ) -> f32 {
        let mut max = 0.0f32;
        // Odd, so that all kinds of phases (including the wrap-around) are covered.
        const SPACING: i32 = 0x0123_4567;
        for i in 0..1000 {
            let theta = i32::MIN.wrapping_add(SPACING.wrapping_mul(i));
            // Also covers the wrap-around of the sample index.
            let centre = (i as u32).wrapping_sub(500);
            let expansion = waveform.expand(centre, theta, step);
            for j in -reach..=reach {
                let actual = expansion.sample(centre.wrapping_add(j as u32));
                let expected = waveform.sample(theta.wrapping_add(step * j));
                max = max.max(actual - expected).max(expected - actual);
            }
        }
        max
    }

    #[test]
    fn expansion_matches_samples() {
        // 50 Hz fundamental sampled at 781.25 kHz.
        const STEP: i32 = (50.0 * 1.28e-6 * (1u64 << 32) as f64) as i32;
        let waveform = HarmonicWaveform::new([
            Harmonic::new(1.0, 0.1),
            Harmonic::new(0.8, 0.7),
            Harmonic::new(0.6, -0.3),
            Harmonic::new(0.4, 0.45),
            Harmonic::new(1.0, 0.123),
        ]);
        // The expansion error is negligible here, even at twice the 8 samples from the centre
        // that `current_sense` typically uses it for; what remains is the error of the lookup
        // (9e-6 of each amplitude), which enters at different phases for the two methods.
        const TOLERANCE: f32 = 2.0 * 9e-6 * (1.0 + 0.8 + 0.6 + 0.4 + 1.0);
        let max = max_expansion_error(&waveform, STEP, 16);
        assert!(max < TOLERANCE, "{max}");
    }

    #[test]
    fn expansion_order() {
        // A fundamental so fast that the phase advances by x = 0.1 rad over 4 samples, where
        // the missing third-order term amounts to x^3 / 6 = 1.7e-4. A first-order expansion
        // would be off by x^2 / 2 = 5e-3.
        const STEP: i32 =
            (0.1 / 4.0 / core::f64::consts::TAU * (1u64 << 32) as f64) as i32;
        let waveform = HarmonicWaveform::new([Harmonic::new(1.0, 0.2)]);
        let max = max_expansion_error(&waveform, STEP, 4);
        assert!(max > 1e-4 && max < 2e-4, "{max}");
    }

    #[test]
    fn expansion_centre() {
        let waveform = HarmonicWaveform::new([
            Harmonic::new(1.0, 0.1),
            Harmonic::new(0.5, 0.6),
        ]);
        for theta in (-8..8).map(|i| i * (1 << 28)) {
            let expansion = waveform.expand(u32::MAX, theta, 12345);
            assert_close(expansion.sample(u32::MAX), waveform.sample(theta));
        }
    }
}
