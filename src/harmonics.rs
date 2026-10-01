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

    /// Evaluate the waveform at `M` consecutive samples, where the fundamental phase
    /// (`1 << 32` per turn) is `theta` for the first sample and advances by `step` per sample.
    ///
    /// To save time, the harmonics are only looked up once, at the centre of the batch; the
    /// samples are then obtained from the second-order expansion of the waveform around the
    /// centre (which comes for free, as the lookup yields both sine and cosine).
    ///
    /// For a harmonic of order `k`, the truncation error relative to its amplitude is at most
    /// `x^3 / 6`, where `x = k * (M - 1) / 2 * step` (in radians) is the harmonic phase
    /// advance over half the batch. This is intended for fundamentals that are slow compared
    /// to the batch rate: for instance, the 5th harmonic of 50 Hz in a batch of 8 samples at
    /// 781.25 kHz gives `x = 7 mrad` and an error of `6e-8`, far below the accuracy of the
    /// lookup itself, so the result is as good as from calling [Self::sample] for each sample.
    pub fn sample_batch<const M: usize>(
        &self,
        theta: i32,
        step: i32,
    ) -> [f32; M] {
        // Fundamental phase at the centre of the batch, i.e. (M - 1) / 2 samples in.
        let centre = theta.wrapping_add(step.wrapping_mul(M as i32 - 1) >> 1);

        // Value and first two derivatives w.r.t. the fundamental phase at the batch centre.
        let (mut y, mut dy, mut ddy) = (0.0, 0.0, 0.0);
        for (harmonic, order) in self.harmonics.iter().zip(1..) {
            let (cos, sin) =
                cossin(centre.wrapping_mul(order).wrapping_add(harmonic.phase));
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
        let c0 = y * COSSIN_SCALE;
        let c1 = dy * (COSSIN_SCALE * step);
        let c2 = ddy * (0.5 * COSSIN_SCALE * step * step);

        let offset = (M - 1) as f32 / 2.0;
        core::array::from_fn(|i| {
            let j = i as f32 - offset;
            c0 + j * (c1 + j * c2)
        })
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

    /// Largest deviation of `sample_batch()` from `sample()` for batches of 8 samples all
    /// around the fundamental period.
    fn max_batch_error<const N: usize>(
        waveform: &HarmonicWaveform<N>,
        step: i32,
    ) -> f32 {
        let mut max = 0.0f32;
        // Odd, so that all kinds of phases (including the wrap-around) are covered.
        const BATCH_SPACING: i32 = 0x0123_4567;
        for batch in 0..1000 {
            let theta =
                i32::MIN.wrapping_add(BATCH_SPACING.wrapping_mul(batch));
            let samples: [f32; 8] = waveform.sample_batch(theta, step);
            for (i, actual) in samples.into_iter().enumerate() {
                let expected =
                    waveform.sample(theta.wrapping_add(step * i as i32));
                max = max.max(actual - expected).max(expected - actual);
            }
        }
        max
    }

    #[test]
    fn batch_matches_samples() {
        // 50 Hz fundamental sampled at 781.25 kHz.
        const STEP: i32 = (50.0 * 1.28e-6 * (1u64 << 32) as f64) as i32;
        let waveform = HarmonicWaveform::new([
            Harmonic::new(1.0, 0.1),
            Harmonic::new(0.8, 0.7),
            Harmonic::new(0.6, -0.3),
            Harmonic::new(0.4, 0.45),
            Harmonic::new(1.0, 0.123),
        ]);
        // The expansion error is negligible here; what remains is the error of the lookup (9e-6
        // of each amplitude), which enters at different phases for the two methods.
        const TOLERANCE: f32 = 2.0 * 9e-6 * (1.0 + 0.8 + 0.6 + 0.4 + 1.0);
        let max = max_batch_error(&waveform, STEP);
        assert!(max < TOLERANCE, "{max}");
    }

    #[test]
    fn batch_expansion_order() {
        // A fundamental so fast that the phase advances by x = 0.1 rad over half a batch
        // (3.5 samples), where the missing third-order term amounts to x^3 / 6 = 1.7e-4. A
        // first-order expansion would be off by x^2 / 2 = 5e-3.
        const STEP: i32 =
            (0.1 / 3.5 / core::f64::consts::TAU * (1u64 << 32) as f64) as i32;
        let waveform = HarmonicWaveform::new([Harmonic::new(1.0, 0.2)]);
        let max = max_batch_error(&waveform, STEP);
        assert!(max > 1e-4 && max < 2e-4, "{max}");
    }

    #[test]
    fn batch_of_one() {
        let waveform = HarmonicWaveform::new([
            Harmonic::new(1.0, 0.1),
            Harmonic::new(0.5, 0.6),
        ]);
        for theta in (-8..8).map(|i| i * (1 << 28)) {
            let [sample] = waveform.sample_batch(theta, 12345);
            assert_close(sample, waveform.sample(theta));
        }
    }
}
