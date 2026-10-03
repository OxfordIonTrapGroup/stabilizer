//! Laser lock support for the `l674` application, see `bin/l674.rs`.
//!
//! The parts of the application which do not touch the hardware: the gain ramp applied to
//! the error signal when the lock is switched on, and the lock detection on the cavity
//! transmission.

/// Linear ramp of the error signal gain from zero to one after the lock is switched on.
///
/// This softens the transient when the controllers are enabled, in particular the kick to
/// the PZTs from the error signal having been outside of the linear range.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GainRamp {
    /// The current gain, in `[0, 1]`.
    current: f32,
    /// The gain increment per sample.
    increment: f32,
}

impl Default for GainRamp {
    /// A finished ramp (unity gain).
    fn default() -> Self {
        Self {
            current: 1.0,
            increment: 0.0,
        }
    }
}

impl GainRamp {
    /// Restart the ramp, reaching unity gain after `ramp_time` (in seconds, for the given
    /// sample period).
    ///
    /// A `ramp_time` of zero (or less) means no ramp: the gain is unity straight away.
    pub fn start(&mut self, ramp_time: f32, sample_period: f32) {
        if ramp_time > 0.0 {
            self.current = 0.0;
            self.increment = sample_period / ramp_time;
        } else {
            *self = Self::default();
        }
    }

    /// The gain for the current sample, advancing the ramp by one sample.
    #[inline]
    pub fn step(&mut self) -> f32 {
        let gain = self.current;
        if gain < 1.0 {
            self.current = (gain + self.increment).min(1.0);
        }
        gain
    }

    /// Whether the ramp has reached unity gain.
    pub fn is_finished(&self) -> bool {
        self.current >= 1.0
    }
}

/// The lock detection parameters in machine units, see [LockDetect].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LockDetectConfig {
    /// Transmission threshold in ADC1 codes.
    threshold: i16,
    /// Decrement of the holdoff counter per sample above threshold (`u32::MAX` corresponds
    /// to the reset time).
    decrement: u32,
}

impl Default for LockDetectConfig {
    /// Never detect a lock.
    fn default() -> Self {
        Self {
            threshold: i16::MAX,
            decrement: 1,
        }
    }
}

impl LockDetectConfig {
    /// Build the configuration from the settings.
    ///
    /// # Args
    /// * `threshold` - The transmission threshold in volts at the ADC1 input.
    /// * `afe_gain` - The gain of the ADC1 analog front end.
    /// * `reset_time` - The time the transmission has to stay above the threshold for the
    ///   lock to be reported, in seconds.
    /// * `sample_period` - The sample period in seconds.
    ///
    /// # Returns
    /// The configuration, or `Err` with a description if the threshold is out of the ADC
    /// range (the reset time is clamped to the representable range).
    pub fn new(
        threshold: f32,
        afe_gain: f32,
        reset_time: f32,
        sample_period: f32,
    ) -> Result<Self, &'static str> {
        let threshold = crate::convert::AdcCode::try_from(threshold * afe_gain)
            .map_err(|_| "Lock detect threshold out of the ADC range")?;
        let reset_samples = (reset_time / sample_period).max(1.0);
        // Counts down from `u32::MAX` in `reset_samples` steps; a decrement of zero would
        // never complete.
        let decrement = ((u32::MAX as f32 / reset_samples) as u32).max(1);
        Ok(Self {
            threshold: threshold.into(),
            decrement,
        })
    }

    /// The transmission threshold in ADC1 codes.
    pub fn threshold(&self) -> i16 {
        self.threshold
    }
}

/// Lock detection on the cavity transmission (ADC1).
///
/// The lock is reported (`locked()`) once the transmission has stayed above the threshold
/// for the reset time, and the report is withdrawn as soon as one sample is below the
/// threshold. The transmission is also low-pass filtered (two cascaded first-order sections
/// with a time constant of `1 << LOWPASS_LOG2_TC` samples each, about 10 ms at 781 kHz), for
/// reading it out over the network.
#[derive(Clone, Debug)]
pub struct LockDetect {
    /// The state of the two low-pass sections, in ADC codes scaled by `1 << FILTER_SHIFT`
    /// (fixed point, as `f32` would lose the increments of a slow filter).
    filter: [i64; 2],
    /// Holdoff counter, counting down from `u32::MAX` while above threshold.
    counter: u32,
    locked: bool,
}

impl Default for LockDetect {
    fn default() -> Self {
        Self {
            filter: [0; 2],
            counter: u32::MAX,
            locked: false,
        }
    }
}

impl LockDetect {
    /// log2 of the time constant of each low-pass section in samples.
    pub const LOWPASS_LOG2_TC: u32 = 13;
    /// Fractional bits of the filter state.
    const FILTER_SHIFT: u32 = 32;

    /// Process one ADC1 sample, returning whether the lock is now detected.
    #[inline]
    pub fn update(&mut self, config: &LockDetectConfig, sample: i16) -> bool {
        let x = i64::from(sample) << Self::FILTER_SHIFT;
        self.filter[0] += (x - self.filter[0]) >> Self::LOWPASS_LOG2_TC;
        self.filter[1] +=
            (self.filter[0] - self.filter[1]) >> Self::LOWPASS_LOG2_TC;

        if sample < config.threshold {
            self.locked = false;
            self.counter = u32::MAX;
        } else if self.counter > 0 {
            self.counter = self.counter.saturating_sub(config.decrement);
            if self.counter == 0 {
                self.locked = true;
            }
        }
        self.locked
    }

    /// Whether the lock is currently detected.
    pub fn locked(&self) -> bool {
        self.locked
    }

    /// The filtered transmission in ADC1 codes.
    pub fn filtered(&self) -> f32 {
        Self::codes(self.filter[1])
    }

    fn codes(state: i64) -> f32 {
        state as f32 / (1u64 << Self::FILTER_SHIFT) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_PERIOD: f32 = 1.28e-6;

    #[test]
    fn ramp_reaches_unity_after_ramp_time() {
        let mut ramp = GainRamp::default();
        assert_eq!(ramp.step(), 1.0);
        ramp.start(1e-3, SAMPLE_PERIOD);
        let samples = (1e-3 / SAMPLE_PERIOD) as usize;
        assert_eq!(ramp.step(), 0.0);
        let mut last = 0.0;
        for _ in 1..samples - 1 {
            let gain = ramp.step();
            assert!(gain > last && gain < 1.0, "gain {gain} after {last}");
            last = gain;
        }
        assert!(!ramp.is_finished());
        // Allow a couple of samples of rounding.
        for _ in 0..4 {
            ramp.step();
        }
        assert!(ramp.is_finished());
        assert_eq!(ramp.step(), 1.0);
    }

    #[test]
    fn ramp_without_time_is_unity() {
        let mut ramp = GainRamp::default();
        ramp.start(0.0, SAMPLE_PERIOD);
        assert_eq!(ramp.step(), 1.0);
        ramp.start(-1.0, SAMPLE_PERIOD);
        assert_eq!(ramp.step(), 1.0);
    }

    #[test]
    fn lock_detect_config() {
        // 0.1 V at gain 10 is 1 V at the ADC, i.e. 1/10.24 of full scale.
        let config =
            LockDetectConfig::new(0.1, 10.0, 1e-3, SAMPLE_PERIOD).unwrap();
        // (Truncated after the floating point conversion.)
        assert!((config.threshold() - (32768.0 / 10.24) as i16).abs() <= 1);
        let samples = 1e-3 / SAMPLE_PERIOD;
        assert_eq!(config.decrement, (u32::MAX as f32 / samples) as u32);
        assert!(LockDetectConfig::new(2.0, 10.0, 1e-3, SAMPLE_PERIOD).is_err());
        // Too short and too long reset times are clamped.
        assert_eq!(
            LockDetectConfig::new(0.1, 1.0, 0.0, SAMPLE_PERIOD)
                .unwrap()
                .decrement,
            u32::MAX
        );
        assert_eq!(
            LockDetectConfig::new(0.1, 1.0, 1e9, SAMPLE_PERIOD)
                .unwrap()
                .decrement,
            1
        );
    }

    #[test]
    fn lock_detect_holdoff() {
        let config =
            LockDetectConfig::new(0.1, 1.0, 1e-3, SAMPLE_PERIOD).unwrap();
        let reset_samples = (1e-3 / SAMPLE_PERIOD) as usize;
        let high = config.threshold() + 100;
        let mut detect = LockDetect::default();
        assert!(!detect.locked());
        for i in 0..reset_samples - 1 {
            assert!(!detect.update(&config, high), "locked after {i} samples");
        }
        // Reported within a couple of samples of the reset time.
        for _ in 0..3 {
            detect.update(&config, high);
        }
        assert!(detect.locked());
        // One sample below the threshold withdraws the report at once.
        assert!(!detect.update(&config, config.threshold() - 1));
        assert!(!detect.locked());
        for _ in 0..reset_samples / 2 {
            assert!(!detect.update(&config, high));
        }
        // The threshold itself counts as above.
        for _ in 0..reset_samples {
            detect.update(&config, config.threshold());
        }
        assert!(detect.locked());
    }

    #[test]
    fn lock_detect_filter_settles() {
        let config = LockDetectConfig::default();
        let mut detect = LockDetect::default();
        for _ in 0..20 << LockDetect::LOWPASS_LOG2_TC {
            detect.update(&config, 1000);
        }
        assert!((detect.filtered() - 1000.0).abs() < 1e-3);
        // With the default configuration, the lock is never detected.
        assert!(!detect.locked());
        // Time constant: after one TC of a step from 1000 to 0, the first section is at
        // e^-1 and the second a bit above; both well above zero.
        for _ in 0..1 << LockDetect::LOWPASS_LOG2_TC {
            detect.update(&config, 0);
        }
        let expected_first = 1000.0 * (-1.0f32).exp();
        assert!(
            (LockDetect::codes(detect.filter[0]) - expected_first).abs() < 1.0
        );
        assert!(detect.filtered() > expected_first);
    }
}
