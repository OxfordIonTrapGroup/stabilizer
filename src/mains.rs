//! Mains phase tracking.
//!
//! A reciprocal PLL ([idsp::RPLL]) reconstructs the phase of a reference (the mains) from
//! timestamps of its edges. It has to be updated at regular intervals of `1 << dt2` timer ticks
//! (see [RPLLConfig]), each time with the timestamp of the edge in the preceding interval, if
//! any. [MainsPll] makes these updates on a fixed grid of timer ticks, independently of when it
//! is called. It can thus be driven from a task that runs at irregular times, as long as the
//! current time is known in timer ticks.

use core::num::Wrapping;

use dsp_process::SplitProcess;
use idsp::{RPLL, RPLLConfig};

/// Tracks the phase of a reference from timestamps of its edges.
///
/// Times are in ticks of the timer that timestamps the edges, wrapping at `1 << 32`. The
/// [RPLL] is updated at every multiple of `1 << dt2` ticks, with the timestamp of an edge
/// supplied in the first update at or after it.
#[derive(Clone, Default)]
pub struct MainsPll {
    rpll: RPLL,
    /// Timestamps closer than this to the previously accepted one are rejected as glitches.
    min_spacing: u32,
    /// The time of the next update, a multiple of `1 << dt2`.
    next: u32,
    /// The latest accepted timestamp.
    last: Option<u32>,
    /// An accepted timestamp not yet supplied to the RPLL.
    pending: Option<u32>,
}

impl MainsPll {
    /// Create a PLL whose first update is at time zero.
    ///
    /// # Args
    /// * `min_spacing` - Timestamps closer than this many ticks to the previously accepted one
    ///   are rejected as glitches.
    pub fn new(min_spacing: u32) -> Self {
        Self {
            min_spacing,
            ..Default::default()
        }
    }

    /// Supply the timestamp of an edge.
    ///
    /// Only the latest accepted timestamp is kept until the PLL is advanced past it.
    pub fn push(&mut self, timestamp: u32) {
        if self
            .last
            .is_none_or(|last| timestamp.wrapping_sub(last) >= self.min_spacing)
        {
            self.last = Some(timestamp);
            self.pending = Some(timestamp);
        }
    }

    /// Advance the PLL up to the time `now`.
    ///
    /// The timestamps of all edges up to `now` must have been supplied before. Calls must be
    /// less than `1 << 31` ticks apart.
    ///
    /// # Returns
    /// The phase and frequency estimate at the latest update.
    pub fn advance(&mut self, config: &RPLLConfig, now: u32) -> Estimate {
        let period = 1u32 << config.dt2;
        while now.wrapping_sub(self.next) as i32 >= 0 {
            let timestamp = self
                .pending
                .filter(|&t| self.next.wrapping_sub(t) as i32 >= 0);
            if timestamp.is_some() {
                self.pending = None;
            }
            config.process(&mut self.rpll, timestamp.map(|t| Wrapping(t as _)));
            self.next = self.next.wrapping_add(period);
        }
        Estimate {
            time: self.next.wrapping_sub(period),
            phase: self.rpll.phase().0,
            frequency: self.rpll.frequency().0,
            dt2: config.dt2,
        }
    }
}

/// The reference phase and frequency estimated at a PLL update.
#[derive(Copy, Clone, Debug)]
pub struct Estimate {
    /// The time of the update.
    time: u32,
    /// The phase at that time (`1 << 32` per turn, zero at the reference edges).
    phase: i32,
    /// The phase advance per `1 << dt2` ticks.
    frequency: u32,
    dt2: u8,
}

impl Estimate {
    /// The phase (`1 << 32` per turn, zero at the reference edges) at time `t`, extrapolated
    /// from the update.
    pub fn phase_at(&self, t: u32) -> i32 {
        self.phase
            .wrapping_add(self.step(t.wrapping_sub(self.time) as i32))
    }

    /// The phase advance over a number of ticks.
    pub fn step(&self, ticks: i32) -> i32 {
        ((self.frequency as i64).wrapping_mul(ticks as i64) >> self.dt2) as i32
    }
}

#[cfg(test)]
mod test {
    use super::*;

    const CONFIG: RPLLConfig = RPLLConfig {
        dt2: 10,
        shift_frequency: 22,
        shift_phase: 21,
    };

    // 50.01 Hz at 100 MHz
    const PERIOD: u64 = 1_999_600;
    const FIRST_EDGE: u64 = 1234;

    /// The latest edge at or before `t`.
    fn edge_before(t: u64) -> Option<u64> {
        t.checked_sub(FIRST_EDGE)
            .map(|dt| FIRST_EDGE + dt / PERIOD * PERIOD)
    }

    struct Rng(u32);

    impl Rng {
        fn next(&mut self, n: u32) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0 % n
        }
    }

    /// Advanced at irregular times, with glitches and across the wrap of the timer, the PLL
    /// must be identical to an RPLL updated on the grid.
    #[test]
    fn irregular_matches_grid() {
        let mut rng = Rng(0x1234_5678);
        let mut mains = MainsPll::new(500_000);
        let mut reference = RPLL::default();
        let mut grid = 0;
        let mut real = 0;
        let mut read = 0;
        // Past the timer wrap at 43 s.
        while real < 50 * 100_000_000 {
            real += 1 + rng.next(4000) as u64;
            // The capture register holds the latest edge, or a glitch 0.5 ms after it.
            if let Some(edge) = edge_before(real) {
                let glitch = edge + 50_000;
                if (edge / PERIOD) % 7 == 0 && glitch <= real && glitch > read {
                    mains.push(glitch as u32);
                } else if edge > read {
                    mains.push(edge as u32);
                }
            }
            read = real;
            // The latest processed sample was a little earlier.
            let now = real - rng.next(300) as u64;
            let estimate = mains.advance(&CONFIG, now as u32);

            while grid <= now {
                let timestamp = edge_before(grid)
                    .filter(|&edge| edge + (1 << CONFIG.dt2) > grid)
                    .map(|edge| Wrapping(edge as u32 as i32));
                CONFIG.process(&mut reference, timestamp);
                grid += 1 << CONFIG.dt2;
            }
            assert_eq!(estimate.time, (grid - (1 << CONFIG.dt2)) as u32);
            assert_eq!(estimate.phase, reference.phase().0);
            assert_eq!(estimate.frequency, reference.frequency().0);
        }
    }

    /// The phase is zero at the edges and advances with the reference frequency between
    /// updates.
    #[test]
    fn phase() {
        let mut mains = MainsPll::new(500_000);
        let mut max = 0f64;
        for i in 0..1_000_000u64 {
            let now = i << CONFIG.dt2;
            if let Some(edge) =
                edge_before(now).filter(|&edge| edge + (1 << CONFIG.dt2) > now)
            {
                mains.push(edge as u32);
            }
            let estimate = mains.advance(&CONFIG, now as u32);
            // Locked after 2 s
            if now < 200_000_000 {
                continue;
            }
            // Between this and the next update, as used for the samples.
            for t in (now..now + (1 << CONFIG.dt2)).step_by(128) {
                let expected =
                    ((t - FIRST_EDGE) % PERIOD) as f64 / PERIOD as f64;
                let actual = estimate.phase_at(t as u32) as u32 as f64
                    / (1u64 << 32) as f64;
                let err = (actual - expected + 0.5).rem_euclid(1.0) - 0.5;
                max = max.max(err.abs());
            }
        }
        assert!(max < 1e-5, "{max}");
    }
}
