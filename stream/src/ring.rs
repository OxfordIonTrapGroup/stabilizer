//! Sample ring for interrupt-to-task handoff.

use core::sync::atomic::{
    AtomicU32,
    Ordering::{Relaxed, SeqCst},
    compiler_fence,
};

/// One set of ADC0, ADC1, DAC0, DAC1 samples.
pub type Sample = [u16; 4];

/// The result of [`SampleRing::pop()`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Pop<const B: usize> {
    /// Less than one batch is pending.
    Empty,
    /// A batch in [`crate::Format::AdcDacData`] layout.
    Batch([[u16; B]; 4]),
    /// Batches skipped after an overrun.
    Lost(u32),
}

/// An overwriting ring of the `N` most recent samples (`N` a power of two).
///
/// A single producer and all readers must run on the same core. The producer may
/// preempt readers, but readers must never preempt the producer. One consumer calls
/// [`pop()`](Self::pop); other readers may use [`latest()`](Self::latest).
///
/// Each push completes before any reader can resume, so producer stores need no
/// barriers. Atomic words prevent torn accesses; consumer compiler fences keep
/// sample loads between the availability and overrun checks in `pop()`.
pub struct SampleRing<const N: usize> {
    /// The total number of samples pushed (wrapping).
    head: AtomicU32,
    data: [[AtomicU32; 2]; N],
}

impl<const N: usize> Default for SampleRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> SampleRing<N> {
    const MASK: usize = {
        assert!(N.is_power_of_two());
        N - 1
    };

    /// Create an empty ring.
    pub const fn new() -> Self {
        Self {
            head: AtomicU32::new(0),
            data: [const { [const { AtomicU32::new(0) }; 2] }; N],
        }
    }

    /// The total number of samples pushed so far (wrapping).
    #[inline(always)]
    pub fn head(&self) -> u32 {
        self.head.load(Relaxed)
    }

    /// Push a sample, overwriting the oldest, and return the wrapping total.
    #[inline(always)]
    pub fn push(&self, sample: Sample) -> u32 {
        let head = self.head.load(Relaxed);
        let slot = &self.data[head as usize & Self::MASK];
        slot[0].store(sample[0] as u32 | (sample[1] as u32) << 16, Relaxed);
        slot[1].store(sample[2] as u32 | (sample[3] as u32) << 16, Relaxed);
        let head = head.wrapping_add(1);
        self.head.store(head, Relaxed);
        head
    }

    /// Read the slot at a wrapping sample index without checking for overwrites.
    ///
    /// The caller must ensure the slot remains valid throughout the read. Its two
    /// words are loaded separately.
    #[inline]
    pub fn get(&self, index: u32) -> Sample {
        let slot = &self.data[index as usize & Self::MASK];
        let (a, b) = (slot[0].load(Relaxed), slot[1].load(Relaxed));
        [a as u16, (a >> 16) as u16, b as u16, (b >> 16) as u16]
    }

    /// Read the latest sample, or zeros before the first push.
    ///
    /// The producer must advance by fewer than `N` samples during this call to
    /// avoid overwriting the slot. This read does not detect overruns.
    #[inline]
    pub fn latest(&self) -> Sample {
        let head = self.head();
        compiler_fence(SeqCst);
        self.get(head.wrapping_sub(1))
    }

    /// Take the oldest pending batch of `B` samples.
    ///
    /// `tail` is the wrapping index of the next sample to consume. Initialize it
    /// to zero or the current [`head()`](Self::head). The producer must stay less
    /// than 2^32 samples ahead of `tail`, including while this call is preempted.
    pub fn pop<const B: usize>(&self, tail: &mut u32) -> Pop<B> {
        const {
            assert!(B.is_power_of_two() && B <= N);
        }
        if (self.head().wrapping_sub(*tail) as usize) < B {
            return Pop::Empty;
        }
        // Keep sample loads after the availability check and before validation.
        compiler_fence(SeqCst);
        let mut batch = [[0; B]; 4];
        for i in 0..B {
            let sample = self.get(tail.wrapping_add(i as u32));
            for (batch, sample) in batch.iter_mut().zip(sample) {
                batch[i] = sample;
            }
        }
        compiler_fence(SeqCst);
        let pending = self.head().wrapping_sub(*tail);
        if pending as usize > N {
            let lost = pending / B as u32;
            *tail = tail.wrapping_add(lost * B as u32);
            Pop::Lost(lost)
        } else {
            *tail = tail.wrapping_add(B as u32);
            Pop::Batch(batch)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn sample(i: u32) -> Sample {
        [i as u16, !i as u16, (i >> 16) as u16, 0x55aa]
    }

    fn fill<const N: usize>(ring: &SampleRing<N>, start: u32, n: u32) {
        for i in 0..n {
            assert_eq!(
                ring.push(sample(start.wrapping_add(i))),
                start.wrapping_add(i).wrapping_add(1)
            );
        }
    }

    #[test]
    fn batches() {
        let ring = SampleRing::<16>::new();
        let mut tail = 0;
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Empty);
        fill(&ring, 0, 3);
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Empty);
        fill(&ring, 3, 6);
        assert_eq!(ring.latest(), sample(8));
        for k in 0..2 {
            let Pop::Batch(batch) = ring.pop::<4>(&mut tail) else {
                panic!()
            };
            for i in 0..4 {
                let s = sample(4 * k + i as u32);
                assert_eq!(
                    [batch[0][i], batch[1][i], batch[2][i], batch[3][i]],
                    s
                );
            }
        }
        assert_eq!(tail, 8);
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Empty);
    }

    #[test]
    fn full_is_not_lost() {
        let ring = SampleRing::<16>::new();
        let mut tail = 0;
        fill(&ring, 0, 16);
        for _ in 0..4 {
            assert!(matches!(ring.pop::<4>(&mut tail), Pop::Batch(_)));
        }
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Empty);
    }

    #[test]
    fn lapped() {
        let ring = SampleRing::<16>::new();
        let mut tail = 0;
        fill(&ring, 0, 16 + 7);
        // 23 pending: 5 complete batches lost, 3 samples remain
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Lost(5));
        assert_eq!(tail, 20);
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Empty);
        fill(&ring, 23, 1);
        let Pop::Batch(batch) = ring.pop::<4>(&mut tail) else {
            panic!()
        };
        assert_eq!(batch[0], [20, 21, 22, 23]);
    }

    #[test]
    fn wrapping_index() {
        let ring = SampleRing::<16>::new();
        let start = u32::MAX - 5;
        ring.head.store(start, Relaxed);
        let mut tail = start;
        fill(&ring, start, 12);
        assert_eq!(ring.head(), 6);
        for k in 0..3 {
            let Pop::Batch(batch) = ring.pop::<4>(&mut tail) else {
                panic!()
            };
            assert_eq!(batch[0][0], start.wrapping_add(4 * k) as u16);
        }
        assert_eq!(tail, 6);
        fill(&ring, 6, 16 + 4);
        assert_eq!(ring.pop::<4>(&mut tail), Pop::Lost(5));
    }
}
