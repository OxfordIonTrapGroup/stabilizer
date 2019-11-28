use core::sync::atomic;
use core::sync::atomic::{AtomicUsize, AtomicBool, Ordering};
use core::default::Default;
use core::marker::Copy;
use core::cmp::min;


pub const STORAGE_SIZE: usize = 1024;

// move to main.rs once const fns work with trait bounds...
// #[link_section = ".sram2.adc_buf"]
pub static mut ADC_BUF: RingBuffer<u8> = RingBuffer {
    storage: [0 as u8; STORAGE_SIZE],
    tail: AtomicUsize::new(0),
    head: AtomicUsize::new(0),
    write_lock: AtomicBool::new(false),
    read_lock: AtomicBool::new(false),
};

/// Thread-safe ring buffer.
///
/// # Notes
/// - The buffer can only be written from one thread at a time
/// - The buffer can only be read from one thread at a time
/// - Reading and writing simultaneously from different threads is allowed
/// - Clearing is both a read and a write
/// - Currently, this contract is enforced using Atomic-based critical sections.
///   This is a bit heavy-handed and likely to be removed in a future version...
///
/// # To do:
/// - Make STORAGE_SIZE a template param when stable
/// - Tests!
pub struct RingBuffer<T> where T: Default + Copy
{
    storage: [T; STORAGE_SIZE],
    tail: AtomicUsize,  // location of next write
    head: AtomicUsize,  // location of next read
    write_lock: AtomicBool,
    read_lock: AtomicBool,
}

#[allow(dead_code)]
impl<T> RingBuffer<T> where T: Default + Copy {

    /// should be const, but doesn't yet work with trait bounds...
    pub fn new() -> Self
    {
        Self {
            storage: [Default::default(); STORAGE_SIZE],
            tail: AtomicUsize::new(0),
            head: AtomicUsize::new(0),
            write_lock: AtomicBool::new(false),
            read_lock: AtomicBool::new(false),
        }
    }

    /// Clear the buffer
    ///
    /// # Panics
    ///
    /// Panics if the buffer is locked for read/write access
    pub fn clear(&mut self) {
        // FIXME: This shouldn't happen!
        // let wl = self.write_lock.compare_and_swap(false, true, Ordering::Acquire);
        // if wl {
        //     panic!("Attempt to clear a write-locked buffer.")
        // }
        let rl = self.read_lock.compare_and_swap(false, true, Ordering::Acquire);
        if rl {
            panic!("Attempt to clear a read-locked buffer.")
        }
        self.tail.store(self.head.load(Ordering::Acquire), Ordering::Release);
        self.write_lock.store(false, Ordering::Release);
        self.read_lock.store(false, Ordering::Release);
    }

    /// Enqueues a value
    ///
    /// # Returns Err(()) if the buffer is full.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is locked for write access
    pub fn enqueue(&mut self, data: T) -> Result<(), ()> {
        if false != self.write_lock.compare_and_swap(false, true, Ordering::Acquire) {
            panic!("Attempt to write to locked buffer.")
        }
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if (tail + 1) % STORAGE_SIZE == head {
            return Err(())
        }
        self.storage[tail] = data;
        self.tail.store((tail + 1) % STORAGE_SIZE, Ordering::Release);
        self.write_lock.store(false, Ordering::Release);
        Ok(())
    }

    /// Enqueues a slice
    ///
    /// # Returns Err(()) if the buffer is full.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is locked for write access
    pub fn enqueue_slice(&mut self, data: &[T]) -> Result<(), ()> {
        if false != self.write_lock.compare_and_swap(false, true, Ordering::Acquire) {
            panic!("Attempt to write to locked buffer.")
        }
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        let space = (STORAGE_SIZE + head - tail - 1) % STORAGE_SIZE;
        if data.len() > space {
            return Err(())
        }
        let size1 = min(STORAGE_SIZE, tail + data.len()) - tail;
        let size2 = data.len() - size1;
        self.storage[tail..tail+size1].copy_from_slice(&data[..size1]);
        self.storage[0..size2].copy_from_slice(&data[size1..]);
        self.tail.store((tail + data.len()) % STORAGE_SIZE, Ordering::Release);
        self.write_lock.store(false, Ordering::Release);
        Ok(())
    }

    /// Dequeues elements into a slice, returning the number of elements written
    ///
    /// # Panics
    ///
    /// Panics if the buffer is locked for read access
    pub fn dequeue_into(&mut self, buf: &mut [T]) -> usize {
        if false != self.read_lock.compare_and_swap(false, true, Ordering::Acquire) {
            panic!("Attempt to read from locked buffer.")
        }
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Relaxed);
        let available = (STORAGE_SIZE + tail - head) % STORAGE_SIZE;
        let to_copy = min(available, buf.len());

        let size1 = min(head + to_copy, STORAGE_SIZE) - head;
        buf[..size1].copy_from_slice(&self.storage[head..head+size1]);
        buf[size1..to_copy].copy_from_slice(&self.storage[..to_copy-size1]);
        self.head.store((head + to_copy) % STORAGE_SIZE, Ordering::Release);
        self.read_lock.store(false, Ordering::Release);
        to_copy
    }
}
