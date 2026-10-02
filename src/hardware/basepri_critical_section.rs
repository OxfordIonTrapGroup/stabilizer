//! `critical-section` implementation that only masks interrupts up to a priority ceiling
//!
//! The usual single-core implementation (`cortex-m/critical-section-single-core`) disables all
//! interrupts. Library code takes critical sections from low-priority tasks: the USB driver
//! (`synopsys-usb-otg`) for every `poll()` and endpoint read, and the RTIC timer queue for every
//! `Systick::delay()` and SysTick tick. Each of these would delay the DSP task (and could make it
//! overrun).
//!
//! Instead, critical sections here raise `BASEPRI` to [`CRITICAL_SECTION_PRIORITY`], so that tasks
//! above it (the DSP `process` tasks and the SPI error handlers) are never delayed by them.
//!
//! This is sound as long as no code running above that priority enters a critical section or
//! accesses data protected by one: all contexts that can enter a critical section are then masked
//! while one is held. `acquire()` checks this at runtime and panics otherwise. Note that this also
//! applies to all exception handlers that use critical sections (e.g. SysTick, see
//! [`super::setup::setup`]), and to RTIC resources shared with priority 16 tasks (RTIC locks those
//! with a critical section).

use core::sync::atomic::{Ordering, compiler_fence};
use cortex_m::{
    peripheral::{NVIC, SCB},
    register::{basepri, basepri_max, primask},
};

use super::hal::stm32::NVIC_PRIO_BITS;

/// Highest (RTIC logical) priority of any context that may enter a critical section
///
/// Critical sections mask interrupts up to and including this priority.
pub const CRITICAL_SECTION_PRIORITY: u8 = 2;

/// Convert an RTIC logical priority to the NVIC hardware priority encoding.
pub const fn logical2hw(logical: u8) -> u8 {
    ((1 << NVIC_PRIO_BITS) - logical) << (8 - NVIC_PRIO_BITS)
}

const CEILING: u8 = logical2hw(CRITICAL_SECTION_PRIORITY);

/// Hardware priority of the active exception, or `None` in thread mode.
fn active_priority() -> Option<u8> {
    // Note(unsafe): Atomic reads of core registers.
    let vectactive = unsafe { (*SCB::PTR).icsr.read() } as usize & 0x1ff;
    match vectactive {
        0 => None,
        // NMI and HardFault: Fixed priorities above all configurable ones.
        1..4 => Some(0),
        4..16 => Some(unsafe { (*SCB::PTR).shpr[vectactive - 4].read() }),
        _ => Some(unsafe { (*NVIC::PTR).ipr[vectactive - 16].read() }),
    }
}

struct BasepriCriticalSection;
critical_section::set_impl!(BasepriCriticalSection);

unsafe impl critical_section::Impl for BasepriCriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        let restore = basepri::read();
        // With interrupts disabled altogether (`init`, panic handler), nothing can preempt us.
        if primask::read().is_active()
            && active_priority().is_some_and(|prio| prio < CEILING)
        {
            panic!(
                "Critical section above priority {CRITICAL_SECTION_PRIORITY}"
            );
        }
        basepri_max::write(CEILING);
        // Ensure no subsequent memory accesses are reordered to before masking.
        compiler_fence(Ordering::SeqCst);
        restore
    }

    unsafe fn release(restore: critical_section::RawRestoreState) {
        // Ensure no preceding memory accesses are reordered to after unmasking.
        compiler_fence(Ordering::SeqCst);
        unsafe { basepri::write(restore) };
    }
}
