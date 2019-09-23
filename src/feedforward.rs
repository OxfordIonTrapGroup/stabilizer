use stm32h7::stm32h743 as pac;
use serde::{Serialize, Deserialize};

const N_HARMONICS: usize = 5;

const N_LOOKUP: usize = 120;
const LINE_FREQ: u32 = 50; // Hz
const TMR_CLK_FREQ: u32 = 200000000; // Hz
const TMR_ARR_NOMINAL: u32 = TMR_CLK_FREQ / (LINE_FREQ* (N_LOOKUP as u32));

#[derive(Serialize)]
pub struct FFState {
    pub id: u32,
    pub n_coarse: u32,
    pub period_correction: i32,
    pub phase: i32
}

#[derive(Debug,Deserialize,Serialize)]
pub struct FFSettings {
    pub sin_amplitudes: [i16; N_HARMONICS],
    pub cos_amplitudes: [i16; N_HARMONICS],
    pub enable: bool
}


fn tim1_setup(tim1: &pac::TIM1) {
    tim1.arr.write(|w| unsafe { w.bits(TMR_ARR_NOMINAL) });
    tim1.dier.write(|w| w.uie().set_bit()); // Interrupt on overflow
    unsafe{
        tim1.ccmr1_input().modify(|_, w| w.cc1s().bits(1) ); // Capture/compare 1 channel is input from TI1
        tim1.ccmr1_input().modify(|_, w| w.ic1f().bits(0b11) ); // f_sampling = f_ck_int, require N=8 stables samples for a transition
    }
    tim1.ccer.modify(|_, w|
        w.cc1e().set_bit() // Enable capture
        );

    tim1.cr1.modify(|_, w|
        w.cen().set_bit());  // enable
}


pub fn setup(tim1: &pac::TIM1, gpioa: &pac::GPIOA)
{
    tim1_setup(tim1);

    // Enable PA8 = DI1 as AF input
    gpioa.moder.modify(|_, w| w.moder8().alternate()); // moder0 corresponds to pin 0 on GPIOA
    gpioa.afrh.modify(|_, w| w.afr8().af1()); // AF1 = TIM1_CH1
}


pub unsafe fn tim_interrupt(tim: &pac::TIM1, ff_state: &mut FFState) {
    static mut N: u32 = 0;
    static mut ID: u32 = 0;
    static mut PHASE_INT: i32 = 0;

    let sr = tim.sr.read();

    if sr.uif().bit_is_set() {
        tim.sr.write(|w| w.uif().clear_bit() );
        N += 1;
        if N == N_LOOKUP as u32 {
            N = 0;
        }
        // TODO: Update DAC
    }

    if sr.cc1if().bit_is_set() {
        tim.sr.write(|w| w.cc1if().clear_bit() );
        ID += 1;
        let n_fine = tim.ccr1.read().bits();
        let phase = n_fine + N*TMR_ARR_NOMINAL;
        let phase_error = phase as i32 - (N_LOOKUP as i32 + 1)*(TMR_ARR_NOMINAL as i32)/2;
        ff_state.id = ID;
        ff_state.n_coarse = N;
        ff_state.phase = phase_error;

        PHASE_INT += phase_error;
        let mut period_correction: i32 = PHASE_INT/10000 + phase_error/500;
        if period_correction > 1000 {period_correction=1000;}
        if period_correction < -1000 {period_correction=-1000;}
        ff_state.period_correction = period_correction;
        let period = TMR_ARR_NOMINAL as i32 + period_correction;
        tim.arr.write(|w| w.bits(period as u32) );
    }
}