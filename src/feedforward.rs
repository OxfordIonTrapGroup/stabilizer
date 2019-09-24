use stm32h7::stm32h743 as pac;
use serde::{Serialize, Deserialize};
use core::f32::consts::PI;
use core::ptr;
extern crate libm;
use libm::F32Ext;


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

#[derive(Debug,Deserialize,Serialize,Clone)]
pub struct FFSettings {
    pub sin_amps: [f32; N_HARMONICS],
    pub cos_amps: [f32; N_HARMONICS],
}

impl FFSettings {
    pub fn new() -> FFSettings {
        FFSettings{ sin_amps: [0.; N_HARMONICS], cos_amps: [0.; N_HARMONICS] }
    }
}


fn tim2_setup(tim2: &pac::TIM2) {
    tim2.arr.write(|w| unsafe { w.bits(TMR_ARR_NOMINAL) });
    tim2.dier.write(|w| w.uie().set_bit()); // Interrupt on overflow
    unsafe{
        tim2.ccmr2_input().modify(|_, w| w.cc4s().bits(1) ); // Capture/compare 4 channel is input from TI4
        tim2.ccmr2_input().modify(|_, w| w.ic4f().bits(0b11) ); // f_sampling = f_ck_int, require N=8 stables samples for a transition
    }
    tim2.ccer.modify(|_, w|
        w.cc4e().set_bit() // Enable capture
        );

    tim2.cr1.modify(|_, w|
        w.cen().set_bit());  // enable
}


pub fn setup(tim2: &pac::TIM2, gpioa: &pac::GPIOA)
{
    tim2_setup(tim2);

    // Enable PA3 = DI0 as AF input
    gpioa.moder.modify(|_, w| w.moder3().alternate());
    gpioa.afrl.modify(|_, w| w.afr3().af1()); // AF1 = TIM2_CH4
}


pub unsafe fn tim_interrupt(
    tim: &pac::TIM2,
    ff_state: &mut FFState,
    waveform: &mut FFWaveform
) {
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
        let spi4 = &*pac::SPI4::ptr();
        let dac_val = waveform.amplitude[N as usize];
        let txdr = &spi4.txdr as *const _ as *mut u16;
        ptr::write_volatile(txdr, dac_val);    
    }

    if sr.cc4if().bit_is_set() {
        tim.sr.write(|w| w.cc4if().clear_bit() );
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


pub struct FFWaveform {
    pub amplitude: [u16; N_LOOKUP],
}

impl FFWaveform {
    pub fn new() -> FFWaveform
    {
        FFWaveform{ amplitude: [0; N_LOOKUP] }
    }

    // Calculates the feedforward signal. This is called whenever the Fourier 
    // coefficients are changed, so that in the interrupt loop we do the minimal 
    // amount of work
    pub fn update(&mut self, settings: &FFSettings)
    {
        for n in 0..N_LOOKUP {
            self.amplitude[n] = FFWaveform::value(n, settings);
        }
    }

    // Calculates the feed-forward signal at point n out of nFeedforward
    fn value(n: usize, s: &FFSettings) -> u16
    {
        let mut sum: f32 = 0.;
        let phase: f32 = 2.*PI * (n as f32) / (N_LOOKUP as f32);
        for i in 0..N_HARMONICS {
            sum += s.sin_amps[i] * unsafe{ (phase*(i as f32)).sin() };
            sum += s.cos_amps[i] * unsafe{ (phase*(i as f32)).cos() };
        }
        
        let dac_max: f32 = 32767.;
        let dac_val: i32 = (dac_max*sum) as i32;
        let dac_val: u16 = ((0x8000 as i32) + dac_val) as u16;
        return dac_val;  
    }
}