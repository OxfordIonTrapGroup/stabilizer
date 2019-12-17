use serde::{Serialize, Deserialize};
use libm;

pub const N_HARMONICS: usize = 5;
pub const N_LOOKUP: usize = 300;

const LINE_FREQ: u32 = 50; // Hz
const TMR_CLK_FREQ: u32 = 200000000; // Hz
pub const TMR_ARR_NOMINAL: u32 = TMR_CLK_FREQ / (LINE_FREQ* (N_LOOKUP as u32));

#[derive(Serialize)]
pub struct State {
    pub id: u32,
    pub n_coarse: u32,
    pub period_correction: i32,
    pub phase: i32
}

#[derive(Debug,Deserialize,Serialize)]
pub struct Settings {
    pub sin_amplitudes: [f32; N_HARMONICS],
    pub cos_amplitudes: [f32; N_HARMONICS],
    pub offset: f32
}


pub struct Waveform {
    pub amplitude: [i16; N_LOOKUP],
}

impl Waveform {
    pub fn new() -> Waveform
    {
        Waveform{ amplitude: [0; N_LOOKUP] }
    }

    // Calculates the feedforward signal. This is called whenever the Fourier
    // coefficients are changed, so that in the interrupt loop we do the minimal
    // amount of work
    pub fn update(&mut self, settings: &Settings)
    {
        for n in 0..N_LOOKUP {
            self.amplitude[n] = Waveform::value(n, settings);
        }
    }

    // Calculates the feed-forward signal at point n out of nFeedforward
    fn value(n: usize, s: &Settings) -> i16
    {
        let mut sum: f32 = 0.;
        let phase: f32 = 2. * core::f32::consts::PI * (n as f32) / (N_LOOKUP as f32);
        for i in 0..N_HARMONICS {
            let harmonic_phase = phase*((i+1) as f32);
            sum += s.sin_amplitudes[i] * libm::sinf(harmonic_phase);
            sum += s.cos_amplitudes[i] * libm::cosf(harmonic_phase);
            sum += s.offset * 0.2_f32;
        }
        return (0x7fff as f32 * sum) as i16;
    }
}
