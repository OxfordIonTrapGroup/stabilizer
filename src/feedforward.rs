// use super::pac;
use serde::{Serialize, Deserialize};


// include!(concat!(env!("OUT_DIR"), "/sin_lookup.rs"));

pub const N_HARMONICS: usize = 5;
pub const N_LOOKUP: usize = 120;

const LINE_FREQ: u32 = 50; // Hz
const TMR_CLK_FREQ: u32 = 200000000; // Hz
pub const TMR_ARR_NOMINAL: u32 = TMR_CLK_FREQ / (LINE_FREQ* (N_LOOKUP as u32));

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
    pub fn update_waveform(mut self, settings: FFSettings)
    {
        for n in 0..N_LOOKUP {
            self.amplitude[n] = FFWaveform::value(n, &settings);
        }
    }

    // Calculates the feed-forward signal at point n out of nFeedforward
    fn value(n: usize, s: &FFSettings) -> u16
    {
        let mut sum: i32 = 0;
        for i in 0..N_HARMONICS {
            // sum += (s.sin_amplitudes[i] as i32)* SIN_TABLE[ (n*(i+1))%120 ];
            // sum += (s.cos_amplitudes[i] as i32)* SIN_TABLE[ (30 + n*(i+1))%120 ];
        }
        
        let dac_val: u16 = 0x8000 + ( (sum>>15) as u16);
        return dac_val;  
    }
}
