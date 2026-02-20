#[derive(Copy, Clone, Debug)]
pub struct BasicConfig<const N: usize> {
    //Zero order frequency
    pub zero_order_frequency: f32,
    pub amplitude: [f32; N],
    pub phase: [f32; N]
}

//Default is phase and amplitudes all 0 with Mains 50Hz frequency
impl<const N: usize> Default for BasicConfig<N>{
    fn default() -> Self {
        Self {
            zero_order_frequency: 50.0,
            amplitude: [0.0;N],
            phase: [0.0;N],
        }
    }
}
/// Represents the errors that can occur when attempting to configure the signal generator.
#[derive(Copy, Clone, Debug)]
pub enum Error {
    /// The provided amplitude is out-of-range.
    InvalidAmplitude,
    /// The provided frequency is out of range.
    InvalidFrequency,
}

impl<const N: usize> BasicConfig<N> {

    pub fn try_into_config(self, sample_period: f32, full_scale: f32)->Result<Config<N>, Error>{
        //const SYMMETRY: f32 = 0.5;
        //const NYQUIST: f32 = (1u32 << 31) as _;
        const PHASE_SCALE: f32 = (u32::MAX as f32) + 1.0; //added

        let ftw: [f32; N] = core::array::from_fn(|i| {
            self.zero_order_frequency
                * ((i + 1) as f32)
                * sample_period
                * PHASE_SCALE//NYQUIST //changed
        });
        // // Check for valid frequencies
        // for &f in &ftw {
        //     if f < 0.0 || 2.0 * f > NYQUIST {
        //         return Err(Error::InvalidFrequency);
        //     }
        // }

        let nyquist_hz = 1.0/(2.0*sample_period);
        for i in 0..N {
            let freq = self.zero_order_frequency * (i as f32 + 1.0);
            if freq > nyquist_hz {
                return Err(Error::InvalidFrequency);
            }
        }

        // let phase_increment: [[i32;2];N] = core::array::from_fn(|i| {
        //     let f = ftw[i];
        //     let val =
        //         (if SYMMETRY * NYQUIST > f {
        //             f / SYMMETRY
        //         } else {
        //             NYQUIST
        //         }) as i32;

        //         [val, val]
        // });
        let phase_increment: [i32; N] = core::array::from_fn(|i| {ftw[i] as i32});

        let amps_f32: [f32; N] = core::array::from_fn(|i: usize| {
            let scale = i16::MAX as f32 / full_scale;
            self.amplitude[i] * scale
        });

        for &amp in &amps_f32{
            if !(i16::MIN as f32..=i16::MAX as f32).contains(&amp){
                return Err(Error::InvalidAmplitude);
            }
        }

        let amps_i16: [i16; N] = core::array::from_fn(|i| {
            amps_f32[i] as i16
        });
        
        let phases_i32: [i32; N] = core::array::from_fn(|i: usize| {
           let p =  self.phase[i] * ((u32::MAX as f32) + 1.0); //changed to u32
           
           p as i32
            
        });

        Ok(Config::<N> {
                amplitude: amps_i16,
                phase_increment,
                phase_offset: phases_i32})
    }

}


#[derive(Copy, Clone, Debug)]
pub struct Config<const N: usize> {
    pub amplitude: [i16; N],
    pub phase_increment: [i32;N],
    pub phase_offset: [i32; N],
}

impl<const N: usize> Default for Config<N>{
    fn default() -> Self {
        Self {
            amplitude: [0;N],
            phase_increment: [0;N],
            phase_offset: [0;N],
        }
    }
}


#[derive(Debug)]
pub struct HarmonicGenerator<const N: usize>{
    phase_accumulator: [i32;N],
    config: Config<N>,
    global_phase_offset: i32,
}

impl<const N: usize> HarmonicGenerator<N> {

    pub fn new(config: Config<N>)-> Self {
        Self {
            config,
            phase_accumulator: [0;N],
            global_phase_offset: 0,
            
        }
    }

    /// Update waveform generation settings.
    pub fn update_waveform(&mut self, new_config: Config<N>) {
        self.config = new_config;
    }

    /// Clear the phase accumulator.
    pub fn clear_phase_accumulator(&mut self) {
        self.phase_accumulator = [0;N];
    }
    
    pub fn set_base_frequency(&mut self, freq: f32, sample_period: f32) {
        const PHASE_SCALE: f32 = (u32::MAX as f32) + 1.0;
        for i in 0..N {
            let harmonic = (i+1) as f32;
            let ftw = freq * harmonic * sample_period * PHASE_SCALE;
            self.config.phase_increment[i] = ftw as i32;
        }
    }

    // pub fn set_phase_offset_cycles(&mut self, phase_cycle: f32){
    //     const PHASE_SCALE: f32 = (u32::MAX as f32) + 1.0;
    //     let offset = (phase_cycle * PHASE_SCALE) as i32;
    //     for i in 0..N {
    //         self.config.phase_offset[i] = offset;
    //     }
    // }

    // pub fn add_global_offset(&mut self, phase_cycle: f32){
    //     const PHASE_SCALE: f32 = (u32::MAX as f32) + 1.0;
    //     let offset = (phase_cycle * PHASE_SCALE) as i32;
    //     for i in 0..N{
    //         self.config.phase_offset[i] = self.config.phase_offset[i].wrapping_add(offset);
    //     }
    // }   

    pub fn current_phase(&self) -> f32 {
        const PHASE_SCALE: f32 = (u32::MAX as f32) + 1.0;
        self.phase_accumulator[0] as f32 / PHASE_SCALE // interested in fundamental
    }

    pub fn set_global_phase_offset(&mut self, phase_cycle: f32){
        const PHASE_SCALE: f32 = (u32::MAX as f32)+ 1.0;
        self.global_phase_offset = (phase_cycle * PHASE_SCALE) as i32;
    }
    
    pub fn get_current_state(&self) -> (f32, f32){
        const PHASE_SCALE: f32 = (u32::MAX as f32)+ 1.0;
        (self.phase_accumulator[0] as f32 / PHASE_SCALE, self.config.phase_increment[0] as f32 / PHASE_SCALE)
    }

}

impl<const N: usize> core::iter::Iterator for HarmonicGenerator<N> {
    type Item = i16;
    fn next(&mut self) -> Option<i16>{

        let mut acc: i32 = 0;
        for i in 0..N
        {

            let phase = self.phase_accumulator[i].wrapping_add(self.config.phase_offset[i]).wrapping_add(self.global_phase_offset);
            //let sign = phase.is_negative();
            self.phase_accumulator[i] = self.phase_accumulator[i].wrapping_add(self.config.phase_increment[i]);
            let scale = idsp::cossin(phase).1 >> 16;
            acc = acc.wrapping_add((self.config.amplitude[i] as i32 * scale) >> 15);
            

        }
        let out = acc.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        Some(out)
    }
}