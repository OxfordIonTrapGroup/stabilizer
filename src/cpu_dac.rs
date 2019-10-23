use core::ops::{Add, Mul};
use serde::{Serialize, Deserialize};
// use serde::de::{self, Deserializer, Unexpected};

use core::{f32, u16};

// pub type CPU_DACState = [u16; 1];

#[derive(Copy,Clone,Deserialize,Serialize)]
pub struct CPU_DAC {
    pub out: u16,
    // #[serde(deserialize_with = "bool_from_int")]
    pub en: bool,
}

#[allow(unused)]
impl CPU_DAC {
    pub fn set_scale_out(&mut self, out: f32) -> Result<(), &str> {
        // out 0 -> 1
        self.out = (out * (0xfff as f32)) as u16;
        Ok(())
    }

    pub fn get_scale_out(&self) -> Result<f32, &str> {
        Ok( (self.out as f32) / (0xfff as f32))
    }

    pub fn set_en(&mut self, enable: bool) -> Result<(), &str> {
        self.en = enable;
        Ok(())
    }
    pub fn get_en(&mut self, enable: bool) -> Result<bool, &str> {
        Ok(self.en as bool)
    }

}
