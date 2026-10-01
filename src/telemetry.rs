use crate::{
    convert::{AdcCode, DacCode, Gain},
    design_parameters,
};
use serde::Serialize;

/// The telemetry buffer is used for storing sample values during execution.
///
/// # Note
/// These values can be converted to SI units immediately before reporting to save processing time.
/// This allows for the DSP process to continually update the values without incurring significant
/// run-time overhead during conversion to SI units.
#[derive(Clone, Default)]
pub struct TelemetryBuffer {
    /// The latest input sample on ADC0/ADC1.
    pub adcs: [AdcCode; 2],
    /// The latest output code on DAC0/DAC1.
    pub dacs: [DacCode; 2],
    /// The latest digital input states during processing.
    pub digital_inputs: [bool; 2],
}

/// The telemetry structure is data that is ultimately reported as telemetry over MQTT.
///
/// # Note
/// This structure should be generated on-demand by the buffer when required to minimize conversion
/// overhead.
#[derive(Serialize)]
pub struct Telemetry {
    /// Most recent input voltage measurement.
    pub adcs: [f32; 2],

    /// Most recent output voltage.
    pub dacs: [f32; 2],

    /// Most recent digital input assertion state.
    pub digital_inputs: [bool; 2],

    /// The CPU temperature in degrees Celsius.
    pub cpu_temp: f32,

    /// Pounder measurements, if applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pounder: Option<PounderTelemetry>,
}

/// Pounder-specific telemetry.
#[derive(Serialize, Clone, Copy)]
pub struct PounderTelemetry {
    /// The Pounder temperature in degrees Celsius.
    pub temperature: f32,

    /// The RF power detected at IN0/IN1 in dBm.
    pub input_power: [f32; 2],
}

impl TelemetryBuffer {
    /// Convert the telemetry buffer to finalized, SI-unit telemetry for reporting.
    ///
    /// # Args
    /// * `afe0` - The current AFE configuration for channel 0.
    /// * `afe1` - The current AFE configuration for channel 1.
    /// * `cpu_temp` - The current CPU temperature.
    ///
    /// # Returns
    /// The finalized telemetry structure that can be serialized and reported.
    pub fn finalize(self, afe0: Gain, afe1: Gain, cpu_temp: f32) -> Telemetry {
        let in0_volts = f32::from(self.adcs[0]) / afe0.gain();
        let in1_volts = f32::from(self.adcs[1]) / afe1.gain();

        Telemetry {
            cpu_temp,
            adcs: [in0_volts, in1_volts],
            dacs: [self.dacs[0].into(), self.dacs[1].into()],
            digital_inputs: self.digital_inputs,
            pounder: None,
        }
    }
}

/// Accumulator for the execution time of a processing routine in CPU cycles.
///
/// Take the timestamps from the DWT cycle counter (`cortex_m::peripheral::DWT::cycle_count()`).
/// It is enabled during hardware setup.
#[derive(Copy, Clone, Debug, Default)]
pub struct ProcessTime {
    max: u32,
    sum: u64,
    count: u32,
}

impl ProcessTime {
    /// Record one execution that took the given number of CPU cycles.
    #[inline(always)]
    pub fn update(&mut self, cycles: u32) {
        self.max = self.max.max(cycles);
        self.sum = self.sum.wrapping_add(cycles as u64);
        self.count = self.count.wrapping_add(1);
    }

    /// Convert the statistics to seconds for reporting.
    pub fn finalize(self) -> Timing {
        Timing::from_cycles(self.sum, self.count, self.max)
    }
}

/// The duration of one CPU cycle in seconds.
pub const CPU_CYCLE: f32 = 1.0 / design_parameters::SYSCLK.to_Hz() as f32;

/// Execution time statistics accumulated since the previous report.
#[derive(Copy, Clone, Debug, Default, Serialize)]
pub struct Timing {
    /// Mean execution time in seconds.
    pub mean: f32,
    /// Maximum execution time in seconds.
    pub max: f32,
}

impl Timing {
    /// Convert the sum and the maximum of `count` measurements in CPU cycles.
    pub fn from_cycles(sum: u64, count: u32, max: u32) -> Self {
        Self {
            mean: if count == 0 {
                0.0
            } else {
                sum as f32 / count as f32 * CPU_CYCLE
            },
            max: max as f32 * CPU_CYCLE,
        }
    }
}
