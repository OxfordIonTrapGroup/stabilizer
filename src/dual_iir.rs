//! Settings and processing shared by the `dual-iir`, `dual-iir-ll` and `current_sense`
//! applications.
//!
//! The low-latency applications (`dual-iir-ll`, `current_sense`) process each sample
//! individually with [`Active`] and report [`Telemetry`].

use core::sync::atomic::{AtomicBool, Ordering};

use idsp::iir::{self, pid::Units};
use miniconf::Tree;
use num_traits::clamp;
use serde::{Deserialize, Serialize};
use signal_generator::Source;

use crate::{
    convert::{DacCode, Gain},
    pounder_config::PounderConfig,
    telemetry::{self, CPU_CYCLE, PounderTelemetry, Timing},
};

/// The number of cascaded IIR biquads per channel. Select 1 or 2!
pub const IIR_CASCADE_LENGTH: usize = 1;

/// The filter built from a [`BiquadRepr`].
pub type Biquad = iir::BiquadClamp<f32, f32>;

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename = "BiquadReprTree"))]
pub struct BiquadRepr {
    /// Biquad parameters
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.repr)]
    _typ: (),
    repr: iir::repr::BiquadRepr<f32, f32>,
}

impl Default for BiquadRepr {
    fn default() -> Self {
        let mut i = iir::BiquadClamp::from(iir::Biquad::IDENTITY);
        i.min = -i16::MAX as _;
        i.max = i16::MAX as _;
        Self {
            _typ: (),
            repr: iir::repr::BiquadRepr::Raw(i),
        }
    }
}

impl BiquadRepr {
    /// Build the filter for the given time and input/output units.
    pub fn build(&self, units: &Units<f32>) -> Biquad {
        self.repr.build(units)
    }
}

#[derive(
    Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Default,
)]
pub enum Run {
    #[default]
    /// Run
    Run,
    /// Hold
    Hold,
    /// Hold controlled by corresponding digital input
    External,
}

impl Run {
    /// Whether the filter runs given the state of the digital input.
    #[inline]
    pub fn run(&self, di: bool) -> bool {
        match self {
            Self::Run => true,
            Self::Hold => false,
            Self::External => di,
        }
    }
}

/// A ADC-DAC channel
#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Channel {
    /// Analog Front End (AFE) gain.
    #[tree(with=miniconf::leaf)]
    pub gain: Gain,
    /// Biquad
    pub biquad: [BiquadRepr; IIR_CASCADE_LENGTH],
    /// Run/Hold behavior
    #[tree(with=miniconf::leaf)]
    pub run: Run,
    /// Signal generator configuration to add to the DAC0/DAC1 outputs
    pub source: signal_generator::Config,
}

impl Channel {
    /// Build the filters for the given sample period and input/output units.
    pub fn build_filter(
        &self,
        units: &Units<f32>,
    ) -> (Run, [Biquad; IIR_CASCADE_LENGTH]) {
        (
            self.run,
            self.biquad.each_ref().map(|biquad| biquad.build(units)),
        )
    }

    /// Build the signal source for the sample period `units.t`.
    ///
    /// Errors are logged.
    pub fn build_source(
        &self,
        units: &Units<f32>,
    ) -> Result<Source, signal_generator::Error> {
        self.source
            .build(units.t, DacCode::FULL_SCALE.recip())
            .inspect_err(|err| {
                log::error!("Failed to update source: {:?}", err)
            })
    }
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct DualIir {
    /// Channel configuration
    pub ch: [Channel; 2],
    /// Trigger both signal sources
    #[tree(with=miniconf::leaf)]
    pub trigger: bool,
    /// Telemetry output period in seconds.
    #[tree(with=miniconf::leaf)]
    pub telemetry_period: f32,
    /// Target IP and port for UDP streaming.
    ///
    /// Can be multicast.
    #[tree(with=miniconf::leaf)]
    pub stream: stream::Target,
    /// Pounder DDS clock, DDS channel, and attenuator configuration.
    ///
    /// Only present if a Pounder is detected.
    pub pounder: Option<PounderConfig>,
}

impl Default for DualIir {
    fn default() -> Self {
        Self {
            telemetry_period: 10.0,
            trigger: false,
            stream: Default::default(),
            ch: Default::default(),
            pounder: None,
        }
    }
}

/// The state of a [`Biquad`] with the input-independent part of its summing junction
/// precomputed.
///
/// A [`Biquad`] evaluates
///
/// `y0 = clamp(b0*x0 + b1*x1 + b2*x2 + a1*y1 + a2*y2 + u, min, max)`
///
/// when the input `x0` arrives. Everything but `b0*x0` is already known one sample earlier.
/// This splits the evaluation in two steps to minimize the delay from input to output:
///
/// * [`output()`](Self::output) computes `y0` from `x0` with a single multiplication, an
///   addition and the clamp.
/// * [`update()`](Self::update) then shifts `x0` and `y0` into the delay lines and precomputes
///   the remaining terms for the next sample. It can run after the output has been used.
///
/// The filter structure (direct form 1 with the clamped output fed back) is that of
/// [`Biquad`]. The results only differ by the rounding of the floating point additions due to
/// their different order.
#[derive(Clone, Debug, Default)]
pub struct Stage {
    /// The input and output delay lines, as for [`Biquad`].
    state: iir::DirectForm1<f32>,
    /// `b1*x1 + b2*x2 + a1*y1 + a2*y2 + u`
    partial: f32,
}

impl Stage {
    /// The output for the input `x0`.
    #[inline(always)]
    pub fn output(&self, biquad: &Biquad, x0: f32) -> f32 {
        clamp(
            biquad.coeff.ba[0] * x0 + self.partial,
            biquad.min,
            biquad.max,
        )
    }

    /// The output while holding: the previous output.
    #[inline(always)]
    pub fn hold(&self) -> f32 {
        self.state.y[0][0]
    }

    /// Shift the input `x0` and the output `y0` into the delay lines and precompute the
    /// input-independent terms for the next sample.
    #[inline(always)]
    pub fn update(&mut self, biquad: &Biquad, x0: f32, y0: f32) {
        self.state.x = [x0, self.state.x[0]];
        self.state.y[0] = [y0, self.state.y[0][0]];
        self.prepare(biquad);
    }

    /// Precompute the input-independent terms.
    ///
    /// To be called whenever the filter changes.
    #[inline(always)]
    pub fn prepare(&mut self, biquad: &Biquad) {
        let ba = &biquad.coeff.ba;
        let ([x1, x2], [y1, y2]) = (self.state.x, self.state.y[0]);
        // Two short independent sums instead of one long accumulation chain.
        self.partial =
            (ba[1] * x1 + ba[2] * x2) + ((ba[3] * y1 + ba[4] * y2) + biquad.u);
    }
}

/// The run-time state of one channel of the low-latency applications.
#[derive(Clone, Debug)]
pub struct Active {
    run: Run,
    biquad: [Biquad; IIR_CASCADE_LENGTH],
    stage: [Stage; IIR_CASCADE_LENGTH],
    /// The signal source. `None` if its output is constant.
    source: Option<Source>,
    /// The signal source contribution to the next output sample in DAC LSB.
    offset: i16,
}

/// The input and the outputs of the filter stages for one sample.
///
/// They are kept from [`Active::process()`] to [`Active::update()`].
pub struct Sample {
    x: f32,
    y: [f32; IIR_CASCADE_LENGTH],
}

impl Active {
    /// Build the state of a channel for the given sample period and input/output units.
    ///
    /// # Panics
    /// If the signal source cannot be built.
    pub fn new(ch: &Channel, units: &Units<f32>) -> Self {
        let (run, biquad) = ch.build_filter(units);
        let mut active = Self {
            source: None,
            offset: 0,
            stage: Default::default(),
            run,
            biquad,
        };
        active.set_filter(active.run, active.biquad.clone());
        active.set_source(ch.build_source(units).unwrap());
        active
    }

    /// Replace the filter. The transfer is bumpless.
    pub fn set_filter(
        &mut self,
        run: Run,
        biquad: [Biquad; IIR_CASCADE_LENGTH],
    ) {
        self.run = run;
        self.biquad = biquad;
        for (stage, biquad) in self.stage.iter_mut().zip(&self.biquad) {
            stage.prepare(biquad);
        }
    }

    /// Replace the signal source.
    ///
    /// Sources with constant output are not iterated.
    pub fn set_source(&mut self, source: Source) {
        if let Some(offset) = source.constant() {
            self.offset = (offset >> 16) as _;
            self.source = None;
        } else {
            self.source = Some(source);
        }
    }

    /// Filter one input sample and add the signal source contribution.
    ///
    /// This is the latency critical part. It only uses what [`update()`](Self::update)
    /// precomputed after the previous sample.
    ///
    /// # Args
    /// * `di` - The digital input state for run/hold control.
    /// * `x` - The input sample in ADC LSB, i.e. `f32::from(adc as i16)` for the ADC code
    ///   `adc`, plus any offset to apply.
    ///
    /// # Returns
    /// The DAC code for the output sample and the data for [`update()`](Self::update).
    #[inline(always)]
    pub fn process(&self, di: &AtomicBool, x: f32) -> (u16, Sample) {
        let run = match self.run {
            Run::Run => true,
            Run::Hold => false,
            Run::External => di.load(Ordering::Relaxed),
        };
        let mut y = [0.0; IIR_CASCADE_LENGTH];
        let mut v = x;
        for ((biquad, stage), y) in
            self.biquad.iter().zip(&self.stage).zip(&mut y)
        {
            v = if run {
                stage.output(biquad, v)
            } else {
                stage.hold()
            };
            *y = v;
        }

        // Note(unsafe): The filter limits must ensure that the value is in range.
        // The truncation introduces 1/2 LSB distortion.
        let v: i16 = unsafe { v.to_int_unchecked() };
        (
            DacCode::from(v.saturating_add(self.offset)).0,
            Sample { x, y },
        )
    }

    /// Update the filter state with the sample and prepare for the next one.
    #[inline(always)]
    pub fn update(&mut self, sample: Sample) {
        let mut x = sample.x;
        for ((biquad, stage), y) in
            self.biquad.iter().zip(&mut self.stage).zip(sample.y)
        {
            stage.update(biquad, x, y);
            x = y;
        }
    }

    /// Advance the signal source for the next sample.
    #[inline(always)]
    pub fn update_source(&mut self) {
        if let Some(source) = &mut self.source {
            self.offset = next(source);
        }
    }
}

/// The next signal source sample in DAC LSB.
///
/// This is kept out of line and next to the sample processing routine in ITCM: there is a
/// single copy of the generator code for both channels and it only runs for active sources.
#[inline(never)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".itcm.process.source"))]
fn next(source: &mut Source) -> i16 {
    (source.next().unwrap() >> 16) as _
}

/// Execution time statistics of the sample processing routine in CPU cycles.
#[derive(Clone, Copy, Default)]
pub struct Times {
    /// Maximum time until the last DAC code has been written
    output_max: u32,
    /// Maximum duration of the routine
    process_max: u32,
    /// Sum of the durations of the routine
    process_sum: u64,
    /// Number of batches at whose end the next sample was already pending
    late: u32,
}

impl Times {
    /// Record one execution of the routine.
    ///
    /// # Args
    /// * `output` - Cycles until the last DAC code has been written.
    /// * `process` - Cycles until the end of the routine.
    /// * `late` - Whether the next sample was already pending at the end (only checked at the
    ///   end of each batch).
    #[inline(always)]
    pub fn update(&mut self, output: u32, process: u32, late: bool) {
        self.output_max = self.output_max.max(output);
        self.process_max = self.process_max.max(process);
        self.process_sum = self.process_sum.wrapping_add(process as _);
        self.late = self.late.wrapping_add(late as _);
    }
}

/// The telemetry reported by the low-latency applications.
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
    /// Execution time of the sample processing routine. Excludes interrupt entry and exit.
    pub process: Timing,
    /// Maximum time in seconds from the start of the sample processing routine until the
    /// last DAC code has been written.
    pub output: f32,
    /// Number of batches (of the stream, eight samples) since the previous report at whose end
    /// the next sample had already arrived: processing had fallen behind. Such samples are
    /// processed with increased latency, not lost. Only checked once per batch.
    pub late: u32,
    /// Pounder measurements, if applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pounder: Option<PounderTelemetry>,
}

impl Telemetry {
    /// Add the execution time statistics for `count` samples to the common telemetry.
    pub fn new(
        telemetry: telemetry::Telemetry,
        times: Times,
        count: u32,
    ) -> Self {
        Self {
            adcs: telemetry.adcs,
            dacs: telemetry.dacs,
            digital_inputs: telemetry.digital_inputs,
            cpu_temp: telemetry.cpu_temp,
            process: Timing::from_cycles(
                times.process_sum,
                count,
                times.process_max,
            ),
            output: times.output_max as f32 * CPU_CYCLE,
            late: times.late,
            pounder: telemetry.pounder,
        }
    }
}

#[cfg(test)]
mod test {
    extern crate std;

    use super::*;
    use dsp_process::SplitProcess;

    struct Rng(u32);

    impl Rng {
        /// Uniform in [-1, 1)
        fn next(&mut self) -> f32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            (self.0 as i32) as f32 / (1u32 << 31) as f32
        }
    }

    fn biquad(ba: [f32; 5], u: f32, limit: f32) -> Biquad {
        let mut biquad = Biquad::from(iir::Biquad::from(ba));
        biquad.u = u;
        biquad.min = -limit;
        biquad.max = limit;
        biquad
    }

    /// The [`Biquad`] in double precision.
    fn exact(filter: &Biquad, state: &mut [f64; 4], x0: f64) -> f64 {
        let ba = filter.coeff.ba.map(f64::from);
        let [x1, x2, y1, y2] = *state;
        let y0 = (ba[0] * x0
            + ba[1] * x1
            + ba[2] * x2
            + ba[3] * y1
            + ba[4] * y2
            + filter.u as f64)
            .clamp(filter.min as _, filter.max as _);
        *state = [x0, x1, y0, y1];
        y0
    }

    /// Run the idsp implementation and the split implementation side by side and return
    /// their maximum deviations from the double precision result.
    fn errors(filters: &[Biquad], samples: usize, hold: bool) -> [f32; 2] {
        let mut rng = Rng(0x1234_5678);
        let mut reference = iir::DirectForm1::<f32>::default();
        let mut stage = Stage::default();
        let mut double = [0f64; 4];
        let mut max = 0f32;
        let mut err = [0f32; 2];
        for (i, filter) in filters.iter().cycle().take(8).enumerate() {
            // Bumpless filter change
            stage.prepare(filter);
            for j in 0..samples {
                let x = (rng.next() * 3e4).round();
                let run = !hold || (i + j / 50) % 3 != 0;
                let (y_ref, y, y_exact) = if run {
                    (
                        filter.process(&mut reference, x),
                        stage.output(filter, x),
                        exact(filter, &mut double, x as _),
                    )
                } else {
                    let y = double[2];
                    double = [x as _, double[0], y, y];
                    (
                        iir::Biquad::<f32>::HOLD.process(&mut reference, x),
                        stage.hold(),
                        y,
                    )
                };
                stage.update(filter, x, y);
                assert!(y >= filter.min && y <= filter.max);
                err[0] = err[0].max((y_ref as f64 - y_exact).abs() as f32);
                err[1] = err[1].max((y as f64 - y_exact).abs() as f32);
                max = max.max(y.abs());
            }
        }
        // The test exercises a significant part of the output range.
        assert!(max > 1e3);
        err
    }

    #[test]
    fn proportional_is_exact() {
        let p = biquad([-0.75, 0., 0., 0., 0.], 100., 3e4);
        assert_eq!(errors(&[p], 1000, true), [0.; 2]);
    }

    #[test]
    fn accuracy() {
        // PI: kp = 0.5, ki*T = 1e-3, clamped
        let pi = biquad([0.5005, -0.4995, 0., 1., 0.], 0., 2e4);
        // PI with high proportional gain and slow integrator
        let pi_hi = biquad([20.000_01, -19.999_99, 0., 1., 0.], 0., 3e4);
        // Lowpass with complex poles
        let lp = biquad([0.02, 0.04, 0.02, 1.56, -0.64], -300., 3e4);
        // Lead/lag with offset, tight clamp
        let ll = biquad([3., -2.7, 0., 0.9, 0.], 1000., 5e3);
        for (name, filters, samples, hold) in [
            ("pi", &[pi.clone()][..], 100_000, false),
            ("pi_hi", &[pi_hi.clone()][..], 100_000, false),
            ("lp", &[lp.clone()][..], 100_000, false),
            ("ll", &[ll.clone()][..], 100_000, false),
            ("mixed", &[pi, lp, ll, pi_hi][..], 2000, true),
        ] {
            let [idsp, split] = errors(filters, samples, hold);
            std::println!("{name}: idsp {idsp:e}, split {split:e} LSB");
            // Both deviate from the exact result by accumulated rounding errors. The split
            // evaluation must be about as accurate as the idsp one.
            assert!(split <= 3. * idsp + 1e-2, "{name}: {split} vs {idsp}");
        }
    }
}
