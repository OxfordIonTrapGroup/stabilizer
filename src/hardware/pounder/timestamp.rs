///! ADC sample timestamper using external Pounder reference clock.
///!
///! # Design
///!
///! The pounder timestamper utilizes the pounder SYNC_CLK output as a fast external reference clock
///! for recording a timestamp for each of the ADC samples.
///!
///! To accomplish this, a timer peripheral is configured to be driven by an external clock input.
///! Due to the limitations of clock frequencies allowed by the timer peripheral, the SYNC_CLK input
///! is divided by 4. This clock then clocks the timer peripheral in a free-running mode with an ARR
///! (max count register value) configured to overflow once per ADC sample batch.
///!
///! Once the timer is configured, an input capture is configured to record the timer count
///! register. The input capture is configured to utilize an internal trigger for the input capture.
///! The internal trigger is selected such that when a sample is generated on ADC0, the input
///! capture is simultaneously triggered. This results in the input capture triggering identically
///! to when the ADC samples the input.
///!
///! Once the input capture is properly configured, a DMA transfer is configured to collect all of
///! timestamps. The DMA transfer collects 1 timestamp for each ADC sample collected. In order to
///! avoid potentially losing a timestamp for a sample, the DMA transfer operates in double-buffer
///! mode. As soon as the DMA transfer completes, the hardware automatically swaps over to a second
///! buffer to continue capturing. This alleviates timing sensitivities of the DMA transfer
///! schedule.
use crate::hardware::{design_parameters, timers};
use core::convert::TryFrom;
use stm32h7xx_hal as hal;

/// Software unit to timestamp stabilizer ADC samples using an external pounder reference clock.
pub struct Timestamper {
    timer: timers::PounderTimestampTimer,
    capture_channel: timers::tim8::Channel1InputCapture,
}

impl Timestamper {
    /// Construct the pounder sample timestamper.
    ///
    /// # Note
    /// The DMA is immediately configured after instantiation. It will not collect any samples
    /// until the sample timer begins to cause input capture triggers.
    ///
    /// # Args
    /// * `timestamp_timer` - The timer peripheral used for capturing timestamps from.
    /// * `stream` - The DMA stream to use for collecting timestamps.
    /// * `capture_channel` - The input capture channel for collecting timestamps.
    /// * `sampling_timer` - The stabilizer ADC sampling timer.
    /// * `_clock_input` - The input pin for the external clock from Pounder.
    ///
    /// # Returns
    /// The new pounder timestamper in an operational state.
    pub fn new(
        mut timestamp_timer: timers::PounderTimestampTimer,
        capture_channel: timers::tim8::Channel1,
        sampling_timer: &mut timers::SamplingTimer,
        _clock_input: hal::gpio::gpioa::PA0<
            hal::gpio::Alternate<hal::gpio::AF3>,
        >,
    ) -> Self {
        // The sampling timer should generate a trigger output when CH1 comparison occurs.
        sampling_timer.generate_trigger(timers::TriggerGenerator::ComparePulse);

        // The timestamp timer trigger input should use TIM2 (SamplingTimer)'s trigger, which is
        // mapped to ITR1.
        timestamp_timer.set_trigger_source(timers::TriggerSource::Trigger1);

        // The capture channel should capture whenever the trigger input occurs.
        let mut input_capture = capture_channel
            .into_input_capture(timers::tim8::CaptureSource1::TRC);

        // Capture at the batch period.
        input_capture.configure_prescaler(
            timers::Prescaler::try_from(
                design_parameters::SAMPLE_BUFFER_SIZE_LOG2,
            )
            .unwrap(),
        );

        Self {
            timer: timestamp_timer,
            capture_channel: input_capture,
        }
    }

    /// Start collecting timestamps.
    pub fn start(&mut self) {
        self.capture_channel.enable();
    }

    /// Update the period of the underlying timestamp timer.
    pub fn update_period(&mut self, period: u16) {
        self.timer.set_period_ticks(period);
    }

    /// Obtain a timestamp.
    ///
    /// # Returns
    /// A `Result` potentially indicating capture overflow and containing a `Option` of a captured
    /// timestamp.
    pub fn latest_timestamp(&mut self) -> Result<Option<u16>, Option<u16>> {
        self.capture_channel.latest_capture()
    }
}
