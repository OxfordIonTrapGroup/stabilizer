//Ignore warnings
#![deny(warnings)]
//Tells rust to not link standard library so annot use heap unless we add one!
#![no_std]
//Do not need main() function in this
#![no_main]

//From core library
//MaybeUninit<T> lets you work with uninitialized memory safely
use core::mem::MaybeUninit;
//Memory ordering and synchronization at CPU level - ordering defiens how memory operations can be reordered
//Fence - inserts memory fence that prevents compiler and CPU from reordering memory operations across it
use core::sync::atomic::{fence, Ordering};
use core::usize;

use serde::{Deserialize, Serialize};
//use array_init::array_init;

// External crate - fugit - provides strongly typed time a duration units e.g. 1.secs()
use fugit::ExtU64;
//External crate mutex_trait - defines generic mutex abstraction - often used when mutex depends on platform
use mutex_trait::prelude::*;

//The iir algorithm that we need - inbuilt package
use idsp::iir;

//use stabilizer::hardware::signal_generator::{BasicConfig, Signal};
use stabilizer::app_utils::harmonic_oscillators::{BasicConfig};
//Features we need from stabilizer
use stabilizer::{
    app_utils::harmonic_oscillators::{HarmonicGenerator},
    hardware::{
        self,
        
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        hal,
        current_sense_dac::CurrentSenseDac,
        pounder::{ClockConfig, PounderConfig},
        setup::PounderDevices as Pounder,
        //signal_generator::{self, SignalGenerator},
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, SerialTerminal, SystemTimer, Systick,
        UsbDevice, AFE0, AFE1,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Tree,
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

//----------------------------------------------------------------------------------------------
//CONSTANTS

//Max magnitude of 16 bit signed signal
const SCALE: f32 = i16::MAX as _;

//Number of cascade IIR biquads used (per channel for dual) 1 (second order filter) or 2 (4th order) stored in iir::Biquad<f32>
const IIR_CASCADE_LENGTH: usize = 1;

//Number of sampels in each batch process - set to 8 - used to DMA buffering and real time efficiency
const BATCH_SIZE: usize = 8;

//Logarithm of number of 100MHz timer ticks between each sample - set to 7 ,eaning value of 128 (2^7) - 1.28us per sample or 781.25kGHz here
const SAMPLE_TICKS_LOG2: u8 = 8;

//This just converts to seconds
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

//This is the period
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

//Frequency of mains
const MAINS_FREQUENCY: f32 = 50.0;

//MAX HARMONCIS
const MAX_HARMONICS: usize = 5;

// NOTE: Use log 2 as hardware timers work best with powers of 2 and allows fast bit shifts instead of division
// NOTE: Use batching as DMA transfers in chunks not one by one - processig batch reduces interrupt overhead and keeps the pipelines for ADC and DAC full - 
//       else too many interruptions means CPU cannot keep up and ADC gets overrun

//----------------------------------------------------------------------------------------------

//Create a Struct for harmonic wave parameters that we can use - unique to this app so keep it here
#[derive(Copy, Clone, Debug, Serialize, Deserialize, Tree)]
pub struct HarmonicWaveParameters {
    /// Amplitude in controller units (use f32 for fractional amplitude)
    pub amp: f32,
    /// Phase in degrees [0..360)
    pub phase: f32,
}
impl Default for HarmonicWaveParameters {
    fn default() -> Self {
        Self {
            amp: 0.0,
            phase: 0.0,
        }
    }
}


#[derive(Clone, Copy, Debug, Tree)]
pub struct Settings{
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    #[tree]
    afe: [Gain;2],

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    /// * `<m>` specifies which cascade to configure. `<m>` := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// See [iir::Biquad]
    #[tree(depth(2))]
    iir_ch: [[iir::Biquad<f32>; IIR_CASCADE_LENGTH];2],

    /// Specified true if DI1 should be used as a "hold" input.
    ///
    /// # Path
    /// `allow_hold`
    ///
    /// # Value
    /// "true" or "false"
    allow_hold: bool,

    /// Specified true if "hold" should be forced regardless of DI1 state and hold allowance.
    ///
    /// # Path
    /// `force_hold`
    ///
    /// # Value
    /// "true" or "false"
    force_hold: bool,

    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any non-zero value less than 65536.
    telemetry_period: u16,

    /// Specifies the target for data livestreaming.
    ///
    /// # Path
    /// `stream_target`
    ///
    /// # Value
    /// See [StreamTarget#miniconf]
    stream_target: StreamTarget,

    /// Specifies the config for signal generators to add on to DAC0/DAC1 outputs.
    ///
    /// # Path
    /// `harmonic_wave_parameters/<n>`
    /// 
    /// * `<n>` specifies which harmonic order to configure. `<n>` := [0,1,2,3,4]
    ///
    ///
    /// # Value
    /// See [HarmonicWaveParameters]
    #[tree(depth(2))]
    harmonic_wave_parameters: [[HarmonicWaveParameters;MAX_HARMONICS];2],

    //TO DO - MAY NOT NEED THIS
    /// Specifies the config for pounder DDS clock configuration, DDS channels & attenuations
    ///
    /// # Path
    /// `pounder`
    ///
    /// # Value
    /// See [PounderConfig#miniconf]
    /// TODO: this was #[miniconf(defer)] -- is this right? Also, miniconf::Option vs Option?
    #[tree]
    pounder: Option<PounderConfig>,

    //Add in the V_offset
    v_offset: f32,

}

impl Default for Settings{
    fn default() -> Self {
        //This is a unity gain filter
        let mut i = iir::Biquad::IDENTITY;
        i.set_min(-SCALE); //Set the lower output limit
        i.set_max(SCALE); //Set the upper output limit

        Self{
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[i; IIR_CASCADE_LENGTH]; 2],
            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream_target: StreamTarget::default(),
            //TO DO - CHOOSE BETTER VALUES FOR DEFAULT!
            harmonic_wave_parameters:[[HarmonicWaveParameters::default(); MAX_HARMONICS], [HarmonicWaveParameters::default(); MAX_HARMONICS]],


            //TO DO - MAY NOT NEED THIS TBH
            pounder: None.into(),
            v_offset: 0.0,
        }
    }
}

//Going to follow logic for older version which uses RTIC
#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {


    use super::*;

    //Define the fact we are using monotonic time - only goes forwards
    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Settings, Telemetry, 3>,
        settings: Settings, //All our settings are shared
        telemetry: TelemetryBuffer,
        harmonic_generators: [HarmonicGenerator<MAX_HARMONICS>;2],
        pounder: Option<Pounder>,
        
    }



    #[local]
    struct Local {
        usb_terminal: SerialTerminal,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        iir_state: [[[f32; 4]; IIR_CASCADE_LENGTH]; 2],
        dds_clock_state: Option<ClockConfig>,
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        current_sense_dac: Option<CurrentSenseDac>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {

        //Define the clock
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        //Configure the MCU
        let (stabilizer, pounder, current_sense_dac) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let device_settings = stabilizer.usb_serial.settings();
        let mut application_settings = Settings::default(); //Use default application settings
        if pounder.is_some() {
            application_settings
                .pounder
                .replace(PounderConfig::default());
        }

        //Define the dds clock state
        let dds_clock_state = pounder.as_ref().map(|_| ClockConfig::default());

        //Define the network settings
        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &device_settings.broker,
            &device_settings.id,
            stabilizer.metadata,
            application_settings,
        );

        let generator = network.configure_streaming(StreamFormat::AdcDacData);

        let harmonic_generators_arr = core::array::from_fn(|channel| {
            let basic_cfg = BasicConfig::<MAX_HARMONICS> {
                        zero_order_frequency: MAINS_FREQUENCY,
                        amplitude: core::array::from_fn(|i| {
                                    application_settings.harmonic_wave_parameters[channel][i].amp
                                }),
                        phase: core::array::from_fn(|i| {application_settings.harmonic_wave_parameters[channel][i].phase/360.0}),
            };

            let cfg = basic_cfg.try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE).unwrap();

            HarmonicGenerator::new(cfg)
        });


        let shared = Shared {
            usb: stabilizer.usb,
            network,
            settings: application_settings,
            telemetry: TelemetryBuffer::default(),
            harmonic_generators: harmonic_generators_arr,
            pounder,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            iir_state: [[[0.; 4]; IIR_CASCADE_LENGTH]; 2],
            dds_clock_state,
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            current_sense_dac,
        };

        // Explicitly write to DAC on set up
        if let Some(dac) = local.current_sense_dac.as_mut() {
            dac.write_voltage(application_settings.v_offset);
        }

        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();


        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
        start::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(stabilizer.systick))
    }

    //Start the sampling ADCs and DACS Timers
    #[task(priority = 1, local=[sampling_timer])]
    fn start(c: start::Context){
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// # Note
    /// Processing time for the DSP application code is bounded by the following constraints:
    ///
    /// DSP application code starts after the ADC has generated a batch of samples and must be
    /// completed by the time the next batch of ADC samples has been acquired (plus the FIFO buffer
    /// time). If this constraint is not met, firmware will panic due to an ADC input overrun.
    ///
    /// The DSP application code must also fill out the next DAC output buffer in time such that the
    /// DAC can switch to it when it has completed the current buffer. If this constraint is not met
    /// it's possible that old DAC codes will be generated on the output and the output samples will
    /// be delayed by 1 batch.
    ///
    /// Because the ADC and DAC operate at the same rate, these two constraints actually implement
    /// the same time bounds, meeting one also means the other is also met.
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[settings, telemetry, harmonic_generators], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context){

        //Define Shared and local resources
        let process::SharedResources {
            settings,
            telemetry,
            harmonic_generators,
        } = c.shared;        
        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
        } = c.local;

        //MAIN DSP ALGORITHM OCCURS HERE
        (settings, telemetry, harmonic_generators).lock(
            |settings, telemetry, harmonic_generators| {
                //Single channel build - only DI0 us used so DI1 set to false as intentionally unused
                let digital_inputs = [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                telemetry.digital_inputs = digital_inputs;

                //Define the hold state - now it is using digial_inputs[0] instead of di1
                let hold = settings.force_hold
                    || (digital_inputs[1] && settings.allow_hold);
                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    //Define our samples

                    let adc_samples = [adc0, adc1];
                    let dac_samples = [dac0, dac1];

                    //let channel_index: usize = if settings.is_set_channel0{0} else {1};

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);



                    for channel in 0..adc_samples.len() {
                    //Take mutable reference to harmonic slot for this channel
                    
                        //Need to append the harmoncics to the adc_samples

                        adc_samples[channel]
                        .iter()
                        .zip(dac_samples[channel].iter_mut())
                        .zip(&mut harmonic_generators[channel])
                        .for_each(|((ai, di), harmonic)|{

                            // let adc_i32: i32 = *ai as i32 + i16::MIN as i32;
                            // let mixed_i16 = adc_i32.saturating_add(harmonic as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16;

                            let ai_i16 = *ai as i16;
                            let mixed = ai_i16.saturating_add(harmonic);

                            let x = f32::from(mixed);
                            //let x = f32::from(*ai as i16);
                            // TO DO FIGER THIS OUT

                            let y = settings.iir_ch[channel].iter().zip(iir_state[channel].iter_mut()).fold(x, |yi, (ch, state)|{
                                let filter = if hold { &iir::Biquad::HOLD} else { ch };
                                filter.update(state, yi)
                            });
                            let y_i16: i16 = unsafe{ y.to_int_unchecked() };

                            //Write out to DAC buffer
                            *di = DacCode::from(y_i16).0;
                        });
                
                    }

                    // Stream the data.
                    const N: usize = BATCH_SIZE * core::mem::size_of::<i16>();
                    generator.add(|buf| {
                        // copy adc0, adc1, dac0, dac1 in that order from adc_samples/dac_samples arrays
                        for (data, buf) in adc_samples
                            .iter()
                            .chain(dac_samples.iter())
                            .zip(buf.chunks_exact_mut(N))

                        {
                            let data = unsafe {
                                core::slice::from_raw_parts(
                                    data.as_ptr() as *const MaybeUninit<u8>,
                                    N,
                                )
                            };
                            buf.copy_from_slice(data)
                        }
                        
                        N * 4
                    });
                    
                    // Update telemetry measurements.
                    // snapshot ADC telemetry: if ADC inactive, report 0 or keep existing snapshot
                    telemetry.adcs = [
                        AdcCode(adc_samples[0][0]),
                        AdcCode(adc_samples[1][0]),
                        
                    ];
                    // snapshot DAC telemetry similarly
                    telemetry.dacs = [
                        DacCode(dac_samples[0][0]),
                        DacCode(dac_samples[1][0]),

                    ];
                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                });
            },
        );
    }

    #[idle(shared=[network, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged(_path) => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    // We can't sleep if USB is not in suspend.
                    if c.shared.usb.lock(|usb| {
                        usb.state()
                            == usb_device::device::UsbDeviceState::Suspend
                    }) {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }


    //Settings update
    #[task(priority = 1, local=[afes, dds_clock_state, current_sense_dac], shared=[network, settings, harmonic_generators, pounder])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c.shared.network.lock(|net| *net.miniconf.settings());
        c.shared.settings.lock(|current| *current = settings);

        c.local.afes.0.set_gain(settings.afe[0]);
        c.local.afes.1.set_gain(settings.afe[1]);

        let offset_1 = &settings.v_offset;
        log::info!("Offset is {}", offset_1);
        
        if let Some(dac) = c.local.current_sense_dac.as_mut() {
            dac.write_voltage(settings.v_offset);
        }
        
        //TO DO - This would be where we update the SPI to CURRENT SENSE BOARD
        //Update harmonic generator
        let harmonic_parameters = &settings.harmonic_wave_parameters;
        
        for channel in 0..2{

            let basic_cfg = BasicConfig::<MAX_HARMONICS> {
                    zero_order_frequency: MAINS_FREQUENCY,
                    amplitude: core::array::from_fn(|i| {
                                harmonic_parameters[channel][i].amp
                            }),
                    phase: core::array::from_fn(|i| {harmonic_parameters[channel][i].phase/360.0}),
                };

            match basic_cfg.try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE) {
                Ok(config)=> {c.shared.harmonic_generators.lock(|harmonic_generator| harmonic_generator[channel].update_waveform(config));}
                Err(err) => log::error!(
                    "Failed to update harmonic generation on channel{}: {:?}",channel,
                    err
                ),
            }
        }
        log::info!(
                "Harmonic Amplitudes are {}, {}, {}, {}, {}",
                harmonic_parameters[0][0].amp,
                harmonic_parameters[0][1].amp,
                harmonic_parameters[0][2].amp,
                harmonic_parameters[0][3].amp,
                harmonic_parameters[0][4].amp,
            );
        log::info!(
                "Harmonic Phases are {}, {}, {}, {}, {}",
                harmonic_parameters[0][0].phase,
                harmonic_parameters[0][1].phase,
                harmonic_parameters[0][2].phase,
                harmonic_parameters[0][3].phase,
                harmonic_parameters[0][4].phase,

            );

        // Update Pounder configurations
        c.shared.pounder.lock(|pounder| {
            if let Some(pounder) = pounder {
                let pounder_settings = settings.pounder.as_ref().unwrap();
                // let mut clocking = c.local.dds_clock_state;
                pounder.update_dds(
                    *pounder_settings,
                    &mut c.local.dds_clock_state,
                );
            }
        });

        let target = settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));
        // log::info!(
        //         "Stream target set to {}.{}.{}.{}:{}",
        //         settings.stream_target.ip[0],
        //         settings.stream_target.ip[1],
        //         settings.stream_target.ip[2],
        //         settings.stream_target.ip[3],
        //         settings.stream_target.port,
        //     );
    }


    //Telemetry update
    #[task(priority = 1, shared=[network, settings, telemetry, pounder], local=[cpu_temp_sensor])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period, pounder_telemetry) =
            (c.shared.settings, c.shared.pounder).lock(|settings, pounder| {
                (
                    settings.afe,
                    settings.telemetry_period,
                    pounder.as_mut().map(|pdr| pdr.get_telemetry()),
                )
            });

        c.shared.network.lock(|net| {
            net.telemetry.publish(&telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
                pounder_telemetry,
            ))
        });

        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }


    //USB UPDATE    
    #[task(priority = 1, shared=[usb], local=[usb_terminal])]
    fn usb(mut c: usb::Context) {
        // Handle the USB serial terminal.
        c.shared.usb.lock(|usb| {
            usb.poll(&mut [c.local.usb_terminal.interface_mut().inner_mut()]);
        });

        c.local.usb_terminal.process().unwrap();

        // Schedule to run this task every 10 milliseconds.
        usb::spawn_after(10u64.millis()).unwrap();
    }
    
    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 4)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 4)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
    }

    #[task(binds = SPI4, priority = 4)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 SPI error");
    }

    #[task(binds = SPI5, priority = 4)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 SPI error");
    }

}
