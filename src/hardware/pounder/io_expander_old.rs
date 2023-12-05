use crate::hardware::{pounder::Error, I2c1Proxy};
use cfg_if::cfg_if;
use enum_iterator::Sequence;

cfg_if! {
    if #[cfg(feature = "pca9539")] {
        const POUNDER_IO_EXPANDER_ADDRESS: u8 = 0x74;
    } else if #[cfg(all(feature = "mcp23017"))] {
        const POUNDER_IO_EXPANDER_ADDRESS: u8 = 0x20;
    }
}

cfg_if! {
    if #[cfg(feature = "pca9539")] {
        #[derive(Debug, Copy, Clone, Sequence)]
        pub enum GpioPin {
            Led4Green,
            Led5Red,
            Led6Green,
            Led7Red,
            Led8Green,
            Led9Red,
            DetPwrDown0,
            DetPwrDown1,
            AttLe0,
            AttLe1,
            AttLe2,
            AttLe3,
            DdsReset,
            AttRstN,
            OscEnN,
            ExtClkSel,
        }

        impl From<GpioPin> for pca9539::expander::Bank {
            fn from(x: GpioPin) -> Self {
                match x {
                    GpioPin::Led4Green => Self::Bank0,
                    GpioPin::Led5Red => Self::Bank0,
                    GpioPin::Led6Green => Self::Bank0,
                    GpioPin::Led7Red => Self::Bank0,
                    GpioPin::Led8Green => Self::Bank0,
                    GpioPin::Led9Red => Self::Bank0,
                    GpioPin::DetPwrDown0 => Self::Bank0,
                    GpioPin::DetPwrDown1 => Self::Bank0,

                    GpioPin::AttLe0 => Self::Bank1,
                    GpioPin::AttLe1 => Self::Bank1,
                    GpioPin::AttLe2 => Self::Bank1,
                    GpioPin::AttLe3 => Self::Bank1,
                    GpioPin::DdsReset => Self::Bank1,
                    GpioPin::AttRstN => Self::Bank1,
                    GpioPin::OscEnN => Self::Bank1,
                    GpioPin::ExtClkSel => Self::Bank1,
                }
            }
        }

        impl From<GpioPin> for pca9539::expander::PinID {
            fn from(x: GpioPin) -> Self {
                match x {
                    GpioPin::Led4Green => Self::Pin0,
                    GpioPin::Led5Red => Self::Pin1,
                    GpioPin::Led6Green => Self::Pin2,
                    GpioPin::Led7Red => Self::Pin3,
                    GpioPin::Led8Green => Self::Pin4,
                    GpioPin::Led9Red => Self::Pin5,
                    GpioPin::DetPwrDown0 => Self::Pin6,
                    GpioPin::DetPwrDown1 => Self::Pin7,

                    GpioPin::AttLe0 => Self::Pin0,
                    GpioPin::AttLe1 => Self::Pin1,
                    GpioPin::AttLe2 => Self::Pin2,
                    GpioPin::AttLe3 => Self::Pin3,
                    GpioPin::DdsReset => Self::Pin4,
                    GpioPin::AttRstN => Self::Pin5,
                    GpioPin::OscEnN => Self::Pin6,
                    GpioPin::ExtClkSel => Self::Pin7,
                }
            }
        }
    } else if #[cfg(feature = "mcp23017")]{
        #[derive(Debug, Copy, Clone, Sequence)]
        pub enum GpioPin {
            Led4Green,
            Led5Red,
            Led6Green,
            Led7Red,
            Led8Green,
            Led9Red,
            AttLe0,
            AttLe1,
            AttLe2,
            AttLe3,
            AttRstN,
            OscEnN,
            ExtClkSel,
        }

        impl From<GpioPin> for mcp230xx::Mcp23017 {
            fn from(x: GpioPin) -> Self {
                match x {
                    GpioPin::Led4Green => Self::A0,
                    GpioPin::Led5Red => Self::A1,
                    GpioPin::Led6Green => Self::A2,
                    GpioPin::Led7Red => Self::A3,
                    GpioPin::Led8Green => Self::A4,
                    GpioPin::Led9Red => Self::A5,
                    GpioPin::AttLe0 => Self::B0,
                    GpioPin::AttLe1 => Self::B1,
                    GpioPin::AttLe2 => Self::B2,
                    GpioPin::AttLe3 => Self::B3,
                    GpioPin::AttRstN => Self::B5,
                    GpioPin::OscEnN => Self::B6,
                    GpioPin::ExtClkSel => Self::B7,
                }
            }
        }
    } else {
        pub enum GpioPin { }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Input,
    Output,
}

#[cfg(feature = "pca9539")]
impl From<Direction> for pca9539::expander::Mode {
    fn from(direction: Direction) -> Self {
        match direction {
            Direction::Input => Self::Input,
            Direction::Output => Self::Output,
        }
    }
}

#[cfg(feature = "mcp23017")]
impl From<Direction> for mcp230xx::Direction {
    fn from(direction: Direction) -> Self {
        match direction {
            Direction::Input => Self::Input,
            Direction::Output => Self::Output,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Level {
    Low = 0,
    High = 1,
}

#[cfg(feature = "mcp23017")]
impl From<Level> for mcp230xx::Level {
    fn from(level: Level) -> Self {
        match level {
            Level::Low => Self::Low,
            Level::High => Self::High,
        }
    }
}

impl From<bool> for Level {
    fn from(is_high: bool) -> Self {
        if is_high {
            Self::High
        } else {
            Self::Low
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

pub struct IoExpander {
    #[cfg(feature = "pca9539")]
    pub io_expander: pca9539::expander::PCA9539<I2c1Proxy>,

    #[cfg(all(feature = "mcp23017", not(feature = "pca9539")))]
    io_expander: mcp230xx::Mcp230xx<I2c1Proxy, mcp230xx::Mcp23017>,
}

impl IoExpander {
    pub fn new(i2c: I2c1Proxy) -> Self {
        #[cfg(feature = "pca9539")]
        return Self {
            io_expander: pca9539::expander::PCA9539::new(
                i2c,
                POUNDER_IO_EXPANDER_ADDRESS,
            ),
        };

        #[cfg(all(feature = "mcp23017", not(feature = "pca9539")))]
        return Self {
            io_expander: mcp230xx::Mcp230xx::new(
                i2c,
                POUNDER_IO_EXPANDER_ADDRESS,
            )
            .unwrap(),
        };

        #[cfg(not(any(feature = "mcp23017", feature = "pca9539")))]
        return Self {};
    }

    #[allow(unreachable_code)]
    pub fn set_level(
        &mut self,
        pin: GpioPin,
        level: Level,
    ) -> Result<(), Error> {
        #[cfg(feature = "pca9539")]
        {
            self.io_expander
                .set_state(pin.into(), pin.into(), level.into());
            return self
                .io_expander
                .write_output_state(pin.into())
                .map_err(|_| Error::I2c);
        }

        #[cfg(feature = "mcp23017")]
        return self
            .io_expander
            .set_level(pin.into(), level.into())
            .map_err(|_| Error::I2c);

        return Err(Error::UnsetExpander);
    }

    #[allow(unreachable_code)]
    pub fn set_direction(
        &mut self,
        pin: GpioPin,
        direction: Direction,
    ) -> Result<(), Error> {
        #[cfg(feature = "pca9539")]
        return self
            .io_expander
            .set_mode(pin.into(), pin.into(), direction.into())
            .map_err(|_| Error::I2c);

        #[cfg(feature = "mcp23017")]
        return self
            .io_expander
            .set_direction(pin.into(), direction.into())
            .map_err(|_| Error::I2c);

        return Err(Error::UnsetExpander);
    }
}
