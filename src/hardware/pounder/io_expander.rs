use super::Error;
use crate::hardware::I2c1Proxy;
use embedded_hal::digital::v2::PinState;
use enum_iterator::Sequence;
use mcp230xx::{Mcp23017, Mcp230xx};
use pca9539::expander::PCA9539;

pub const IO_EXPANDER_MCP23017_ADDRESS: u8 = 0x20;
pub const IO_EXPANDER_PCA9539_ADDRESS: u8 = 0x74;

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

impl From<GpioPin> for Mcp23017 {
    fn from(x: GpioPin) -> Self {
        match x {
            GpioPin::Led4Green => Self::A0,
            GpioPin::Led5Red => Self::A1,
            GpioPin::Led6Green => Self::A2,
            GpioPin::Led7Red => Self::A3,
            GpioPin::Led8Green => Self::A4,
            GpioPin::Led9Red => Self::A5,
            GpioPin::DetPwrDown0 => Self::A6, // used only pounder 1.2 onwards
            GpioPin::DetPwrDown1 => Self::A7, // used only pounder 1.2 onwards
            GpioPin::AttLe0 => Self::B0,
            GpioPin::AttLe1 => Self::B1,
            GpioPin::AttLe2 => Self::B2,
            GpioPin::AttLe3 => Self::B3,
            GpioPin::DdsReset => Self::B4, // used only pounder 1.2 onwards
            GpioPin::AttRstN => Self::B5,
            GpioPin::OscEnN => Self::B6,
            GpioPin::ExtClkSel => Self::B7,
        }
    }
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

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Input,
    Output,
}

impl From<Direction> for pca9539::expander::Mode {
    fn from(direction: Direction) -> Self {
        match direction {
            Direction::Input => Self::Input,
            Direction::Output => Self::Output,
        }
    }
}

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

impl From<PinState> for Level {
    fn from(pin_state: PinState) -> Self {
        match pin_state {
            PinState::High => Self::High,
            PinState::Low => Self::Low,
        }
    }
}

impl From<Level> for PinState {
    fn from(level: Level) -> PinState {
        match level {
            Level::High => Self::High,
            Level::Low => Self::Low,
        }
    }
}

pub enum IoExpander {
    Mcp23017(Mcp230xx<I2c1Proxy, Mcp23017>),
    Pca9539(PCA9539<I2c1Proxy>),
}

impl IoExpander {
    pub fn new(i2c: I2c1Proxy) -> Self {
        match Mcp230xx::new(i2c.clone(), IO_EXPANDER_MCP23017_ADDRESS) {
            Ok(expander) => Self::Mcp23017(expander),
            _ => Self::Pca9539(PCA9539::new(i2c, IO_EXPANDER_PCA9539_ADDRESS)),
        }
    }

    pub fn set_level(
        &mut self,
        pin: GpioPin,
        level: Level,
    ) -> Result<(), Error> {
        match self {
            Self::Mcp23017(expander) => expander
                .set_gpio(pin.into(), level.into())
                .map_err(|_| Error::I2c),
            Self::Pca9539(expander) => {
                expander.set_state(pin.into(), pin.into(), level.into());
                expander
                    .write_output_state(pin.into())
                    .map_err(|_| Error::I2c)
            }
        }
    }

    pub fn set_direction(
        &mut self,
        pin: GpioPin,
        direction: Direction,
    ) -> Result<(), Error> {
        match self {
            Self::Mcp23017(expander) => expander
                .set_direction(pin.into(), direction.into())
                .map_err(|_| Error::I2c),
            Self::Pca9539(expander) => expander
                .set_mode(pin.into(), pin.into(), direction.into())
                .map_err(|_| Error::I2c),
        }
    }

    pub fn get_pca9539_handle(&mut self) -> Result<&mut PCA9539<I2c1Proxy>, Error> {
        match self {
            Self::Mcp23017(_) => Err(Error::NotImplemented),
            Self::Pca9539(expander) => Ok(expander),
        }
    }
}
