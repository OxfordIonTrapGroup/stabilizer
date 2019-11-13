use smoltcp::wire::{IpCidr, IpAddress, EthernetAddress};

use super::eeprom;
use stm32h7::stm32h743 as pac;

extern "C" {
    static _flash_config_base: [u8; 128*1024];
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub ip: Option<IpCidr>,
    pub mac: Option<EthernetAddress>,
    magic: u32
}

const MAGIC: u32 = 0xdeadbeef;

impl Config {
    pub fn new() -> Config {
        Config {
            ip: Some(IpCidr::new(IpAddress::v4(10, 0, 16, 99), 24)),
            mac: None,
            magic: MAGIC
        }
    }

    pub fn load(&mut self) {
        let stored_config: Config =  unsafe{ core::ptr::read(_flash_config_base.as_ptr() as *const _) };

        if stored_config.magic != MAGIC {
            info!("Stored config invalid - using defaults");
            return;
        }
    }

    // pub fn save(&self) {
    // }

    pub fn get_hardware_addr(&self, i2c: &pac::I2C2) -> EthernetAddress {
        match self.mac {
            Some(mac) => mac,
            None => EthernetAddress(
                match eeprom::read_eui48(i2c) {
                    Err(_) => {
                        info!("Could not read EEPROM, using default MAC address");
                        [0x10, 0xE2, 0xD5, 0x00, 0x03, 0x00]
                    },
                    Ok(raw_mac) => raw_mac
                })
        }
    }
}