use core::fmt::Write;
use stm32h7xx_hal::flash::LockedFlashBank;

#[derive(Clone, serde::Serialize, serde::Deserialize, miniconf::Tree)]
pub struct Settings {
    pub broker: heapless::String<255>,
    pub id: heapless::String<23>,
    #[serde(skip)]
    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self::new(self.mac)
    }
}

impl Settings {
    pub fn new(mac: smoltcp_nal::smoltcp::wire::EthernetAddress) -> Self {
        let mut id = heapless::String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: "mqtt".into(),
            id,
            mac,
        }
    }
}

pub struct Flash(pub LockedFlashBank);

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error =
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl embedded_storage::nor_flash::NorFlash for Flash {
    const WRITE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::WRITE_SIZE;
    const ERASE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.write(offset, bytes)
    }
}
