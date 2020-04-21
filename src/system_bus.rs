use super::cartridge;
use super::memory;

/// Bytes of internal RAM available to the NES.
const RAM_SIZE: usize = 0x0800;
/// Bit-mask used to extract the mirrored RAM address from a full address.
const RAM_MIRRORING_MASK: u16 = 0x07FF;
/// Final address mapped to internal RAM, including mirrored addresses.
const RAM_END_ADDRESS: u16 = 0x1FFF;

/// The system bus of the NES, handling interaction between the CPU, PPU, APU, cartridge ROM, and
/// other memories.
pub struct SystemBus {
    /// Internal RAM.
    ram: Vec<u8>,
    /// The current cartridge connected to the system.
    cartridge: Option<cartridge::Cartridge>,
}

impl SystemBus {
    /// Initialise a new system bus with all memory cleared.
    pub fn new() -> SystemBus {
        SystemBus {
            ram: vec![0; RAM_SIZE],
            cartridge: None,
        }
    }

    /// Read the specified ROM file into memory beginning at address `$8000`.
    pub fn load_rom(&mut self, rom_file: &String) {
        // "Insert" the cartridge described in the specified file
        self.cartridge = Some(cartridge::Cartridge::new(rom_file));
    }
}
