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
    cartridge: Option<Box<dyn cartridge::Cartridge>>,
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
        self.cartridge = Some(Box::new(cartridge::new(rom_file)));
    }

    /// Read a byte from RAM at the specified address.  The NES has 2 kB of internal RAM occupying
    /// address space $0000 through $07FF; however, this address space is mirrored thrice in
    /// addresses $0800 through $1FFF, with the low 11 bits used as the address to fetch.
    fn read_ram_byte(&self, address: u16) -> u8 {
        self.ram[(address & RAM_MIRRORING_MASK) as usize]
    }
}

impl memory::Readable for SystemBus {
    fn read_byte(&self, address: u16) -> Option<u8> {
        match address {
            // Address maps to internal RAM
            0x0000..=RAM_END_ADDRESS => Some(self.read_ram_byte(address)),
            // Address did not map anywhere else, so assume it maps to cartridge
            _ => self.cartridge.as_ref().unwrap().read_byte(address),
        }
    }

    fn read_word(&self, address: u16) -> Option<u16> {
        // Cannot read from the last byte of mirrored RAM
        if address == (RAM_END_ADDRESS - 1) {
            None
        } else {
            memory::read_word_simple(self, address)
        }
    }
}
