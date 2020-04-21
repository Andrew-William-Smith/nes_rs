use super::memory;
use std::fs::File;
use std::io::{BufReader, Read};

/// Bytes of internal RAM available to the NES.
const RAM_SIZE: usize = 0x0800;
/// Bit-mask used to extract the mirrored RAM address from a full address.
const RAM_MIRRORING_MASK: u16 = 0x07FF;
/// Final address mapped to internal RAM, including mirrored addresses.
const RAM_END_ADDRESS: u16 = 0x1FFF;

/// Bytes of ROM available to the NES using Mapper 0.
const ROM_SIZE: usize = 0x8000;
/// Starting address mapped to ROM.
const ROM_START_ADDRESS: u16 = 0x8000;
/// Final address mapped to ROM.
const ROM_END_ADDRESS: u16 = 0xFFFF;

/// Magic identifier for the [iNES file format](https://wiki.nesdev.com/w/index.php/INES).
const INES_MAGIC_STRING: &str = "NES\x1A";
/// The number of bytes of metadata in the iNES format following the magic string.
const INES_METADATA_BYTES: usize = 12;

/// The system bus of the NES, handling interaction between the CPU, PPU, APU, cartridge ROM, and
/// other memories.
pub struct SystemBus {
    /// Internal RAM.
    ram: Vec<u8>,
    /// Cartridge ROM.  This will eventually need to be more complex to support other mappers, but
    /// it should work fine for targeting Mapper 0.
    rom: Vec<u8>,
}

impl SystemBus {
    /// Initialise a new system bus with all memory cleared.
    pub fn new() -> SystemBus {
        SystemBus {
            ram: vec![0; RAM_SIZE],
            rom: vec![0; 0],
        }
    }

    /// Read the specified ROM file into memory beginning at address `$8000`.
    pub fn load_rom(&mut self, rom_file: &String) {
        // Open file for reading
        let file = File::open(rom_file).expect("Unable to read the specified file.");
        let mut reader = BufReader::new(file);

        // Confirm iNES magic string
        if !SystemBus::is_ines_rom(&mut reader) {
            panic!("The specified file is not a valid iNES ROM (wrong magic string).");
        }

        let metadata = SystemBus::read_ines_metadata(&mut reader);
        //file.read(&mut self.rom).expect("Buffer overflow reading ROM!");
    }

    /// Return whether the file handled by the specified reader is a valid iNES-format ROM by
    /// validating the magic string at the beginning of the file.
    fn is_ines_rom(reader: &mut BufReader<File>) -> bool {
        let mut buffer = [0; INES_MAGIC_STRING.len()];
        let mut magic_reader = reader.take(INES_MAGIC_STRING.len() as u64);
        magic_reader
            .read(&mut buffer)
            .expect("Failed attempting to read iNES magic string.");
        let magic_string = String::from_utf8_lossy(&buffer);
        magic_string.eq(INES_MAGIC_STRING)
    }

    /// Read the iNES metadata from the file handled by the specified reader.  The file should
    /// already have been verified to be an iNES ROM.
    fn read_ines_metadata(reader: &mut BufReader<File>) -> [u8; INES_METADATA_BYTES] {
        let mut buffer = [0; INES_METADATA_BYTES];
        let mut meta_reader = reader.take(INES_METADATA_BYTES as u64);
        meta_reader
            .read(&mut buffer)
            .expect("Failed attempting to read iNES metadata.");
        buffer
    }
}
