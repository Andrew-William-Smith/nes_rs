use std::fs::File;
use std::io::{BufReader, Read};

/// Bytes in a single page of iNES PRG ROM.
const PRG_ROM_PAGE_SIZE: usize = 0x4000;
/// Starting address mapped to ROM.
const ROM_START_ADDRESS: u16 = 0x8000;
/// Final address mapped to ROM.
const ROM_END_ADDRESS: u16 = 0xFFFF;

/// Magic identifier for the [iNES file format](https://wiki.nesdev.com/w/index.php/INES).
const INES_MAGIC_STRING: &str = "NES\x1A";
/// The number of bytes of metadata in the iNES format following the magic string.
const INES_METADATA_BYTES: usize = 12;

/// The significant metadata bytes in the iNES format header.
enum INesMetadata {
    PrgRomPages,
    ChrRomPages,
    Flags6,
    Flags7,
    PrgRamPages,
    TvSystem,
    UnofficialPrgRam,
}

/// A readable NES cartridge.
pub struct Cartridge {
    /// PRG ROM, the executable code of the game.
    prg_rom: Vec<u8>,
    /// The memory mapper used in this cartridge.
    mapper: u16,
}

impl Cartridge {
    /// Initialise this cartridge with the ROM data stored in the specified file.
    pub fn new(rom_file: &String) -> Cartridge {
        // Open file for reading
        let file = File::open(rom_file).expect("Unable to read the specified file.");
        let mut reader = BufReader::new(file);

        // Confirm iNES magic string
        if !Cartridge::is_ines_rom(&mut reader) {
            panic!(
                "The specified file \"{}\" is not a valid iNES ROM (wrong magic string).",
                rom_file
            );
        }

        // Read metadata and determine mapper
        let metadata = Cartridge::read_ines_metadata(&mut reader);
        let mapper = Cartridge::identify_mapper(
            metadata[INesMetadata::Flags7 as usize],
            metadata[INesMetadata::Flags6 as usize],
        );

        // Read in ROM data
        let prg_rom = Cartridge::read_prg_rom(
            &mut reader,
            metadata[INesMetadata::PrgRomPages as usize] as usize,
        );

        Cartridge { prg_rom, mapper }
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

    /// Construct the mapper number from the upper nybbles of the specified high and low bytes,
    /// which should correspond to iNES metadata bytes Flags 6 and 7.
    fn identify_mapper(high_byte: u8, low_byte: u8) -> u16 {
        ((high_byte & 0xF0) | ((low_byte >> 4) & 0xF)) as u16
    }

    /// Read the specified number of pages of PRG ROM into memory from the file controlled by the
    /// specified reader.
    fn read_prg_rom(reader: &mut BufReader<File>, pages: usize) -> Vec<u8> {
        let mut rom = vec![0; pages * PRG_ROM_PAGE_SIZE];
        let mut rom_reader = reader.take((pages * PRG_ROM_PAGE_SIZE) as u64);
        rom_reader
            .read(&mut rom)
            .expect("Failed attempting to read PRG rom.");
        rom
    }
}