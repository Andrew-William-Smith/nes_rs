use super::cpu;
use super::memory;
use std::collections::BTreeMap;
use std::fs;
use std::io::{BufReader, Read};

mod nrom;

/// Bytes in a single page of iNES PRG ROM.
const PRG_ROM_PAGE_SIZE: usize = 0x4000;

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
pub trait Cartridge: memory::Readable {
    /// Return the iNES mapper number used by this cartridge.
    fn mapper_number(&self) -> u16;

    /// Return whether the specified address maps to data in this cartridge.
    fn handles_address(&self, address: u16) -> bool;

    /// Disassemble the PRG ROM to allow it to be viewed in the UI.  Attempt to synchronise the
    /// disassembly to the specified program counter.
    fn disassemble(&self, program_counter: u16) -> BTreeMap<u16, (String, String)>;
}

/// Initialise a cartridge with the ROM data stored in the specified file.
pub fn new(rom_file: &String) -> impl Cartridge {
    // Open file for reading
    let file = fs::File::open(rom_file).expect("Unable to read the specified file.");
    let file_meta = fs::metadata(rom_file).expect("Unable to read file metadata.");
    let mut reader = BufReader::with_capacity(file_meta.len() as usize, file);

    // Confirm iNES magic string
    if !is_ines_rom(&mut reader) {
        panic!(
            "The specified file \"{}\" is not a valid iNES ROM (wrong magic string).",
            rom_file
        );
    }

    // Read metadata and determine mapper
    let metadata = read_ines_metadata(&mut reader);
    let mapper = identify_mapper(
        metadata[INesMetadata::Flags7 as usize],
        metadata[INesMetadata::Flags6 as usize],
    );

    // Read in ROM data
    let num_prg_pages = metadata[INesMetadata::PrgRomPages as usize] as usize;
    let prg_rom = read_prg_rom(&mut reader, num_prg_pages);

    // Construct a cartridge of the specified mapper
    match mapper {
        0 => nrom::NromCartridge::new(prg_rom),
        m => panic!("Mapper number {} is unsupported.", m),
    }
}

/// Return whether the file handled by the specified reader is a valid iNES-format ROM by
/// validating the magic string at the beginning of the file.
fn is_ines_rom(reader: &mut BufReader<fs::File>) -> bool {
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
fn read_ines_metadata(reader: &mut BufReader<fs::File>) -> [u8; INES_METADATA_BYTES] {
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
fn read_prg_rom(reader: &mut BufReader<fs::File>, pages: usize) -> Vec<u8> {
    let mut rom = vec![0; pages * PRG_ROM_PAGE_SIZE];
    let mut rom_reader = reader.take((pages * PRG_ROM_PAGE_SIZE) as u64);
    rom_reader
        .read(&mut rom)
        .expect("Failed attempting to read PRG rom.");
    rom
}

/// Return a textual representation of the instruction at the specified address, along with its
/// bytecode and the number of bytes it occupied in the PRG ROM.
#[rustfmt::skip]
pub fn disassemble_instruction(cart: &impl Cartridge, address: u16) -> (String, String, u16) {
    // Get the instruction definition
    let opcode = cart.read_byte(address).unwrap();
    let instruction = &cpu::INSTRUCTIONS[opcode as usize];

    // Get the next byte and word for convenient operand encoding
    let next_byte = cart.read_byte(address.wrapping_add(1)).unwrap_or_default();
    let next_word = cart.read_word(address.wrapping_add(1)).unwrap_or_default();

    // Format the operand
    let (operand, extra_bytes) = match instruction.addressing_mode {
        cpu::AddressingMode::Accumulator => (String::from("A"), 0),
        cpu::AddressingMode::Immediate => (format!("#${:02X}", next_byte), 1),
        cpu::AddressingMode::Absolute => (format!("${:04X}", next_word), 2),
        cpu::AddressingMode::ZeroPage => (format!("${:02X}", next_byte), 1),
        cpu::AddressingMode::IndexedZeroPageX => (format!("${:02X},X", next_byte), 1),
        cpu::AddressingMode::IndexedZeroPageY => (format!("${:02X},Y", next_byte), 1),
        cpu::AddressingMode::IndexedAbsoluteX => (format!("${:04X},X", next_word), 2),
        cpu::AddressingMode::IndexedAbsoluteY => (format!("${:04X},Y", next_word), 2),
        cpu::AddressingMode::Implied => (String::new(), 0),
        cpu::AddressingMode::Relative => {
            let target_address =
                ((address as i32).wrapping_add(2)).wrapping_add(next_byte as i32) as u16;
            (format!("${:04X}", target_address), 1)
        }
        cpu::AddressingMode::IndexedIndirect => (format!("(${:02X},X)", next_byte), 1),
        cpu::AddressingMode::IndirectIndexed => (format!("(${:02X}),Y", next_byte), 1),
        cpu::AddressingMode::AbsoluteIndirect => (format!("(${:04X})", next_word), 2),
    };

    // Format bytecode
    let bytecode = match extra_bytes {
        0 => format!("{:02X}", opcode),
        1 => format!("{:02X} {:02X}", opcode, next_byte),
        _ => format!("{:02X} {:02X} {:02X}", opcode, next_word as u8, (next_word >> 8) as u8),
    };

    let assembly = format!("{:>4} {}", instruction.mnemonic, operand);
    (assembly, bytecode, 1 + extra_bytes)
}
