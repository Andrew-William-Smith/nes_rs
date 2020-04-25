use super::memory;
use crate::cartridge;
use crate::cartridge::Cartridge;

const NROM_MAPPER_NUMBER: u16 = 0;

/// The number of bytes in an NROM-128 PRG ROM.
const NROM_128_PRG_SIZE: usize = 0x4000;
/// The first address to which PRG ROM is mapped.
const NROM_PRG_START: u16 = 0x8000;
/// The last address to which PRG ROM is mapped for 256-kb cartridges, and mirrored ROM is mapped
/// for 128-kb cartridges.
const NROM_PRG_END: u16 = 0xFFFF;

/// Mask applied to determine mirrored addresses for NROM-128 cartridges.
const NROM_128_MIRRORING_MASK: u16 = 0x3FFF;
/// Mask applied to determine mirrored addresses for NROM-256 cartridges.
const NROM_256_MIRRORING_MASK: u16 = 0x7FFF;

/// A cartridge that uses the NROM mapper (iNES mapper 000).  This mapper has two variants, NROM-128
/// and NROM-256, which provide 128 and 256 kilobits of PRG ROM, respectively.  The address space of
/// NROM-128 is mirrored once to provide the same interface as NROM-256.
pub struct NromCartridge {
    /// PRG ROM, the executable code in the cartridge.
    prg_rom: Vec<u8>,
    /// Whether this ROM contains 128 (`false`) or 256 (`true`) kilobits.
    is_nrom_256: bool,
}

impl NromCartridge {
    pub fn new(prg_rom: Vec<u8>) -> NromCartridge {
        NromCartridge {
            is_nrom_256: prg_rom.len() > NROM_128_PRG_SIZE,
            prg_rom,
        }
    }
}

impl Cartridge for NromCartridge {
    fn mapper_number(&self) -> u16 {
        NROM_MAPPER_NUMBER
    }

    fn handles_address(&self, address: u16) -> bool {
        address >= NROM_PRG_START && address <= NROM_PRG_END
    }

    fn disassemble(&self) -> Vec<(String, String)> {
        let mut assembly = Vec::new();
        let mut rom_address = NROM_PRG_START;
        while rom_address >= NROM_PRG_START {
            let (instruction, bytecode, size) =
                cartridge::disassemble_instruction(self, rom_address);
            assembly.push((format!("{:4X}: {:8}", rom_address, bytecode), instruction));
            rom_address = rom_address.wrapping_add(size);
        }

        assembly
    }
}

impl super::memory::Readable for NromCartridge {
    fn read_byte(&self, address: u16) -> Option<u8> {
        if self.handles_address(address) {
            let mask = if self.is_nrom_256 {
                NROM_256_MIRRORING_MASK
            } else {
                NROM_128_MIRRORING_MASK
            };
            Some(self.prg_rom[(address & mask) as usize])
        } else {
            None
        }
    }

    fn read_word(&self, address: u16) -> Option<u16> {
        memory::read_word_simple(self, address)
    }
}
