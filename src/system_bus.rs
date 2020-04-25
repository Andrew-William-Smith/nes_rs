use super::cartridge;
use super::memory;
use super::memory::Readable;
use super::ui;
use imgui::{ChildWindow, Ui};

/// Bytes of internal RAM available to the NES.
const RAM_SIZE: usize = 0x0800;
/// Bit-mask used to extract the mirrored RAM address from a full address.
const RAM_MIRRORING_MASK: u16 = 0x07FF;
/// Final address mapped to internal RAM, including mirrored addresses.
const RAM_END_ADDRESS: u16 = 0x1FFF;

/// Address (in cartridge memory) of the reset vector.
const RESET_VECTOR_ADDRESS: u16 = 0xFFFC;
/// Address (in cartridge memory) of the IRQ/`BRK` vector.
const IRQ_VECTOR_ADDRESS: u16 = 0xFFFE;

/// The system bus of the NES, handling interaction between the CPU, PPU, APU, cartridge ROM, and
/// other memories.
pub struct SystemBus {
    /// Internal RAM.
    ram: Vec<u8>,
    /// The current cartridge connected to the system.
    cartridge: Option<Box<dyn cartridge::Cartridge + Send + Sync>>,
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

    /// Write the specified byte of data to RAM at the specified address.  The address may be
    /// mirrored.
    fn write_ram_byte(&mut self, address: u16, data: u8) -> bool {
        self.ram[(address & RAM_MIRRORING_MASK) as usize] = data;
        true
    }

    /// Return the reset vector, the address to which the program counter is set when the CPU
    /// resets.  The reset vector is read from the cartridge at address `$FFFC`.
    pub fn read_reset_vector(&self) -> u16 {
        self.read_word(RESET_VECTOR_ADDRESS).unwrap()
    }

    /// Return the IRQ/`BRK` vector, the address to which the program counter is set when the CPU
    /// receives an interrupt.  The IRQ vector is read from the cartridge at address `$FFFE`.
    pub fn read_irq_vector(&self) -> u16 {
        self.read_word(IRQ_VECTOR_ADDRESS).unwrap()
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
        if address == RAM_END_ADDRESS {
            None
        } else {
            memory::read_word_simple(self, address)
        }
    }
}

impl memory::Writable for SystemBus {
    fn write_byte(&mut self, address: u16, data: u8) -> bool {
        match address {
            0x0000..=RAM_END_ADDRESS => self.write_ram_byte(address, data),
            _ => false,
        }
    }
}

impl ui::Visualisable for SystemBus {
    fn display(&mut self, ui: &Ui) {
        use imgui::*;

        // Disassembly window
        if let Some(cartridge) = self.cartridge.as_ref() {
            Window::new(im_str!("ROM Disassembly"))
                .size([280.0, 555.0], Condition::Appearing)
                .position([180.0, 10.0], Condition::Appearing)
                .build(ui, || {
                    ui.text(format!("Mapper number: {:03}", cartridge.mapper_number()));

                    ChildWindow::new("Disassembled code")
                        .size([0.0, 0.0])
                        .border(true)
                        .build(ui, || {
                            ui.columns(2, im_str!("Code columns"), true);
                            for (address, instruction) in cartridge.disassemble().iter() {
                                ui.text(address);
                                ui.next_column();
                                ui.text(instruction);
                                ui.next_column();
                            }
                        });
                });
        }
    }
}
