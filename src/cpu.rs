// Due to the way the instructions are encoded, there will always be a few unused arguments.
#![allow(unused_variables)]

use super::memory::Readable;
use super::system_bus;
use super::ui;
use imgui::Ui;
use std::fmt;

/// A macro to quickly define instructions.
macro_rules! ins {
    ( $mnemonic:expr, $opcode:expr, $cycles:expr, $mode:ident, $operation:ident ) => {
        Instruction {
            mnemonic: $mnemonic,
            opcode: $opcode,
            cycles: $cycles,
            addressing_mode: AddressingMode::$mode,
            operation: CPU::$operation,
        }
    };
}

/// The value by which the 2A03 stack is offset from its actual contents.
const STACK_OFFSET: u16 = 0x100;

/// An implementation of the Ricoh 2A03 processor used by the NES,  a very slight variation of the
/// [MOS 6502](http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf) without decimal mode.
pub struct CPU {
    /// The programmer-accessible registers in the 2A03.
    reg: RegisterFile,
    /// The system bus, which handles all interaction with memory.
    bus: system_bus::SystemBus,
    /// The number of cycles remaining in the current instruction.
    cycles_remaining: u8,
    /// The cycle count after startup or the last reset.
    cycle: u64,
    /// Whether execution is automatic or single-stepping is enabled.
    running: bool,
    /// Whether the CPU is halted due to an error.
    faulted: bool,
}

impl CPU {
    /// Create a new instance of the 2A03 CPU in the post-startup state.
    pub fn new() -> CPU {
        CPU {
            reg: RegisterFile::new(),
            bus: system_bus::SystemBus::new(),
            cycles_remaining: 0,
            cycle: 0,
            running: false,
            faulted: false,
        }
    }

    /// Trigger a read of the specified ROM file on the system bus.
    pub fn load_rom(&mut self, rom_file: &String) {
        self.bus.load_rom(rom_file);
        // Set the program counter to the address in the reset vector
        // TODO: Restore after nestest passes
        // self.reg.PC = self.bus.read_reset_vector();
        self.reg.PC = 0xC000;
    }

    /// Perform the current operation for a single tick of the clock.  If a previous instruction has
    /// not finished executing, the remaining cycles counter will be decremented but no other
    /// operation will be performed.
    fn step_cycle(&mut self) {
        if self.cycles_remaining == 0 {
            self.execute_instruction();
        }
        self.cycles_remaining -= 1;
        self.cycle += 1;
    }

    /// Kick off execution of the instruction pointed to by the current program counter.
    fn execute_instruction(&mut self) {
        // Ensure that the program counter points to a valid memory address
        if let Some(opcode) = self.bus.read_byte(self.reg.PC) {
            // Decode the instruction
            self.reg.PC += 1;
            let instruction: &Instruction = &INSTRUCTIONS[opcode as usize];

            // Fetch the memory required to execute the instruction
            if let Some(memory) = self.memory_fetch(&instruction.addressing_mode) {
                // Execute the instruction and set its cycle count
                (instruction.operation)(self, opcode, &memory);
                self.cycles_remaining += instruction.cycles + memory.additional_cycles;
            } else {
                self.faulted = true;
            }
        } else {
            self.faulted = true;
        }
    }

    /// Fetch a byte from the memory address at the current program counter according to the
    /// specified addressing mode.
    fn memory_fetch(&mut self, addressing_mode: &AddressingMode) -> Option<FetchedMemory> {
        match addressing_mode {
            AddressingMode::Absolute => self.fetch_absolute(),
            _ => {
                self.faulted = true;
                None
            }
        }
    }
}

/// Flag offsets within the processor status register (`P`).
enum StatusFlag {
    Negative = 1 << 7,
    Overflow = 1 << 6,
    Unused = 1 << 5,
    Break = 1 << 4,
    DecimalMode = 1 << 3,
    IrqDisable = 1 << 2,
    Zero = 1 << 1,
    Carry = 1 << 0,
}

/// Names and masks for the processor status flags, required for iteration.
static STATUS_FLAG_NAMES: &[(char, u8)] = &[
    ('N', StatusFlag::Negative as u8),
    ('V', StatusFlag::Overflow as u8),
    (' ', StatusFlag::Unused as u8),
    ('B', StatusFlag::Break as u8),
    ('D', StatusFlag::DecimalMode as u8),
    ('I', StatusFlag::IrqDisable as u8),
    ('Z', StatusFlag::Zero as u8),
    ('C', StatusFlag::Carry as u8),
];

/// The register file used by the 2A03, as defined on page 8 of the Ricoh R6500 datasheet.
#[allow(non_snake_case)]
struct RegisterFile {
    /// Accumulator
    A: u8,
    /// Index register Y
    Y: u8,
    /// Index register X
    X: u8,
    /// Program counter
    PC: u16,
    /// Stack pointer (offset -0x100)
    S: u8,
    /// Processor status register
    P: u8,
    /// Visualisation window
    vis: RegisterFileVisualisation,
}

#[derive(Default)]
struct RegisterFileVisualisation {
    base: usize,
}

impl RegisterFile {
    /// Create a new instance of the CPU's register file in the post-startup state as described
    /// [here](http://wiki.nesdev.com/w/index.php/CPU_power_up_state).
    pub fn new() -> RegisterFile {
        RegisterFile {
            // Hardware registers
            A: 0,
            Y: 0,
            X: 0,
            PC: 0,
            S: 0xFD,
            P: 0x34,
            // Visualisation state
            vis: RegisterFileVisualisation::default(),
        }
    }

    /// Return whether the specified status flag in register `P` is set.
    pub fn get_status_flag(&self, flag: StatusFlag) -> bool {
        (self.P & flag as u8) != 0
    }

    /// Set the specified status flag in the processor status register.
    pub fn set_status_flag(&mut self, flag: StatusFlag, set: bool) {
        if set {
            self.P |= flag as u8;
        } else {
            self.P &= !(flag as u8);
        }
    }

    /// Return the actual (non-offset) memory address referenced by the stack-pointer.
    pub fn stack_pointer_address(&self) -> u16 {
        self.S as u16 + STACK_OFFSET
    }
}

/// A single 2A03 instruction and the information necessary to decode it.
struct Instruction {
    mnemonic: &'static str,
    opcode: u8,
    cycles: u8,
    addressing_mode: AddressingMode,
    operation: fn(&mut CPU, u8, &FetchedMemory),
}

/// A memory addressing mode supported by the 2A03.
enum AddressingMode {
    Accumulator,
    Immediate,
    Absolute,
    ZeroPage,
    IndexedZeroPageX,
    IndexedZeroPageY,
    IndexedAbsoluteX,
    IndexedAbsoluteY,
    Implied,
    Relative,
    IndexedIndirect,
    IndirectIndexed,
    AbsoluteIndirect,
}

/// Result of a memory fetch operation.
struct FetchedMemory {
    data: u8,
    address: u16,
    additional_cycles: u8,
}

/// List of all instructions provided by the 2A03.
const INSTRUCTIONS: [Instruction; 256] = [
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("JMP", 0x4C, 3, Absolute, instruction_jmp),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
    ins!("UUU", 0x00, 1, Absolute, unimplemented_instruction),
];

impl CPU {
    /// Fetch two bytes at the current program counter, from which a full memory address is
    /// constructed; this addressing mode allows access to the full memory range allowed by the
    /// 2A03.  The single byte located at this address is returned.
    fn fetch_absolute(&mut self) -> Option<FetchedMemory> {
        let address = self.bus.read_word(self.reg.PC)?;
        self.reg.PC += 2;
        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles: 0,
        })
    }

    /// `JMP` instruction.  Unconditionally branches to the specified memory address.
    ///
    /// Flags modified: *None*
    fn instruction_jmp(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.PC = fetched.address;
    }

    /// Halt on invalid instruction.
    fn unimplemented_instruction(&mut self, opcode: u8, fetched: &FetchedMemory) {
        println!("Invalid instruction with opcode ${:02X}", opcode);
        self.faulted = true;
    }
}

impl ui::Visualisable for CPU {
    fn display(&mut self, ui: &Ui) {
        use imgui::*;
        Window::new(im_str!("CPU Status"))
            .size([160.0, 170.0], Condition::Appearing)
            .position([10.0, 205.0], Condition::Appearing)
            .resizable(false)
            .build(ui, || {
                ui.text(format!("Cycle: {}", self.cycle));
                ui.text(format!("Instr. cycles: {}", self.cycles_remaining));
                ui.text(format!("Status:"));
                ui.same_line(0.0);
                if self.faulted {
                    ui.text_colored([1.0, 0.0, 0.0, 1.0], "FAULT");
                } else if self.cycles_remaining == 0 {
                    ui.text_colored([0.0, 1.0, 0.0, 1.0], "READY");
                } else {
                    ui.text_colored([1.0, 1.0, 0.0, 1.0], "WAITING");
                }
                ui.separator();

                ui.text(im_str!("Debugging controls"));
                if self.faulted {
                    ui.text_colored([1.0, 0.0, 0.0, 1.0], "Fault occurred!");
                    ui.text_colored([1.0, 0.0, 0.0, 1.0], "Execution halted.");
                } else {
                    if self.running {
                        ui.button(im_str!("Pause execution"), [145.0, 18.0]);
                    } else {
                        ui.button(im_str!("Resume execution"), [145.0, 18.0]);

                        if ui.button(im_str!("Step cycle"), [145.0, 18.0]) {
                            self.step_cycle();
                        }

                        ui.button(im_str!("Step instruction"), [145.0, 18.0]);
                    }
                }
            });

        // Display subcomponents
        self.reg.display(ui);
    }
}

impl ui::Visualisable for RegisterFile {
    fn display(&mut self, ui: &Ui) {
        use imgui::*;

        // Determine the base in which the number should be formatted
        fn format_register_value<
            T: Into<u64> + fmt::Binary + fmt::Octal + fmt::UpperHex + fmt::Display,
        >(
            name: &str,
            value: T,
            size_binary: usize,
            base: usize,
        ) -> String {
            match base {
                0 => format!("{:>2}: {:0width$b}", name, value, width = size_binary),
                1 => {
                    let width = (size_binary as f32 / 3.0).ceil() as usize;
                    format!("{:>2}: {:0width$o}", name, value, width = width)
                }
                2 => format!("{:>2}: {}", name, value),
                3 => {
                    let width = (size_binary as f32 / 4.0).ceil() as usize;
                    format!("{:>2}: {:0width$X}", name, value, width = width)
                }
                b => panic!("Invalid register file base {}", b),
            }
        }

        // Format the status flag display
        fn format_status_flags(p_value: u8) -> String {
            if p_value == 0 {
                String::from("    (none set)")
            } else {
                let mut status_format = String::with_capacity(8);
                status_format.push_str("    ");
                // Build format string from members
                for (name, mask) in STATUS_FLAG_NAMES.iter() {
                    if (p_value & *mask) != 0 {
                        status_format.push(*name);
                    } else {
                        status_format.push(' ');
                    }
                }
                status_format
            }
        }

        let base = self.vis.base;
        Window::new(im_str!("CPU Registers"))
            .size([160.0, 185.0], Condition::Appearing)
            .position([10.0, 10.0], Condition::Appearing)
            .resizable(false)
            .build(ui, || {
                ui.text(format_register_value("A", self.A, 8, base));
                ui.text(format_register_value("Y", self.Y, 8, base));
                ui.text(format_register_value("X", self.X, 8, base));
                ui.separator();

                ui.text(format_register_value("PC", self.PC, 16, base));
                let s_value = self.stack_pointer_address();
                ui.text(format_register_value("S", s_value, 9, base));
                ui.separator();

                ui.text(format_register_value("P", self.P, 8, base));
                ui.text(format_status_flags(self.P));
                ui.separator();

                ComboBox::new(im_str!("Base")).build_simple_string(
                    ui,
                    &mut self.vis.base,
                    &[
                        im_str!("2 (binary)"),
                        im_str!("8 (octal)"),
                        im_str!("10 (decimal)"),
                        im_str!("16 (hexadecimal)"),
                    ],
                );
            });
    }
}
