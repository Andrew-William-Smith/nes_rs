// Due to the way the instructions are encoded, there will always be a few unused arguments.
#![allow(unused_variables)]

use super::memory::{Readable, Writable};
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

/// Shorthand for swapping the values in two registers.
macro_rules! swap_registers {
    ( $cpu:ident, $source:expr, $dest:expr ) => {
        let data = $source;
        $cpu.set_value_status(data);
        $dest = data;
    };
}

/// A function that implements a CPU instruction;
type InstructionFn = fn(&mut CPU, u8, &FetchedMemory);

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
    cycles_remaining: i8,
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
            running: true,
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

    /// Return whether this CPU is in a state in which it can run continuously: that is, the user
    /// has not disabled continuous execution and the CPU has not faulted.
    pub fn is_running(&self) -> bool {
        self.running && !self.faulted
    }

    /// Perform the current operation for a single tick of the clock.  If a previous instruction has
    /// not finished executing, the remaining cycles counter will be decremented but no other
    /// operation will be performed.
    pub fn step_cycle(&mut self) {
        if self.cycles_remaining == 0 {
            self.execute_instruction();
        }
        self.cycles_remaining -= 1;
        self.cycle += 1;
    }

    /// Perform the current operation until it has finished executing.
    fn step_instruction(&mut self) {
        self.execute_instruction();
        self.cycle += self.cycles_remaining as u64;
        self.cycles_remaining = 0;
    }

    /// Kick off execution of the instruction pointed to by the current program counter.
    fn execute_instruction(&mut self) {
        // Print CPU status in nestest log format
        println!(
            "{:04X} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} CYC:{}",
            self.reg.PC,
            self.reg.A,
            self.reg.X,
            self.reg.Y,
            self.reg.P & 0xEF,
            self.reg.S,
            self.cycle + 7
        );

        // Ensure that the program counter points to a valid memory address
        if let Some(opcode) = self.bus.read_byte(self.reg.PC) {
            // Decode the instruction
            self.reg.PC += 1;
            let instruction: &Instruction = &INSTRUCTIONS[opcode as usize];

            // Fetch the memory required to execute the instruction
            if let Some(memory) = self.memory_fetch(&instruction.addressing_mode) {
                // Execute the instruction and set its cycle count
                (instruction.operation)(self, opcode, &memory);
                self.cycles_remaining += (instruction.cycles + memory.additional_cycles) as i8;
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
            AddressingMode::Accumulator => Some(FetchedMemory {
                data: self.reg.A,
                address: 0,
                additional_cycles: 0,
            }),
            AddressingMode::Immediate => self.fetch_immediate(),
            AddressingMode::Absolute => self.fetch_absolute(),
            AddressingMode::ZeroPage => self.fetch_zero_page(),
            AddressingMode::IndexedZeroPageX => self.fetch_indexed_zero_page(self.reg.X),
            AddressingMode::IndexedZeroPageY => self.fetch_indexed_zero_page(self.reg.Y),
            AddressingMode::IndexedAbsoluteX => self.fetch_indexed_absolute(self.reg.X),
            AddressingMode::IndexedAbsoluteY => self.fetch_indexed_absolute(self.reg.Y),
            AddressingMode::Implied => Some(FetchedMemory {
                data: 0,
                address: 0,
                additional_cycles: 0,
            }),
            AddressingMode::Relative => self.fetch_relative(),
            AddressingMode::IndexedIndirect => self.fetch_indexed_indirect(),
            AddressingMode::IndirectIndexed => self.fetch_indirect_indexed(),
            AddressingMode::AbsoluteIndirect => self.fetch_absolute_indirect(),
        }
    }

    /// Push a single byte onto the stack, modifying the stack pointer.
    fn stack_push_byte(&mut self, data: u8) {
        self.bus.write_byte(self.reg.stack_pointer_address(), data);
        self.reg.S -= 1;
    }

    /// Push two bytes onto the stack, modifying the stack pointer.
    fn stack_push_word(&mut self, data: u16) {
        self.stack_push_byte(((data >> 8) & 0xFF) as u8);
        self.stack_push_byte((data & 0xFF) as u8);
    }

    /// Pop a single byte off the stack, modifying the stack pointer.
    fn stack_pop_byte(&mut self) -> u8 {
        self.reg.S += 1;
        match self.bus.read_byte(self.reg.stack_pointer_address()) {
            Some(data) => data,
            None => {
                self.faulted = true;
                0
            }
        }
    }

    /// Pop two bytes off of the stack, modifying the stack pointer.
    fn stack_pop_word(&mut self) -> u16 {
        let low_byte = self.stack_pop_byte();
        let high_byte = self.stack_pop_byte();
        ((high_byte as u16) << 8) | (low_byte as u16)
    }

    /// Return whether both of the specified memory addresses are in the same 256-byte page.
    fn same_pages(address1: u16, address2: u16) -> bool {
        (address1 & 0xFF00) == (address2 & 0xFF00)
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
    operation: InstructionFn,
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
#[rustfmt::skip]
const INSTRUCTIONS: [Instruction; 256] = [
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("ORA",  0x01, 6, IndexedIndirect,  instruction_ora),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*SLO", 0x03, 8, IndexedIndirect,  instruction_undocumented_slo),
    ins!("*NOP", 0x04, 3, ZeroPage,         instruction_nop),
    ins!("ORA",  0x05, 3, ZeroPage,         instruction_ora),
    ins!("ASL",  0x06, 5, ZeroPage,         instruction_asl),
    ins!("*SLO", 0x07, 5, ZeroPage,         instruction_undocumented_slo),
    ins!("PHP",  0x08, 3, Implied,          instruction_php),
    ins!("ORA",  0x09, 2, Immediate,        instruction_ora),
    ins!("ASL",  0x0A, 2, Accumulator,      instruction_asl),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*NOP", 0x0C, 4, Absolute,         instruction_nop),
    ins!("ORA",  0x0D, 4, Absolute,         instruction_ora),
    ins!("ASL",  0x0E, 6, Absolute,         instruction_asl),
    ins!("*SLO", 0x0F, 6, Absolute,         instruction_undocumented_slo),
    ins!("BPL",  0x10, 2, Relative,         instruction_bpl),
    ins!("ORA",  0x11, 5, IndirectIndexed,  instruction_ora),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*SLO", 0x13, 8, IndirectIndexed,  instruction_undocumented_slo),
    ins!("*NOP", 0x14, 4, IndexedZeroPageX, instruction_nop),
    ins!("ORA",  0x15, 4, IndexedZeroPageX, instruction_ora),
    ins!("ASL",  0x16, 6, IndexedZeroPageX, instruction_asl),
    ins!("*SLO", 0x17, 6, IndexedZeroPageX, instruction_undocumented_slo),
    ins!("CLC",  0x18, 2, Implied,          instruction_clc),
    ins!("ORA",  0x19, 4, IndexedAbsoluteY, instruction_ora),
    ins!("*NOP", 0x1A, 2, Implied,          instruction_nop),
    ins!("*SLO", 0x1B, 7, IndexedAbsoluteY, instruction_undocumented_slo),
    ins!("*NOP", 0x1C, 4, IndexedAbsoluteX, instruction_nop),
    ins!("ORA",  0x1D, 4, IndexedAbsoluteX, instruction_ora),
    ins!("ASL",  0x1E, 7, IndexedAbsoluteX, instruction_asl),
    ins!("*SLO", 0x1F, 7, IndexedAbsoluteX, instruction_undocumented_slo),
    ins!("JSR",  0x20, 6, Absolute,         instruction_jsr),
    ins!("AND",  0x21, 6, IndexedIndirect,  instruction_and),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*RLA", 0x23, 8, IndexedIndirect,  instruction_undocumented_rla),
    ins!("BIT",  0x24, 3, ZeroPage,         instruction_bit),
    ins!("AND",  0x25, 3, ZeroPage,         instruction_and),
    ins!("ROL",  0x26, 5, ZeroPage,         instruction_rol),
    ins!("*RLA", 0x27, 5, ZeroPage,         instruction_undocumented_rla),
    ins!("PLP",  0x28, 4, Implied,          instruction_plp),
    ins!("AND",  0x29, 2, Immediate,        instruction_and),
    ins!("ROL",  0x2A, 2, Accumulator,      instruction_rol),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("BIT",  0x2C, 4, Absolute,         instruction_bit),
    ins!("AND",  0x2D, 4, Absolute,         instruction_and),
    ins!("ROL",  0x2E, 6, Absolute,         instruction_rol),
    ins!("*RLA", 0x2F, 6, Absolute,         instruction_undocumented_rla),
    ins!("BMI",  0x30, 2, Relative,         instruction_bmi),
    ins!("AND",  0x31, 5, IndirectIndexed,  instruction_and),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*RLA", 0x33, 8, IndirectIndexed,  instruction_undocumented_rla),
    ins!("*NOP", 0x34, 4, IndexedZeroPageX, instruction_nop),
    ins!("AND",  0x35, 4, IndexedZeroPageX, instruction_and),
    ins!("ROL",  0x36, 6, IndexedZeroPageX, instruction_rol),
    ins!("*RLA", 0x37, 6, IndexedZeroPageX, instruction_undocumented_rla),
    ins!("SEC",  0x38, 2, Implied,          instruction_sec),
    ins!("AND",  0x39, 4, IndexedAbsoluteY, instruction_and),
    ins!("*NOP", 0x3A, 2, Implied,          instruction_nop),
    ins!("*RLA", 0x3B, 7, IndexedAbsoluteY, instruction_undocumented_rla),
    ins!("*NOP", 0x3C, 4, IndexedAbsoluteX, instruction_nop),
    ins!("AND",  0x3D, 4, IndexedAbsoluteX, instruction_and),
    ins!("ROL",  0x3E, 7, IndexedAbsoluteX, instruction_rol),
    ins!("*RLA", 0x3F, 7, IndexedAbsoluteX, instruction_undocumented_rla),
    ins!("RTI",  0x40, 6, Implied,          instruction_rti),
    ins!("EOR",  0x41, 6, IndexedIndirect,  instruction_eor),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*SRE", 0x43, 8, IndexedIndirect,  instruction_undocumented_sre),
    ins!("*NOP", 0x44, 3, ZeroPage,         instruction_nop),
    ins!("EOR",  0x45, 3, ZeroPage,         instruction_eor),
    ins!("LSR",  0x46, 5, ZeroPage,         instruction_lsr),
    ins!("*SRE", 0x47, 5, ZeroPage,         instruction_undocumented_sre),
    ins!("PHA",  0x48, 3, Implied,          instruction_pha),
    ins!("EOR",  0x49, 2, Immediate,        instruction_eor),
    ins!("LSR",  0x4A, 2, Accumulator,      instruction_lsr),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("JMP",  0x4C, 3, Absolute,         instruction_jmp),
    ins!("EOR",  0x4D, 4, Absolute,         instruction_eor),
    ins!("LSR",  0x4E, 6, Absolute,         instruction_lsr),
    ins!("*SRE", 0x4F, 6, Absolute,         instruction_undocumented_sre),
    ins!("BVC",  0x50, 2, Relative,         instruction_bvc),
    ins!("EOR",  0x51, 5, IndirectIndexed,  instruction_eor),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*SRE", 0x53, 8, IndirectIndexed,  instruction_undocumented_sre),
    ins!("*NOP", 0x54, 4, IndexedZeroPageX, instruction_nop),
    ins!("EOR",  0x55, 4, IndexedZeroPageX, instruction_eor),
    ins!("LSR",  0x56, 6, IndexedZeroPageX, instruction_lsr),
    ins!("*SRE", 0x57, 6, IndexedZeroPageX, instruction_undocumented_sre),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("EOR",  0x59, 4, IndexedAbsoluteY, instruction_eor),
    ins!("*NOP", 0x5A, 2, Implied,          instruction_nop),
    ins!("*SRE", 0x5B, 7, IndexedAbsoluteY, instruction_undocumented_sre),
    ins!("*NOP", 0x5C, 4, IndexedAbsoluteX, instruction_nop),
    ins!("EOR",  0x5D, 4, IndexedAbsoluteX, instruction_eor),
    ins!("LSR",  0x5E, 7, IndexedAbsoluteX, instruction_lsr),
    ins!("*SRE", 0x5F, 7, IndexedAbsoluteX, instruction_undocumented_sre),
    ins!("RTS",  0x60, 6, Implied,          instruction_rts),
    ins!("ADC",  0x61, 6, IndexedIndirect,  instruction_adc),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*NOP", 0x64, 3, ZeroPage,         instruction_nop),
    ins!("ADC",  0x65, 3, ZeroPage,         instruction_adc),
    ins!("ROR",  0x66, 5, ZeroPage,         instruction_ror),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("PLA",  0x68, 4, Implied,          instruction_pla),
    ins!("ADC",  0x69, 2, Immediate,        instruction_adc),
    ins!("ROR",  0x6A, 2, Accumulator,      instruction_ror),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("JMP",  0x6C, 5, AbsoluteIndirect, instruction_jmp),
    ins!("ADC",  0x6D, 4, Absolute,         instruction_adc),
    ins!("ROR",  0x6E, 6, Absolute,         instruction_ror),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("BVS",  0x70, 2, Relative,         instruction_bvs),
    ins!("ADC",  0x71, 5, IndirectIndexed,  instruction_adc),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*NOP", 0x74, 4, IndexedZeroPageX, instruction_nop),
    ins!("ADC",  0x75, 4, IndexedZeroPageX, instruction_adc),
    ins!("ROR",  0x76, 6, IndexedZeroPageX, instruction_ror),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("SEI",  0x78, 2, Implied,          instruction_sei),
    ins!("ADC",  0x79, 4, IndexedAbsoluteY, instruction_adc),
    ins!("*NOP", 0x7A, 2, Implied,          instruction_nop),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*NOP", 0x7C, 4, IndexedAbsoluteX, instruction_nop),
    ins!("ADC",  0x7D, 4, IndexedAbsoluteX, instruction_adc),
    ins!("ROR",  0x7E, 7, IndexedAbsoluteX, instruction_ror),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*NOP", 0x80, 2, Immediate,        instruction_nop),
    ins!("STA",  0x81, 6, IndexedIndirect,  instruction_sta),
    ins!("*NOP", 0x82, 2, Immediate,        instruction_nop),
    ins!("*SAX", 0x83, 6, IndexedIndirect,  instruction_undocumented_sax),
    ins!("STY",  0x84, 3, ZeroPage,         instruction_sty),
    ins!("STA",  0x85, 3, ZeroPage,         instruction_sta),
    ins!("STX",  0x86, 3, ZeroPage,         instruction_stx),
    ins!("*SAX", 0x87, 3, ZeroPage,         instruction_undocumented_sax),
    ins!("DEY",  0x88, 2, Implied,          instruction_dey),
    ins!("*NOP", 0x89, 2, Immediate,        instruction_nop),
    ins!("TXA",  0x8A, 2, Implied,          instruction_txa),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("STY",  0x8C, 4, Absolute,         instruction_sty),
    ins!("STA",  0x8D, 4, Absolute,         instruction_sta),
    ins!("STX",  0x8E, 4, Absolute,         instruction_stx),
    ins!("*SAX", 0x8F, 4, Absolute,         instruction_undocumented_sax),
    ins!("BCC",  0x90, 2, Relative,         instruction_bcc),
    ins!("STA",  0x91, 6, IndirectIndexed,  instruction_sta),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("STY",  0x94, 4, IndexedZeroPageX, instruction_sty),
    ins!("STA",  0x95, 4, IndexedZeroPageX, instruction_sta),
    ins!("STX",  0x96, 4, IndexedZeroPageY, instruction_stx),
    ins!("*SAX", 0x97, 4, IndexedZeroPageY, instruction_undocumented_sax),
    ins!("TYA",  0x98, 2, Implied,          instruction_tya),
    ins!("STA",  0x99, 5, IndexedAbsoluteY, instruction_sta),
    ins!("TXS",  0x9A, 2, Implied,          instruction_txs),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("STA",  0x9D, 5, IndexedAbsoluteX, instruction_sta),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("LDY",  0xA0, 2, Immediate,        instruction_ldy),
    ins!("LDA",  0xA1, 6, IndexedIndirect,  instruction_lda),
    ins!("LDX",  0xA2, 2, Immediate,        instruction_ldx),
    ins!("*LAX", 0xA3, 6, IndexedIndirect,  instruction_undocumented_lax),
    ins!("LDY",  0xA4, 3, ZeroPage,         instruction_ldy),
    ins!("LDA",  0xA5, 3, ZeroPage,         instruction_lda),
    ins!("LDX",  0xA6, 3, ZeroPage,         instruction_ldx),
    ins!("*LAX", 0xA7, 3, ZeroPage,         instruction_undocumented_lax),
    ins!("TAY",  0xA8, 2, Implied,          instruction_tay),
    ins!("LDA",  0xA9, 2, Immediate,        instruction_lda),
    ins!("TAX",  0xAA, 2, Implied,          instruction_tax),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("LDY",  0xAC, 4, Absolute,         instruction_ldy),
    ins!("LDA",  0xAD, 4, Absolute,         instruction_lda),
    ins!("LDX",  0xAE, 4, Absolute,         instruction_ldx),
    ins!("*LAX", 0xAF, 4, Absolute,         instruction_undocumented_lax),
    ins!("BCS",  0xB0, 2, Relative,         instruction_bcs),
    ins!("LDA",  0xB1, 5, IndirectIndexed,  instruction_lda),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*LAX", 0xB3, 5, IndirectIndexed,  instruction_undocumented_lax),
    ins!("LDY",  0xB4, 4, IndexedZeroPageX, instruction_ldy),
    ins!("LDA",  0xB5, 4, IndexedZeroPageX, instruction_lda),
    ins!("LDX",  0xB6, 4, IndexedZeroPageY, instruction_ldx),
    ins!("*LAX", 0xB7, 4, IndexedZeroPageY, instruction_undocumented_lax),
    ins!("CLV",  0xB8, 2, Implied,          instruction_clv),
    ins!("LDA",  0xB9, 4, IndexedAbsoluteY, instruction_lda),
    ins!("TSX",  0xBA, 2, Implied,          instruction_tsx),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("LDY",  0xBC, 4, IndexedAbsoluteX, instruction_ldy),
    ins!("LDA",  0xBD, 4, IndexedAbsoluteX, instruction_lda),
    ins!("LDX",  0xBE, 4, IndexedAbsoluteY, instruction_ldx),
    ins!("*LAX", 0xBF, 4, IndexedAbsoluteY, instruction_undocumented_lax),
    ins!("CPY",  0xC0, 2, Immediate,        instruction_cpy),
    ins!("CMP",  0xC1, 6, IndexedIndirect,  instruction_cmp),
    ins!("*NOP", 0xC2, 2, Immediate,        instruction_nop),
    ins!("*DCP", 0xC3, 8, IndexedIndirect,  instruction_undocumented_dcp),
    ins!("CPY",  0xC4, 3, ZeroPage,         instruction_cpy),
    ins!("CMP",  0xC5, 3, ZeroPage,         instruction_cmp),
    ins!("DEC",  0xC6, 5, ZeroPage,         instruction_dec),
    ins!("*DCP", 0xC7, 5, ZeroPage,         instruction_undocumented_dcp),
    ins!("INY",  0xC8, 2, Implied,          instruction_iny),
    ins!("CMP",  0xC9, 2, Immediate,        instruction_cmp),
    ins!("DEX",  0xCA, 2, Implied,          instruction_dex),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("CPY",  0xCC, 4, Absolute,         instruction_cpy),
    ins!("CMP",  0xCD, 4, Absolute,         instruction_cmp),
    ins!("DEC",  0xCE, 6, Absolute,         instruction_dec),
    ins!("*DCP", 0xCF, 6, Absolute,         instruction_undocumented_dcp),
    ins!("BNE",  0xD0, 2, Relative,         instruction_bne),
    ins!("CMP",  0xD1, 5, IndirectIndexed,  instruction_cmp),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*DCP", 0xD3, 8, IndirectIndexed,  instruction_undocumented_dcp),
    ins!("*NOP", 0xD4, 4, IndexedZeroPageX, instruction_nop),
    ins!("CMP",  0xD5, 4, IndexedZeroPageX, instruction_cmp),
    ins!("DEC",  0xD6, 6, IndexedZeroPageX, instruction_dec),
    ins!("*DCP", 0xD7, 6, IndexedZeroPageX, instruction_undocumented_dcp),
    ins!("CLD",  0xD8, 2, Implied,          instruction_cld),
    ins!("CMP",  0xD9, 4, IndexedAbsoluteY, instruction_cmp),
    ins!("*NOP", 0xDA, 2, Implied,          instruction_nop),
    ins!("*DCP", 0xDB, 7, IndexedAbsoluteY, instruction_undocumented_dcp),
    ins!("*NOP", 0xDC, 4, IndexedAbsoluteX, instruction_nop),
    ins!("CMP",  0xDD, 4, IndexedAbsoluteX, instruction_cmp),
    ins!("DEC",  0xDE, 7, IndexedAbsoluteX, instruction_dec),
    ins!("*DCP", 0xDF, 7, IndexedAbsoluteX, instruction_undocumented_dcp),
    ins!("CPX",  0xE0, 2, Immediate,        instruction_cpx),
    ins!("SBC",  0xE1, 6, IndexedIndirect,  instruction_sbc),
    ins!("*NOP", 0xE2, 2, Immediate,        instruction_nop),
    ins!("*ISB", 0xE3, 8, IndexedIndirect,  instruction_undocumented_isb),
    ins!("CPX",  0xE4, 3, ZeroPage,         instruction_cpx),
    ins!("SBC",  0xE5, 3, ZeroPage,         instruction_sbc),
    ins!("INC",  0xE6, 5, ZeroPage,         instruction_inc),
    ins!("*ISB", 0xE7, 5, ZeroPage,         instruction_undocumented_isb),
    ins!("INX",  0xE8, 2, Implied,          instruction_inx),
    ins!("SBC",  0xE9, 2, Immediate,        instruction_sbc),
    ins!("NOP",  0xEA, 2, Implied,          instruction_nop),
    ins!("SBC",  0xEB, 2, Immediate,        instruction_sbc),
    ins!("CPX",  0xEC, 4, Absolute,         instruction_cpx),
    ins!("SBC",  0xED, 4, Absolute,         instruction_sbc),
    ins!("INC",  0xEE, 6, Absolute,         instruction_inc),
    ins!("*ISB", 0xEF, 6, Absolute,         instruction_undocumented_isb),
    ins!("BEQ",  0xF0, 2, Relative,         instruction_beq),
    ins!("SBC",  0xF1, 5, IndirectIndexed,  instruction_sbc),
    ins!("UUU",  0x00, 1, Absolute,         unimplemented_instruction),
    ins!("*ISB", 0xF3, 8, IndirectIndexed,  instruction_undocumented_isb),
    ins!("*NOP", 0xF4, 4, IndexedZeroPageX, instruction_nop),
    ins!("SBC",  0xF5, 4, IndexedZeroPageX, instruction_sbc),
    ins!("INC",  0xF6, 6, IndexedZeroPageX, instruction_inc),
    ins!("*ISB", 0xF7, 6, IndexedZeroPageX, instruction_undocumented_isb),
    ins!("SED",  0xF8, 2, Implied,          instruction_sed),
    ins!("SBC",  0xF9, 4, IndexedAbsoluteY, instruction_sbc),
    ins!("*NOP", 0xFA, 2, Implied,          instruction_nop),
    ins!("*ISB", 0xFB, 7, IndexedAbsoluteY, instruction_undocumented_isb),
    ins!("*NOP", 0xFC, 4, IndexedAbsoluteX, instruction_nop),
    ins!("SBC",  0xFD, 4, IndexedAbsoluteX, instruction_sbc),
    ins!("INC",  0xFE, 7, IndexedAbsoluteX, instruction_inc),
    ins!("*ISB", 0xFF, 7, IndexedAbsoluteX, instruction_undocumented_isb),
];

impl CPU {
    /// Fetch the byte at the current program counter.  No further memory access is performed.
    fn fetch_immediate(&mut self) -> Option<FetchedMemory> {
        let data = self.bus.read_byte(self.reg.PC)?;
        self.reg.PC += 1;
        Some(FetchedMemory {
            data,
            address: self.reg.PC - 1,
            additional_cycles: 0,
        })
    }

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

    /// Fetch a byte from memory using zero-page addressing, which uses the current byte at the
    /// program counter as the low byte of the memory address to fetch and sets the high byte to 0.
    /// Thus, the range of accessible addresses is effectively limited to page 0.
    fn fetch_zero_page(&mut self) -> Option<FetchedMemory> {
        let address = self.bus.read_byte(self.reg.PC)? as u16;
        self.reg.PC += 1;
        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles: 0,
        })
    }

    /// Fetch a byte from memory using indexed zero-page addressing, which fetches a single byte at
    /// the program counter, adds either the X or Y index register to it without carry, and uses the
    /// sum to determine the address at which to read.  Due to the addition being performed without
    /// carry, addresses are limited to the zero page.
    fn fetch_indexed_zero_page(&mut self, index_reg: u8) -> Option<FetchedMemory> {
        // Determine the address, limited to 8 bits
        let offset = self.bus.read_byte(self.reg.PC)?;
        self.reg.PC += 1;
        let address = offset.wrapping_add(index_reg) as u16;

        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles: 0,
        })
    }

    /// Fetch a byte from memory using indexed absolute addressing, which fetches two bytes at the
    /// program counter and adds either the X or Y index register to this value to determine the
    /// address at which to read.
    fn fetch_indexed_absolute(&mut self, index_reg: u8) -> Option<FetchedMemory> {
        // Compute the address from which to read
        let absolute_address = self.bus.read_word(self.reg.PC)?;
        self.reg.PC += 2;
        let address = absolute_address.wrapping_add(index_reg as u16);

        // A 1-cycle penalty is incurred if adding the index caused the page boundary to be crossed
        let additional_cycles = if CPU::same_pages(absolute_address, address) {
            0
        } else {
            1
        };

        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles,
        })
    }

    /// Fetch a byte from memory using relative addressing, in which the current byte at the program
    /// counter is interpreted as an 8-bit signed integer and added to the program counter to
    /// determine the address from which to read.
    fn fetch_relative(&mut self) -> Option<FetchedMemory> {
        let offset = self.bus.read_byte(self.reg.PC)? as i8;
        self.reg.PC += 1;
        let address = ((self.reg.PC as i32) + (offset as i32)) as u16;
        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles: 0,
        })
    }

    /// Fetch a byte from memory using indexed indirect addressing [(IND, X)], in which the current
    /// byte at the program counter is added to index register X, the result of which is truncated
    /// to a zero-page address from which two bytes may be fetched to find the memory address from
    /// which to fetch data.
    fn fetch_indexed_indirect(&mut self) -> Option<FetchedMemory> {
        // Compute address of indirect address
        let offset = self.bus.read_byte(self.reg.PC)? as i8;
        self.reg.PC += 1;
        let addr_address = ((self.reg.X as i32) + (offset as i32)) as u16;

        // Simulate address wrap-around
        let low_byte = self.bus.read_byte(addr_address & 0xFF)? as u16;
        let high_byte = self.bus.read_byte(addr_address.wrapping_add(1) & 0xFF)? as u16;
        let address = (high_byte << 8) | low_byte;

        // Fetch the actual data
        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles: 0,
        })
    }

    /// Fetch a byte from memory using indirect indexed addressing [(IND), Y], in which the current
    /// byte at the program counter is used to index into the zero page, from which a byte is
    /// fetched and added to index register Y to form the low-order byte of the address from which
    /// to fetch data.  The high-order byte is comprised of the sum of the carry from the low-order
    /// addition and the following zero-page byte.
    fn fetch_indirect_indexed(&mut self) -> Option<FetchedMemory> {
        // Get offset from zero page
        let zp_offset = self.bus.read_byte(self.reg.PC)? as u16;
        self.reg.PC += 1;
        // Construct read address
        let low_byte = self.bus.read_byte(zp_offset & 0xFF)? as u16;
        let high_byte = self.bus.read_byte(zp_offset.wrapping_add(1) & 0xFF)? as u16;
        let address = ((high_byte << 8) | low_byte).wrapping_add(self.reg.Y as u16);

        // If the offset carried the read to a different page, a 1-cycle penalty is incurred
        let additional_cycles = if CPU::same_pages(address, high_byte << 8) {
            0
        } else {
            1
        };

        // Fetch data from memory
        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles,
        })
    }

    /// Fetch a byte from memory using absolute indirect addressing, in which two bytes at the
    /// program counter constitute an address from which to read the address at which the actual
    /// memory fetch occurs.  Note that this addressing mode is subject to a hardware bug in which
    /// attempting to read the address across pages will wrap the high-order bit around to the
    /// beginning of the same page as the low-order bit, which comes first in memory given the
    /// little-endian architecture of the 6502.
    fn fetch_absolute_indirect(&mut self) -> Option<FetchedMemory> {
        // Get the address from which to fetch
        let indirect_addr = self.bus.read_word(self.reg.PC)?;
        self.reg.PC += 2;

        // Simulate wrap-around bug
        let address = if (indirect_addr & 0xFF) == 0xFF {
            let low_byte = self.bus.read_byte(indirect_addr)? as u16;
            let high_byte = self.bus.read_byte(indirect_addr & 0xFF00)? as u16;
            (high_byte << 8) | low_byte
        } else {
            self.bus.read_word(indirect_addr)?
        };

        // No instruction actually uses this data, but fetch it for debugging
        let data = self.bus.read_byte(address)?;
        Some(FetchedMemory {
            data,
            address,
            additional_cycles: 0,
        })
    }

    /// Execute a conditional branch to the specified address, based on whether the specified status
    /// flag is set to the specified value.
    fn conditional_branch(&mut self, flag: StatusFlag, value: bool, address: u16) {
        if self.reg.get_status_flag(flag) == value {
            // Branch penalty of 1 cycle
            self.cycles_remaining += 1;
            if !CPU::same_pages(self.reg.PC, address) {
                // Additional cycle penalty if the branch occurred to a different page
                self.cycles_remaining += 1;
            }

            // Execute the branch
            self.reg.PC = address;
        }
    }

    /// Set the value-dependent flags (Negative and Zero) based on the specified value.
    fn set_value_status(&mut self, value: u8) {
        self.reg
            .set_status_flag(StatusFlag::Negative, (value & 0x80) != 0);
        self.reg.set_status_flag(StatusFlag::Zero, value == 0);
    }

    /// Write the specified value back to either the accumulator or memory, depending on the
    /// addressing mode of the instruction.
    fn accumulator_write_back(&mut self, opcode: u8, value: u8, address: u16) {
        match INSTRUCTIONS[opcode as usize].addressing_mode {
            AddressingMode::Accumulator => self.reg.A = value,
            _ => {
                self.bus.write_byte(address, value);
            }
        }
    }

    /// Perform an undocumented "combined instruction": that is, an instruction that essentially
    /// mimics two separate instructions running independently with the same operands.
    fn combined_instruction(
        &mut self,
        opcode: u8,
        first: InstructionFn,
        second: InstructionFn,
        fetched: &FetchedMemory,
    ) {
        // Run the first instruction
        (first)(self, opcode, &fetched);

        // Fetch memory to feed into second instruction
        let new_value = self.bus.read_byte(fetched.address).unwrap();
        // Run the second instruction with the result of the first
        (second)(
            self,
            opcode,
            &FetchedMemory {
                data: new_value,
                address: fetched.address,
                additional_cycles: 0,
            },
        );
    }

    /// `ADC` instruction.  Adds together the value in the accumulator, a value from memory, and the
    /// carry flag, allowing for chained addition of values greater than 8 bits.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Overflow
    /// - Zero
    fn instruction_adc(&mut self, opcode: u8, fetched: &FetchedMemory) {
        // Cast to 16 bits to allow inspection of carry
        let reg_a = self.reg.A as u16;
        let operand = fetched.data as u16;
        let carry = self.reg.get_status_flag(StatusFlag::Carry) as u16;

        // Perform initial addition as normal
        let sum = reg_a.wrapping_add(operand).wrapping_add(carry);
        self.set_value_status(sum as u8);
        // Carry is set if bit 9 of the sum is set
        self.reg.set_status_flag(StatusFlag::Carry, sum > 0xFF);
        // Signed overflow (thanks, bits.c!)
        let overflow = (((reg_a ^ operand) & 0x80) == 0) && (((reg_a ^ sum) & 0x80) != 0);
        self.reg.set_status_flag(StatusFlag::Overflow, overflow);

        // Set the actual sum
        self.reg.A = sum as u8;
    }

    /// `AND` instruction.  Performs a bitwise AND operation on the value in the accumulator and a
    /// value from memory, storing the result in the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_and(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = self.reg.A & fetched.data;
        self.set_value_status(data);
        self.reg.A = data;
    }

    /// `ASL` instruction.  Perform an arithmetic shift one bit to the left on the operand.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_asl(&mut self, opcode: u8, fetched: &FetchedMemory) {
        // Perform the shift
        let shifted = fetched.data.wrapping_shl(1);
        self.set_value_status(shifted);
        self.reg
            .set_status_flag(StatusFlag::Carry, (fetched.data & 0x80) != 0);

        self.accumulator_write_back(opcode, shifted, fetched.address);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `BCC` instruction.  Branch to a relative memory address if the Carry flag is not set.
    ///
    /// Flags modified: *None*
    fn instruction_bcc(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Carry, false, fetched.address);
    }

    /// `BCS` instruction.  Branch to a relative memory address if the Carry flag is set.
    ///
    /// Flags modified: *None*
    fn instruction_bcs(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Carry, true, fetched.address);
    }

    /// `BEQ` instruction.  Branch to a relative memory address if the Zero flag is set.
    ///
    /// Flags modified: *None*
    fn instruction_beq(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Zero, true, fetched.address);
    }

    /// `BIT` instruction.  Test bits in memory with accumulator.  Somewhat akin to a test-and-set
    /// instruction, and can be used for I/O synchronisation.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    /// - Overflow
    fn instruction_bit(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let operand = fetched.data;
        // Bits 7 and 6 of operands are transferred to (N,V)
        self.reg
            .set_status_flag(StatusFlag::Negative, ((operand >> 7) & 1) == 1);
        self.reg
            .set_status_flag(StatusFlag::Overflow, ((operand >> 6) & 1) == 1);
        self.reg
            .set_status_flag(StatusFlag::Zero, (self.reg.A & operand) == 0);
    }

    /// `BMI` instruction.  Branch to a relative memory address if the Negative flag is set.
    ///
    /// Flags modified: *None*
    fn instruction_bmi(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Negative, true, fetched.address);
    }

    /// `BNE` instruction.  Branch to a relative memory address if the Zero flag is not set.
    ///
    /// Flags modified: *None*
    fn instruction_bne(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Zero, false, fetched.address);
    }

    /// `BPL` instruction.  Branch to a relative memory address if the Negative flag is not set.
    ///
    /// Flags modified: *None*
    fn instruction_bpl(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Negative, false, fetched.address);
    }

    /// `BVC` instruction.  Branch to a relative memory address if the Overflow flag is not set.
    ///
    /// Flags modified: *None*
    fn instruction_bvc(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Overflow, false, fetched.address);
    }

    /// `BVS` instruction.  Branch to a relative memory address if the Overflow flag is set.
    ///
    /// Flags modified: *None*
    fn instruction_bvs(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.conditional_branch(StatusFlag::Overflow, true, fetched.address);
    }

    /// `CLC` instruction.  Sets the carry flag low.
    ///
    /// Flags modified: Carry
    fn instruction_clc(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.set_status_flag(StatusFlag::Carry, false);
    }

    /// `CLD` instruction.  Sets the decimal mode flag low.
    ///
    /// Flags modified: Decimal mode
    fn instruction_cld(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.set_status_flag(StatusFlag::DecimalMode, false);
    }

    /// `CLD` instruction.  Sets the overflow flag low.
    ///
    /// Flags modified: Overflow
    fn instruction_clv(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.set_status_flag(StatusFlag::Overflow, false);
    }

    /// Perform a comparison operation upon the specified values, setting flags as necessary.
    fn perform_compare(&mut self, value1: u8, value2: u8) {
        let difference = value1.wrapping_sub(value2);
        self.set_value_status(difference);
        self.reg
            .set_status_flag(StatusFlag::Carry, value1 >= value2);
    }

    /// `CMP` instruction.  Compares the value in the accumulator with a value in memory by way of
    /// subtraction.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_cmp(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.perform_compare(self.reg.A, fetched.data);
    }

    /// `CPX` instruction.  Compares the value in index register X with a value in memory by way of
    /// subtraction.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_cpx(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.perform_compare(self.reg.X, fetched.data);
    }

    /// `CPY` instruction.  Compares the value in index register Y with a value in memory by way of
    /// subtraction.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_cpy(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.perform_compare(self.reg.Y, fetched.data);
    }

    /// Undocumented `DCP` instruction.  Decrements a value in memory by 1 (like `DEC`) and compares
    /// it to the accumulator (like `CMP`).  This instruction is useful because it provides some
    /// additional addressing modes over `DEC`, which has a fairly limited set of variants.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_undocumented_dcp(&mut self, opcode: u8, fetched: &FetchedMemory) {
        // Perform the DEC portion
        self.instruction_dec(opcode, &fetched);
        // Perform the CMP portion
        let new_value = self.bus.read_byte(fetched.address).unwrap();
        self.perform_compare(self.reg.A, new_value);
    }

    /// `DEC` instruction.  Decrement a value in memory by 1.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_dec(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let difference = fetched.data.wrapping_sub(1);
        self.set_value_status(difference);
        self.bus.write_byte(fetched.address, difference);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `DEX` instruction.  Decrement the value in the X index register by 1.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_dex(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let difference = self.reg.X.wrapping_sub(1);
        self.set_value_status(difference);
        self.reg.X = difference;
    }

    /// `DEY` instruction.  Decrement the value in the Y index register by 1.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_dey(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let difference = self.reg.Y.wrapping_sub(1);
        self.set_value_status(difference);
        self.reg.Y = difference;
    }

    /// `EOR` instruction.  Performs a bitwise exclusive OR (XOR) operation on the value in the
    /// accumulator and a value from memory, storing the result in the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_eor(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = self.reg.A ^ fetched.data;
        self.set_value_status(data);
        self.reg.A = data;
    }

    /// `INC` instruction.  Increment a value in memory by 1.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_inc(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let sum = fetched.data.wrapping_add(1);
        self.set_value_status(sum);
        self.bus.write_byte(fetched.address, sum);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `INX` instruction.  Increment the value in the X index register by 1.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_inx(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let sum = self.reg.X.wrapping_add(1);
        self.set_value_status(sum);
        self.reg.X = sum;
    }

    /// `INY` instruction.  Increment the value in the Y index register by 1.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_iny(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let sum = self.reg.Y.wrapping_add(1);
        self.set_value_status(sum);
        self.reg.Y = sum;
    }

    /// Undocumented `ISB` instruction.  Increments a value in memory by 1 (like `INC`) then
    /// performs a `SBC` on the result, subtracting from the accumulator.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Overflow
    /// - Zero
    fn instruction_undocumented_isb(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.combined_instruction(opcode, CPU::instruction_inc, CPU::instruction_sbc, fetched);
    }

    /// `JMP` instruction.  Unconditionally branches to the specified memory address.
    ///
    /// Flags modified: *None*
    fn instruction_jmp(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.PC = fetched.address;
    }

    /// `JSR` instruction.  Unconditionally branches to the specified memory address, storing the
    /// address of the next instruction onto the stack.
    ///
    /// Flags modified: *None*
    fn instruction_jsr(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.stack_push_word(self.reg.PC - 1);
        self.reg.PC = fetched.address;
    }

    /// Undocumented `LAX` instruction.  Loads a value into both the accumulator and index register
    /// X.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_undocumented_lax(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = fetched.data;
        self.reg.A = data;
        self.reg.X = data;
        self.set_value_status(data);
    }

    /// `LDA` instruction.  Loads a value into the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_lda(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = fetched.data;
        self.reg.A = data;
        self.set_value_status(data);
    }

    /// `LDX` instruction.  Loads a value into the X index register.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_ldx(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = fetched.data;
        self.reg.X = data;
        self.set_value_status(data);
    }

    /// `LDY` instruction.  Loads a value into the Y index register.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_ldy(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = fetched.data;
        self.reg.Y = data;
        self.set_value_status(data);
    }

    /// `LSR` instruction.  Logical shift the operand value one bit to the right.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_lsr(&mut self, opcode: u8, fetched: &FetchedMemory) {
        // Perform the shift
        self.reg
            .set_status_flag(StatusFlag::Carry, (fetched.data & 1) == 1);
        let shifted = fetched.data >> 1;
        self.set_value_status(shifted);

        self.accumulator_write_back(opcode, shifted, fetched.address);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `NOP` instruction.  As the name implies, performs no operation.
    ///
    /// Flags modified: *None*
    fn instruction_nop(&mut self, opcode: u8, fetched: &FetchedMemory) {}

    /// `ORA` instruction.  Performs a bitwise OR operation on the value in the accumulator and a
    /// value from memory, storing the result in the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_ora(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = self.reg.A | fetched.data;
        self.set_value_status(data);
        self.reg.A = data;
    }

    /// `PHA` instruction.  Push the accumulator register (`A`) onto the stack.
    ///
    /// Flags modified: *None*
    fn instruction_pha(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.stack_push_byte(self.reg.A);
    }

    /// `PHP` instruction.  Push the processor status register (`P`) onto the stack.
    ///
    /// Flags modified: *None*
    fn instruction_php(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.stack_push_byte(self.reg.P);
    }

    /// `PLA` instruction.  Pop the value at the top of the stack into the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_pla(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = self.stack_pop_byte();
        self.set_value_status(data);
        self.reg.A = data;
    }

    /// `PLP` instruction.  Pop the value at the top of the stack into the processor status
    /// register (`P`).
    ///
    /// Flags modified: All flags set from stack
    fn instruction_plp(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.P = self.stack_pop_byte();
        // Always set the unused flag high
        self.reg.set_status_flag(StatusFlag::Unused, true);
    }

    /// Undocumented `RLA` instruction.  Perform a `ROL` of a memory value one bit to the left, then
    /// `AND` the result with the value in the accumulator.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_undocumented_rla(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.combined_instruction(opcode, CPU::instruction_rol, CPU::instruction_and, fetched);
    }

    /// `ROL` instruction.  Rotate the operand one bit to the left, wrapping the truncated bit
    /// around to the least significant bit of the result.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_rol(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let prior_carry = self.reg.get_status_flag(StatusFlag::Carry) as u8;
        let rotated = (fetched.data << 1) | prior_carry;
        self.set_value_status(rotated);
        self.reg
            .set_status_flag(StatusFlag::Carry, (fetched.data & 0x80) != 0);

        self.accumulator_write_back(opcode, rotated, fetched.address);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `ROR` instruction.  Rotate the operand one bit to the right, wrapping the truncated bit
    /// around to the most significant bit of the result.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_ror(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let prior_carry = self.reg.get_status_flag(StatusFlag::Carry) as u8;
        let rotated = (fetched.data >> 1) | (prior_carry << 7);
        self.set_value_status(rotated);
        self.reg
            .set_status_flag(StatusFlag::Carry, (fetched.data & 1) == 1);

        self.accumulator_write_back(opcode, rotated, fetched.address);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `RTI` instruction.  Used to return from an interrupt, pops the status register and program
    /// counter from the stack.
    ///
    /// Flags modified: All flags set from stack
    fn instruction_rti(&mut self, opcode: u8, fetched: &FetchedMemory) {
        // Get status register from stack
        self.reg.P = self.stack_pop_byte();
        self.reg.set_status_flag(StatusFlag::Unused, true);
        // Get the program counter
        self.reg.PC = self.stack_pop_word();
    }

    /// `RTS` instruction.  Return from subroutine.  Pops a 16-bit value off of the stack and jumps
    /// to the address it denotes.
    ///
    /// Flags modified: *None*
    fn instruction_rts(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.PC = self.stack_pop_word() + 1;
    }

    /// Undocumented `SAX` instruction.  Performs an AND on the accumulator and index register X,
    /// and store the result in memory.
    ///
    /// Flags modified: *None*
    fn instruction_undocumented_sax(&mut self, opcode: u8, fetched: &FetchedMemory) {
        let data = self.reg.A & self.reg.X;
        self.bus.write_byte(fetched.address, data);
    }

    /// `SBC` instruction.  Subtracts a value from memory from the value in the accumulator,
    /// borrowing a 1 in from the carry flag (1 - C).
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Overflow
    /// - Zero
    fn instruction_sbc(&mut self, opcode: u8, fetched: &FetchedMemory) {
        // Cast to 16 bits to allow inspection of carry
        let reg_a = self.reg.A as u16;
        let operand = fetched.data as u16;
        let carry = !self.reg.get_status_flag(StatusFlag::Carry) as u16;

        // Perform basic subtraction
        let difference = reg_a.wrapping_sub(operand).wrapping_sub(carry);
        self.set_value_status(difference as u8);
        self.reg
            .set_status_flag(StatusFlag::Carry, difference <= 0xFF);
        // Overflow flag
        let overflow = (((reg_a ^ operand) & 0x80) != 0) && (((reg_a ^ difference) & 0x80) != 0);
        self.reg.set_status_flag(StatusFlag::Overflow, overflow);

        // Set the actual difference
        self.reg.A = difference as u8;
    }

    /// `SEC` instruction.  Sets the carry flag high.
    ///
    /// Flags modified: Carry
    fn instruction_sec(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.set_status_flag(StatusFlag::Carry, true);
    }

    /// `SED` instruction.  Sets the decimal mode flag high.
    ///
    /// Flags modified: Decimal
    fn instruction_sed(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.set_status_flag(StatusFlag::DecimalMode, true);
    }

    /// `SEI` instruction.  Sets the interrupt disable flag high.
    ///
    /// Flags modified: Interrupt disable
    fn instruction_sei(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.set_status_flag(StatusFlag::IrqDisable, true);
    }

    /// Undocumented `SLO` instruction.  Perform a left shift (like `ASL`) on a value in memory,
    /// then performs an `ORA` on that shifted value and the accumulator.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_undocumented_slo(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.combined_instruction(opcode, CPU::instruction_asl, CPU::instruction_ora, fetched);
    }

    /// Undocumented `SRE` instruction.  Shifts a memory value rightward one bit (like `LSR`), then
    /// performs an `EOR` on that shifted value and the accumulator.
    ///
    /// Flags modified:
    /// - Carry
    /// - Negative
    /// - Zero
    fn instruction_undocumented_sre(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.combined_instruction(opcode, CPU::instruction_lsr, CPU::instruction_eor, fetched);
    }

    /// `STA` instruction.  Stores the accumulator into memory.
    ///
    /// Flags modified: *None*
    fn instruction_sta(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.bus.write_byte(fetched.address, self.reg.A);
        self.cycles_remaining -= fetched.additional_cycles as i8;
    }

    /// `STX` instruction.  Stores the X index register into memory.
    ///
    /// Flags modified: *None*
    fn instruction_stx(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.bus.write_byte(fetched.address, self.reg.X);
    }

    /// `STY` instruction.  Stores the Y index register into memory.
    ///
    /// Flags modified: *None*
    fn instruction_sty(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.bus.write_byte(fetched.address, self.reg.Y);
    }

    /// `TAX` instruction.  Transfer the value in the accumulator into index register X.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_tax(&mut self, opcode: u8, fetched: &FetchedMemory) {
        swap_registers!(self, self.reg.A, self.reg.X);
    }

    /// `TAY` instruction.  Transfer the value in the accumulator into index register Y.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_tay(&mut self, opcode: u8, fetched: &FetchedMemory) {
        swap_registers!(self, self.reg.A, self.reg.Y);
    }

    /// `TSX` instruction.  Transfer the stack pointer into index register X.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_tsx(&mut self, opcode: u8, fetched: &FetchedMemory) {
        swap_registers!(self, self.reg.S, self.reg.X);
    }

    /// `TXA` instruction.  Transfer the value in index register X into the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_txa(&mut self, opcode: u8, fetched: &FetchedMemory) {
        swap_registers!(self, self.reg.X, self.reg.A);
    }

    /// `TXS` instruction.  Transfer the value in index register X into the stack pointer.
    ///
    /// Flags modified: *None*
    fn instruction_txs(&mut self, opcode: u8, fetched: &FetchedMemory) {
        self.reg.S = self.reg.X;
    }

    /// `TYA` instruction.  Transfer the value in index register Y into the accumulator.
    ///
    /// Flags modified:
    /// - Negative
    /// - Zero
    fn instruction_tya(&mut self, opcode: u8, fetched: &FetchedMemory) {
        swap_registers!(self, self.reg.Y, self.reg.A);
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
                        if ui.button(im_str!("Pause execution"), [145.0, 18.0]) {
                            self.running = false;
                        }
                    } else {
                        if ui.button(im_str!("Resume execution"), [145.0, 18.0]) {
                            self.running = true;
                        }

                        if ui.button(im_str!("Step cycle"), [145.0, 18.0]) {
                            self.step_cycle();
                        }

                        if ui.button(im_str!("Step instruction"), [145.0, 18.0]) {
                            self.step_instruction();
                        }
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
