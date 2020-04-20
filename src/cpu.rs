use super::ui;
use imgui::Ui;
use std::fmt;

/// The value by which the 2A03 stack is offset from its actual contents.
const STACK_OFFSET: u16 = 0x100;

/// An implementation of the Ricoh 2A03 processor used by the NES,  a very slight variation of the
/// [MOS 6502](http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf) without decimal mode.
pub struct CPU {
    /// The programmer-accessible registers in the 2A03.
    reg: RegisterFile,
    /// The number of cycles remaining in the current instruction.
    cycles_remaining: u8,
    /// The cycle count after startup or the last reset.
    cycle: u64,
}

impl CPU {
    /// Create a new instance of the 2A03 CPU in the post-startup state.
    pub fn new() -> CPU {
        CPU {
            reg: RegisterFile::new(),
            cycles_remaining: 0,
            cycle: 0,
        }
    }
}

impl ui::Visualisable for CPU {
    fn display(&mut self, ui: &Ui) {
        self.reg.display(ui);
    }
}

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

        let base = self.vis.base;
        Window::new(im_str!("2A03 Registers"))
            .size([160.0, 185.0], Condition::Appearing)
            .position([10.0, 10.0], Condition::Appearing)
            .resizable(false)
            .build(ui, || {
                ui.text(format_register_value("A", self.A, 8, base));
                ui.text(format_register_value("Y", self.Y, 8, base));
                ui.text(format_register_value("X", self.X, 8, base));
                ui.separator();
                ui.text(format_register_value("PC", self.PC, 16, base));
                ui.text(format_register_value("S", self.S, 9, base));
                ui.separator();
                ui.text(format_register_value("P", self.P, 8, base));
                ui.text(im_str!("    NV BDIZC"));
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
