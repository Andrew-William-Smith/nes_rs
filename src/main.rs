use std::cell::RefCell;
use std::env;
use std::rc::Rc;

mod cartridge;
mod cpu;
mod memory;
mod system_bus;
mod ui;

fn main() {
    // Read the ROM file specified on the command line
    if let Some(rom_file) = env::args().nth(1) {
        // Initialise the CPU and read in the ROM
        let mut nes_cpu = cpu::CPU::new();
        nes_cpu.load_rom(&rom_file);

        // Kick off UI rendering
        let cpu_cell = Rc::new(RefCell::new(nes_cpu));
        ui::NesUi::run_loop(cpu_cell);
    } else {
        println!("Please specify a ROM file to run.");
    }
}
