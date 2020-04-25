use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{env, thread};

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
        let cpu_cell = Arc::new(Mutex::new(nes_cpu));

        // Kick off emulation
        let emu_cell = cpu_cell.clone();
        let emu_thread = thread::spawn(move || loop {
            let mut cpu = emu_cell.lock().unwrap();
            if cpu.is_running() {
                cpu.step_cycle();
            }
        });

        // Run UI thread and wait for emulation to exit
        ui::NesUi::run_loop(cpu_cell);
        emu_thread.join().unwrap();
    } else {
        println!("Please specify a ROM file to run.");
    }
}
