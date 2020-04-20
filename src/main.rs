use std::cell::RefCell;
use std::rc::Rc;

mod cpu;
mod ui;

fn main() {
    let mut nes_cpu = Rc::new(RefCell::new(cpu::CPU::new()));
    ui::NesUi::run_loop(nes_cpu);
}
