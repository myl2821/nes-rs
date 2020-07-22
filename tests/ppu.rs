use nes::{Bus, Cartridge, Mapper0, CPU, PPU};
use std::fs::File;
use std::path::Path;

use serde::Deserialize;
use std::cell::RefCell;
use std::rc::Rc;

#[test]
fn step() {
    let path = Path::new("tests/fixture/nestest.nes");
    let cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(cartridge);

    let cpu = Rc::new(RefCell::new(CPU::new()));
    let ppu = Rc::new(RefCell::new(PPU::new()));
    let bus = Rc::new(RefCell::new(Bus::new(mapper0, cpu.clone(), ppu.clone())));
    cpu.borrow_mut().connect_bus(bus.clone());
    ppu.borrow_mut().connect_bus(bus.clone());

    cpu.borrow_mut().reset();
    println!("{}", cpu.borrow().debug_info().0);

    for _n in 1..1000 {
        let cpu_cycles = cpu.borrow_mut().step();
        println!("{}", cpu.borrow().debug_info().0);
        for i in 0..(cpu_cycles * 3) {
            ppu.borrow_mut().step();
            let color = ppu.borrow().front;
            println!("{:?}", color)
        }
    }
}
