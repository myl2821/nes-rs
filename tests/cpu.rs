extern crate csv;

use nes::{Bus, Cartridge, Mapper0, CPU, PPU};
use std::fs::File;
use std::path::Path;

use serde::Deserialize;
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug, Deserialize, PartialEq)]
struct Status {
    addr: String,
    ins: String,
    a: String,
    x: String,
    y: String,
    p: String,
    sp: String,
    cyc: u64,
}

#[test]
fn mapper0() {
    let path = Path::new("tests/fixture/nestest.nes");
    let cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(cartridge);
    let cpu = CPU::new(mapper0);

    assert_eq!(0xc5af, cpu.nmi());
    assert_eq!(0xc004, cpu.rst());
    assert_eq!(0xc5f4, cpu.irq());
}

#[test]
fn compare_with_nestest() {
    let path = Path::new("tests/fixture/nestest.nes");
    let cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(cartridge);

    let cpu = Rc::new(RefCell::new(CPU::new(mapper0)));
    let ppu = Rc::new(RefCell::new(PPU::new()));
    let bus = Rc::new(RefCell::new(Bus::new(cpu.clone(), ppu.clone())));
    cpu.borrow_mut().connect_bus(bus.clone());

    let mut i = 1;
    println!("check status before running line {}...", i);

    cpu.borrow_mut().set_PC(0xc000);
    cpu.borrow_mut().set_cycles(7);

    let csv_file = File::open("tests/fixture/status.txt").unwrap();
    let mut rdr = csv::Reader::from_reader(csv_file);
    for result in rdr.deserialize() {
        let target: Status = result.unwrap();
        match cpu.borrow().debug_info() {
            (s, addr, ins, a, x, y, p, sp, cyc) => {
                let current = Status {
                    addr: format!("{:04X}", addr),
                    ins: ins,
                    a: format!("{:02X}", a),
                    x: format!("{:02X}", x),
                    y: format!("{:02X}", y),
                    p: format!("{:02X}", p),
                    sp: format!("{:02X}", sp),
                    cyc: cyc,
                };
                println!("current: {:?}", current);
                println!("target : {:?}", target);
                assert_eq!(current, target);
            }
        }
        println!("execute line {}...\n", i);
        cpu.borrow_mut().step();

        i += 1;
        println!("check status before running line {}...", i);
    }
}
