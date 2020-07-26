extern crate csv;

use nes::{new_mapper, Bus, Cartridge, Mapper0, CPU, NES, PPU};
use std::fs::File;
use std::path::Path;

use serde::Deserialize;

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
    let mapper = new_mapper(path).unwrap();

    let ppu = PPU::new(mapper);
    let bus = Bus::new(ppu);
    let cpu = CPU::new(bus);

    assert_eq!(0xc004, cpu.rst());
}

#[test]
fn compare_with_nestest() {
    let path = Path::new("tests/fixture/nestest.nes");
    let mapper = new_mapper(path).unwrap();

    let ppu = PPU::new(mapper);
    let bus = Bus::new(ppu);
    let mut cpu = CPU::new(bus);

    let mut i = 1;
    println!("check status before running line {}...", i);

    cpu.reset();
    cpu.set_PC(0xc000);
    cpu.set_cycles(7);

    let csv_file = File::open("tests/fixture/status.txt").unwrap();
    let mut rdr = csv::Reader::from_reader(csv_file);
    for result in rdr.deserialize() {
        let target: Status = result.unwrap();
        match cpu.debug_info() {
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
        cpu.step();

        i += 1;
        println!("check status before running line {}...", i);
    }
}
