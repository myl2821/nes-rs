extern crate csv;

use nes::{Cartridge, Mapper0, CPU};
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
    let mut cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(&mut cartridge);
    let cpu = CPU::new(mapper0);

    assert_eq!(0xc5af, cpu.nmi());
    assert_eq!(0xc004, cpu.rst());
    assert_eq!(0xc5f4, cpu.irq());
}

#[test]
fn run() {
    let path = Path::new("tests/fixture/nestest.nes");
    let mut cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(&mut cartridge);
    let mut cpu = CPU::new(mapper0);

    let mut i = 1;
    println!("check status before running line {}...", i);
    cpu.set_PC(0xc000);
    cpu.set_cycles(7);

    let csv_file = File::open("tests/fixture/status.txt").unwrap();
    let mut rdr = csv::Reader::from_reader(csv_file);
    for result in rdr.deserialize() {
        let target: Status = result.unwrap();
        match cpu.debug_info() {
            (s, addr, ins, a, x, y, p, sp, cyc) => {
                let status = Status {
                    addr: format!("{:04X}", addr),
                    ins: ins.clone(),
                    a: format!("{:02X}", a),
                    x: format!("{:02X}", x),
                    y: format!("{:02X}", y),
                    p: format!("{:02X}", p),
                    sp: format!("{:02X}", sp),
                    cyc: cyc,
                };
                println!("current: {:?}", target);
                println!("target : {:?}", target);
                assert_eq!(status, target);
            }
        }
        println!("execute line {}...\n", i);
        cpu.step();

        i += 1;
        println!("check status before running line {}...", i);
    }
}
