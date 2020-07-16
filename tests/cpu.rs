use nes::{Cartridge, Mapper0, CPU};
use std::path::Path;

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

    cpu.set_PC(0xc000);
    cpu.set_cycles(7);
    println!("{}", cpu.debug_info());

    for n in 1..10 {
        cpu.run();
        println!("{}", cpu.debug_info());
    }
}
