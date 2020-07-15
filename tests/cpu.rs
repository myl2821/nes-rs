use nes::{Cartridge, Mapper0, CPU};
use std::path::Path;

#[test]
fn mapper0() {
    let path = Path::new("tests/fixture/nestest.nes");
    let mut cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(&mut cartridge);
    let cpu = CPU::new(mapper0);

    assert_eq!(0xc5af, cpu.nmi().unwrap());
    assert_eq!(0xc004, cpu.rst().unwrap());
    assert_eq!(0xc5f4, cpu.irq().unwrap());
}
