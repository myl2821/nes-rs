use nes::{new_mapper, Bus, Cartridge, Interrupt, Mapper0, CPU, PPU};
use std::path::Path;

#[test]
fn step() {
    let path = Path::new("tests/fixture/nestest.nes");
    let mapper = new_mapper(path).unwrap();

    let ppu = PPU::new(mapper);
    let bus = Bus::new(ppu);
    let mut cpu = CPU::new(bus);

    cpu.reset();

    for _n in 1..30000 {
        let cpu_cycles = cpu.step();
        println!("{}", cpu.debug_info().0);
        for _ in 0..(cpu_cycles * 3) {
            let interrupt = cpu.bus.ppu.borrow_mut().step();
            match interrupt {
                Interrupt::NMI => cpu.set_nmi(),
                _ => (),
            }
            let pixel = cpu.bus.ppu.borrow_mut().back;
            println!("{:?}", pixel)
        }
    }
}
