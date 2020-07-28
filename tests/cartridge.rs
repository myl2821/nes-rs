use nes::Cartridge;
use std::path::Path;

#[test]
fn load_cartridge() {
    let path = Path::new("tests/fixture/nestest.nes");
    let cartridge = Cartridge::new(path).unwrap();
    assert_eq!(16384, cartridge.prg.len());
    assert_eq!(8192, cartridge.chr.len());
    assert_eq!(0, cartridge.mapper);
    assert_eq!(0, cartridge.mirror);
    assert_eq!(false, cartridge.has_sram);
}
