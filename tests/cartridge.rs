use nes::cartridge::Cartridge;
use nes::cartridge::CartridgeHeader;
use std::path::Path;

#[test]
fn read_header() {
    let path = Path::new("tests/fixture/nestest.nes");
    let header = CartridgeHeader::new(path).unwrap();
    assert_eq!(0x1a53454e, header.magic);
    assert_eq!(1, header.prg_size);
    assert_eq!(1, header.chr_size);
    assert_eq!(0x00, header.mapper_num());
}

#[test]
fn load_cartridge() {
    let path = Path::new("tests/fixture/nestest.nes");
    let cartridge = Cartridge::new(path).unwrap();
    assert_eq!(16384, cartridge.prg.len());
    assert_eq!(8192, cartridge.chr.len());
    assert_eq!(0, cartridge.mapper);
    assert_eq!(0, cartridge.mirror);
    assert_eq!(false, cartridge.battery);
}
