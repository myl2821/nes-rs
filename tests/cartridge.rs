use nes::cartridge::CartridgeHeader;
use std::path::Path;

#[test]
fn read_header() {
    let path = Path::new("tests/fixture/nestest.nes");
    let header = CartridgeHeader::new(path).unwrap();
    assert_eq!(0x1a53454e, header.magic);
    assert_eq!(1, header.prg_size);
    assert_eq!(1, header.chg_size);
    assert_eq!(0x00, header.mapper_num());
}
