use crate::ppu::PPU;
use crate::{Mapper, CPU};
use std::cell::RefCell;
use std::rc::Rc;

pub struct Bus<T: Mapper> {
    ram: [u8; 0x0800],        // 2KB
    name_table: [u8; 0x1000], // 4KB
    palette: [u8; 0x0020],    // 32B
    mapper: T,
    cpu: Rc<RefCell<CPU<T>>>,
    ppu: Rc<RefCell<PPU<T>>>,
}

impl<T: Mapper> Bus<T> {
    pub fn new(mapper: T, cpu: Rc<RefCell<CPU<T>>>, ppu: Rc<RefCell<PPU<T>>>) -> Self {
        Self {
            ram: [0; 0x0800],
            name_table: [0; 0x1000],
            palette: [0; 0x0020],
            mapper: mapper,
            cpu: cpu,
            ppu: ppu,
        }
    }

    // CPU Memory map
    // -------------------------------------------------------------------------------------------------
    // Address range |  Size   |  Device
    // $0000-$07FF   |  $0800  |  2KB internal RAM
    // $0800-$0FFF   |  $0800  |  Mirrors of $0000-$07FF
    // $1000-$17FF   |  $0800  |  Mirrors of $0000-$07FF
    // $1800-$1FFF   |  $0800  |  Mirrors of $0000-$07FF
    // $2000-$2007   |  $0008  |  NES PPU registers
    // $2008-$3FFF   |  $1FF8  |  Mirrors of $2000-2007 (repeats every 8 bytes)
    // $4000-$4017   |  $0018  |  NES APU and I/O registers
    // $4018-$401F   |  $0008  |  APU and I/O functionality that is normally disabled. See CPU Test Mode
    // $4020-$FFFF   |  $BFE0  |  Cartridge space: PRG ROM, PRG RAM, and mapper registers (See Note)
    pub fn cpu_read8(&self, addr: u16) -> u8 {
        match addr {
            0..=0x1fff => {
                let a = addr & 0x07ff;
                self.ram[a as usize]
            }
            0x2000..=0x3fff => self.ppu.borrow().read_register(0x2000 | (addr & 0x0008)),
            0x4000..=0x4013 => 0,
            0x4014 => todo!(),
            0x4015 => todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => 0, // normally disabled, maybe should return Err
            0x4020..=0xffff => self.mapper.read(addr),
        }
    }

    pub fn cpu_write8(&mut self, addr: u16, v: u8) {
        match addr {
            0..=0x1fff => {
                let a = addr & 0x07ff;
                self.ram[a as usize] = v;
            }
            0x2000..=0x3fff => self
                .ppu
                .borrow_mut()
                .write_register(0x2000 | (addr & 0x0008), v),
            0x4000..=0x4013 => (), //FIXME todo!(),
            0x4014 => self.ppu.borrow_mut().write_register(addr, v),
            0x4015 => (), //FIXME todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => todo!(),
            0x4020..=0xffff => self.mapper.write(addr, v),
        }
    }

    // PPU Memory map
    // -------------------------------------------------------------------------------------------------
    // Address range |  Size   |  Device
    // $0000-$1FFF   |  $2000  |  8KB Pattern Tables, VRAM, CHR ROM
    // $2000-$23FF   |  $0400  |  Name Table 0
    // $2400-$27FF   |  $0400  |  Name Table 1
    // $2800-$2BFF   |  $0400  |  Name Table 2
    // $2C00-$2FFF   |  $0400  |  Name Table 3
    // $3000-$3EFF   |  $0800  |  Mirrors of $2000-$2EFF
    // $3F00-$3F0F   |  $0010  |  Image Palette
    // $3F10-$3F1F   |  $0010  |  Sprite Palette
    // $3F20-$3FFF   |  $00E0  |  Mirrors of $3F00-$3F1F
    // $4000-$FFFF   |  $C000  |  Mirrors of $0000-$3FFF
    pub fn ppu_read8(&self, addr: u16) -> u8 {
        match addr {
            0..=0x1fff => self.mapper.read(addr),
            0x2000..=0x3eff => self.name_table[(addr & 0x0fff) as usize],
            0x3f00..=0x3fff => self.palette[(addr % 0x001f) as usize],
            0x4000..=0xffff => self.ppu_read8(addr & 0x3fff),
        }
    }

    pub fn ppu_write8(&mut self, addr: u16, v: u8) {
        match addr {
            0..=0x1fff => self.mapper.write(addr, v),
            0x2000..=0x3eff => self.name_table[(addr & 0x0fff) as usize] = v,
            0x3f00..=0x3fff => self.palette[(addr % 0x001f) as usize] = v,
            0x4000..=0xffff => self.ppu_write8(addr & 0x3fff, v),
        }
    }
}
