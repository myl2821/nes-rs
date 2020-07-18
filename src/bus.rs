use crate::ppu::PPU;
use crate::{Mapper, CPU};
use std::cell::RefCell;
use std::rc::Rc;

pub struct Bus<T: Mapper> {
    ram: [u8; 0x0800],
    mapper: T,
    cpu: Rc<RefCell<CPU<T>>>,
    ppu: Rc<RefCell<PPU<T>>>,
}

impl<T: Mapper> Bus<T> {
    pub fn new(mapper: T, cpu: Rc<RefCell<CPU<T>>>, ppu: Rc<RefCell<PPU<T>>>) -> Self {
        Self {
            ram: [0; 0x0800],
            mapper: mapper,
            cpu: cpu,
            ppu: ppu,
        }
    }
    pub fn read8(&self, addr: u16) -> u8 {
        match addr {
            0..=0x1fff => {
                let a = addr % 0x0800;
                self.ram[a as usize]
            }
            0x2000..=0x3fff => todo!(),
            0x4000..=0x4013 => 0,
            0x4014 => todo!(),
            0x4015 => todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => 0, // normally disabled, maybe should return Err
            0x4020..=0xffff => self.mapper.read(addr),
        }
    }

    pub fn write8(&mut self, addr: u16, v: u8) {
        match addr {
            0..=0x1fff => {
                let a = addr % 0x0800;
                self.ram[a as usize] = v;
            }
            0x2000..=0x3fff => todo!(),
            0x4000..=0x4013 => (), //FIXME todo!(),
            0x4014 => todo!(),
            0x4015 => (), //FIXME todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => todo!(),
            0x4020..=0xffff => self.mapper.write(addr, v),
        }
    }
}
