use crate::ppu::PPU;
use crate::{Cartridge, Mapper, Mapper0, CPU};
use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

pub struct Bus<T: Mapper> {
    pub ram: [u8; 2048],
    cpu: Rc<RefCell<CPU<T>>>,
    ppu: Rc<RefCell<PPU>>,
}

impl<T: Mapper> Bus<T> {
    pub fn new(cpu: Rc<RefCell<CPU<T>>>, ppu: Rc<RefCell<PPU>>) -> Self {
        Self {
            ram: [0; 2048],
            cpu: cpu,
            ppu: ppu,
        }
    }
}
