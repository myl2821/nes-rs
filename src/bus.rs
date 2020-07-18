use crate::ppu::PPU;
use crate::{Mapper, CPU};
use std::cell::RefCell;
use std::rc::Rc;

pub struct Bus<T: Mapper> {
    pub ram: [u8; 2048],
    pub mapper: T,
    cpu: Rc<RefCell<CPU<T>>>,
    ppu: Rc<RefCell<PPU<T>>>,
}

impl<T: Mapper> Bus<T> {
    pub fn new(mapper: T, cpu: Rc<RefCell<CPU<T>>>, ppu: Rc<RefCell<PPU<T>>>) -> Self {
        Self {
            ram: [0; 2048],
            mapper: mapper,
            cpu: cpu,
            ppu: ppu,
        }
    }
}
