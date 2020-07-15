use crate::{Mapper, Result};

const VERCTOR_NMI: u16 = 0xFFFA;
const VERCTOR_RST: u16 = 0xFFFC;
const VERCTOR_IRQ: u16 = 0xFFFE;

pub struct CPU<T: Mapper> {
    mapper: T
}

// CPU Memory map
// -------------------------------------------------------------------------------------------------
// Address range |  Size   |  Device
// $0000-$07FF	 |  $0800  |  2KB internal RAM
// $0800-$0FFF	 |  $0800  |  Mirrors of $0000-$07FF
// $1000-$17FF	 |  $0800  |  Mirrors of $0000-$07FF
// $1800-$1FFF	 |  $0800  |  Mirrors of $0000-$07FF
// $2000-$2007	 |  $0008  |  NES PPU registers
// $2008-$3FFF	 |  $1FF8  |  Mirrors of $2000-2007 (repeats every 8 bytes)
// $4000-$4017	 |  $0018  |  NES APU and I/O registers
// $4018-$401F	 |  $0008  |  APU and I/O functionality that is normally disabled. See CPU Test Mode
// $4020-$FFFF	 |  $BFE0  |  Cartridge space: PRG ROM, PRG RAM, and mapper registers (See Note)
impl<T: Mapper> CPU<T> {
    pub fn new(mapper: T) -> Self {
        CPU { mapper }
    }

    pub fn read(&self, addr: u16) -> Result<u8> {
        match addr {
            0..=0x1fff => todo!(),
            0x2000..=0x3fff => todo!(),
            0x4000..=0x4013 => Ok(0),
            0x4014 => todo!(),
            0x4015 => todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => Ok(0), // normally disabled, maybe should return Err
            0x4020..=0xffff => self.mapper.read(addr)
        }
    }

    pub fn read16(&self, addr: u16) -> Result<u16> {
        let lo = self.read(addr)? as u16;
        let hi = self.read(addr + 1)? as u16;
        Ok(lo | hi << 8)
    }

    pub fn nmi(&self) -> Result<u16> {
        self.read16(VERCTOR_NMI)
    }

    pub fn rst(&self) -> Result<u16> {
        self.read16(VERCTOR_RST)
    }

    pub fn irq(&self) -> Result<u16> {
        self.read16(VERCTOR_IRQ)
    }
}