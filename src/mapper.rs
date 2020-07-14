use crate::{Cartridge, Result};

// The NES’ limited memory was sufficient for early games, however as they became more
// complex, games became larger and the memory was insufficient. To allow cartridges to
// contain more ROM, the NES had to be able to swap the data in and out of memory when it
// was needed. Since the NES could not address beyond $FFFF, switching hardware in the
// cartridges themselves was used. This hardware was known as a memory mapper or MMC
// (Memory Management Chip).
//
// The basic idea of memory mapping is that when the system requires access to data on a
// ROM bank that is not currently loaded in memory, the software indicates the need to switch
// banks and the selected bank is loaded into a page in memory, replacing the existing
// contents. The use of memory mappers was one of the factors in the NES’ longevity, allowing
// it to survive technological deficiencies.
pub trait Mapper {
    fn read(&self, addr: u16) -> Result<u8>;
    fn write(&mut self, addr: u16, v: u8);
}

// UNROM only allowed switching of PRG-ROM banks. It provided no support for CHR-ROM.
// 16 KB PRG-ROM bank number to load into $8000 directly
// $8000 ~ $BFFF -> First 16 KB of ROM
// $C000 ~ $FFFF -> Last 16 KB of ROM (NROM-256) or mirror of $8000-$BFFF (NROM-128)
pub struct Mapper0<'a> {
    cartridge: &'a mut Cartridge,
}

impl<'a> Mapper0<'a> {
    pub fn new(cartridge: &'a mut Cartridge) -> Self {
        Mapper0 { cartridge }
    }

    fn is_unrom256(&self) -> bool {
        self.cartridge.prg.len() / (1 << 10) > 16
    }
}

impl<'a> Mapper for Mapper0<'a> {
    fn read(&self, addr: u16) -> Result<u8> {
        match addr {
            // READ From CPU RAM (2KB)
            0..=0x1fff => todo!(),
            // PPU Registers
            0x2000..=0x3fff => todo!(),
            // APU Registers
            0x4000..=0x4013 => Ok(0),
            // OAM DAM
            0x4014 => todo!(),
            // APU Register
            0x4015 => todo!(),
            // Joypad 1
            0x4016 => todo!(),
            // Joypad 2
            0x4017 => todo!(),
            // cartridge
            0x4018..=0xffff => todo!(),
        }
    }
    fn write(&mut self, addr: u16, v: u8) {}
}
