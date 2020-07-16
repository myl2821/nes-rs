use crate::{Cartridge, Result};
use std::fmt::format;

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
    fn read(&self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, v: u8);
}

// UNROM only allowed switching of PRG-ROM banks. It provided no support for CHR-ROM.
// 16 KB PRG-ROM bank number to load into $8000 directly
// $8000 ~ $BFFF -> First 16 KB of ROM
// $C000 ~ $FFFF -> Last 16 KB of ROM (NROM-256) or mirror of $8000-$BFFF (NROM-128)
pub struct Mapper0<'a> {
    cartridge: &'a mut Cartridge,
    is_unrom256: bool,
}

impl<'a> Mapper0<'a> {
    pub fn new(cartridge: &'a mut Cartridge) -> Self {
        let is_unrom256 = cartridge.prg.len() / (1 << 10) > 16;
        Mapper0 {
            cartridge,
            is_unrom256,
        }
    }
}

impl<'a> Mapper for Mapper0<'a> {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0x6000..=0x7fff => self.cartridge.sram[addr as usize - 0x6000],
            0x8000..=0xbfff => self.cartridge.prg[addr as usize - 0x8000],
            0xc000..=0xffff => {
                if (self.is_unrom256) {
                    let last_bank_start = (self.cartridge.prg.len() / 0x4000 - 1) * 0x4000;
                    self.cartridge.prg[last_bank_start + addr as usize - 0xc0000]
                } else {
                    self.cartridge.prg[addr as usize - 0xc000]
                }
            }
            _ => panic!("invalid address: {:#x}", addr),
        }
    }

    fn write(&mut self, addr: u16, v: u8) {
        match addr {
            0x6000..=0x7fff => self.cartridge.sram[addr as usize - 0x6000] = v,
            0x8000..=0xbfff => self.cartridge.prg[addr as usize - 0x8000] = v,
            0xc000..=0xffff => {
                if (self.is_unrom256) {
                    let last_bank_start = (self.cartridge.prg.len() / 0x4000 - 1) * 0x4000;
                    self.cartridge.prg[last_bank_start + addr as usize - 0xc0000] = v
                } else {
                    self.cartridge.prg[addr as usize - 0xc000] = v
                }
            }
            _ => panic!("invalid address: {:#x}", addr),
        }
    }
}
