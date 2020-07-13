use crate::Result;

use std::fs::File;
use std::io::{self, Read};
use std::path::Path;
use std::slice;

pub struct CartridgeHeader {
    // Should be 0x1a53454e to identify the file as an iNES file
    pub magic: u32,
    // number of 16 KB PRG-ROM banks
    pub prg_size: u8,
    // number of 8 KB PRG-ROM banks
    pub chg_size: u8,
    // ROM control byte 1
    pub rom_ctrl1: u8,
    // ROM control byte 2
    pub rom_ctrl2: u8,
    // number of RAM banks (8KB for each), assume 1 when this is 0
    pub ram_size: u8,
    // reserved bytes
    pub reserved: [u8; 7],
}

impl CartridgeHeader {
    fn read_struct<T, R: Read>(mut read: R) -> io::Result<T> {
        let num_bytes = ::std::mem::size_of::<T>();
        unsafe {
            let mut s = ::std::mem::MaybeUninit::uninit().assume_init();
            let buffer = slice::from_raw_parts_mut(&mut s as *mut T as *mut u8, num_bytes);
            match read.read_exact(buffer) {
                Ok(()) => Ok(s),
                Err(e) => {
                    ::std::mem::forget(s);
                    Err(e)
                }
            }
        }
    }

    pub fn new(path: &Path) -> Result<Self> {
        let f = File::open(path)?;
        Ok(Self::read_struct(f)?)
    }

    pub fn mapper_num(&self) -> u8 {
        (self.rom_ctrl1 >> 4) | (self.rom_ctrl2 & 0xf0)
    }
}
