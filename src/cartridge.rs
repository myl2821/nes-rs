use crate::Result;

use std::fs::File;
use std::io::prelude::*;
use std::io::{self, Read, SeekFrom};
use std::path::Path;
use std::slice;

pub const MAGIC_MUMBER: u32 = 0x1a53454e;

pub struct CartridgeHeader {
    // Should be 0x1a53454e to identify the file as an iNES file
    pub magic: u32,
    // number of 16 KB PRG-ROM banks
    pub prg_size: u8,
    // number of 8 KB CHR-ROM banks
    pub chr_size: u8,
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

    pub fn mirror_num(&self) -> u8 {
        (self.rom_ctrl1 & 1) | (((self.rom_ctrl1 >> 3) & 1) << 1)
    }

    pub fn battery(&self) -> bool {
        (self.rom_ctrl1 >> 1 & 1) == 1
    }

    pub fn trainer(&self) -> bool {
        (self.rom_ctrl1 & 4) == 4
    }
}

pub struct Cartridge {
    // PRG-ROM data
    pub prg: Vec<u8>,
    // CHR-ROM data
    pub chr: Vec<u8>,
    // Mapper number
    pub mapper: u8,
    // Mirror tag
    pub mirror: u8,
    // Battery tag
    pub battery: bool,
}

impl Cartridge {
    pub fn new(path: &Path) -> Result<Self> {
        let header = CartridgeHeader::new(path)?;
        if header.magic != MAGIC_MUMBER {
            return Err(From::from("Magic number is wrong"));
        }

        let mapper = header.mapper_num();
        let mirror = header.mirror_num();
        let battery = header.battery();

        let mut f = File::open(path)?;

        // Skip header info
        f.seek(SeekFrom::Start(
            ::std::mem::size_of::<CartridgeHeader>() as u64
        ))?;

        // Ignore trainer data
        if header.trainer() {
            let mut trainer = [0; 512];
            f.read(&mut trainer)?;
        }

        // Read PRG-ROM, 16KB each
        let mut prg = vec![0; (header.prg_size as usize) * 16 * (1 << 10)];
        f.read(&mut prg)?;

        // Read CHR-ROM, 16KB each
        let mut chr = vec![0; (header.chr_size as usize) * 8 * (1 << 10)];
        f.read(&mut chr)?;

        Ok(Cartridge {
            prg,
            chr,
            mapper,
            mirror,
            battery,
        })
    }
}
