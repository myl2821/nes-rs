// http://wiki.nesdev.com/w/index.php/PPU_programmer_reference

use crate::{Bus, Mapper};
use std::cell::RefCell;
use std::rc::Rc;

bitflags! {
    pub struct CtrlFlag: u8 {
        const name_table = 0x03; // Nametable ($2000 / $2400 / $2800 / $2C00).
        const incr = 0x04;       // Address increment (1 / 32).
        const spr_tbl = 0x08;    // Sprite pattern table ($0000 / $1000).
        const bg_tbl = 0x10;     // BG pattern table ($0000 / $1000).
        const spr_sz = 0x20;     // Sprite size (8x8 / 8x16).
        const slave = 0x40;      // PPU master/slave.
        const nmi = 0x80;        // Enable NMI.
    }
}

bitflags! {
    pub struct MaskFlag: u8 {
        const gray = 0x01;     // Grayscale.
        const bg_left = 0x02;  // Show background in leftmost 8 pixels.
        const spr_left = 0x04; // Show sprite in leftmost 8 pixels.
        const bg = 0x08;       // Show background.
        const spr = 0x10;      // Show sprites.
        const red = 0x20;      // Intensify reds.
        const green = 0x40;    // Intensify greens.
        const blue = 0x80;     // Intensify blues.
    }
}

bitflags! {
    pub struct StatusFlag: u8 {
        const bus = 0x1f;      // Not significant.
        const spr_ovf = 0x20;  // Sprite overflow.
        const spr_hit = 0x40;  // Sprite 0 Hit.
        const v_blank = 0x80;  // In VBlank?
    }
}

// http://wiki.nesdev.com/w/index.php/PPU_programmer_reference
// http://wiki.nesdev.com/w/index.php/PPU_scrolling
pub struct PPU<T: Mapper> {
    ctrl: CtrlFlag,         // 0x2000  PPU Control Register
    mask: MaskFlag,         // 0x2001  PPU Mask Register
    status: StatusFlag,     // 0x2002  PPU status register
    oam_addr: u8,           // 0x2003  OAM address port
    oam_data: [u8; 0x0100], // 0x2004 OAM data port
    data: u8,               // 0x2007 PPU data port. The PPUDATA read buffer

    // The 15 bit registers t and v are composed this way during rendering:
    // yyy NN YYYYY XXXXX
    // ||| || ||||| +++++-- coarse X scroll
    // ||| || +++++-------- coarse Y scroll
    // ||| ++-------------- nametable select
    // +++----------------- fine Y scroll
    v: u16, // Current VRAM address (15 bits)
    t: u16, // Temporary VRAM address (15 bits); can also be thought of as the address of the top left onscreen tile.
    x: u8,  // Fine X scroll (3 bits)
    w: u8,  // First or second write toggle (1 bit)

    bus: Option<Rc<RefCell<Bus<T>>>>,
}

impl<T: Mapper> PPU<T> {
    pub fn new() -> Self {
        Self {
            ctrl: CtrlFlag::empty(),
            mask: MaskFlag::empty(),
            status: StatusFlag::empty(),
            oam_addr: 0x00,
            oam_data: [0x00; 0x0100],
            data: 0x00,
            v: 0x0000,
            t: 0x0000,
            x: 0x00,
            w: 0x00,
            bus: None,
        }
    }

    pub fn connect_bus(&mut self, bus: Rc<RefCell<Bus<T>>>) {
        self.bus = Some(bus);
    }

    pub fn read8(&self, addr: u16) -> u8 {
        self.bus.as_ref().unwrap().borrow_mut().ppu_read8(addr)
    }

    pub fn write8(&mut self, addr: u16, v: u8) {
        self.bus.as_ref().unwrap().borrow_mut().ppu_write8(addr, v)
    }

    // Reading any readable port (PPUSTATUS 0x2002, OAMDATA 0x2004, or PPUDATA 0x2007) also fills the latch with the bits read.
    pub fn read_register(&mut self, addr: u16) -> u8 {
        match addr {
            0x2002 => self.read_status(),
            0x2004 => self.read_oam_data(),
            0x2007 => self.read_data(),
            _ => panic!(),
        }
    }

    // Writing any value to any PPU port, even to the nominally read-only PPUSTATUS, will fill this latch.
    pub fn write_register(&mut self, addr: u16, v: u8) {
        match addr {
            0x2000 => self.write_ctrl(v),
            0x2001 => self.write_mask(v),
            0x2003 => self.write_oam_addr(v),
            0x2004 => self.write_oam_data(v),
            0x2005 => self.write_scroll(v),
            0x2006 => self.write_addr(v),
            0x2007 => self.write_data(v),
            0x4014 => todo!(),
            _ => panic!(),
        }
    }

    // 0x2002
    // Reading the status register will clear bit 7
    // w:= 0
    fn read_status(&mut self) -> u8 {
        let result = self.status.bits;
        self.status.remove(StatusFlag::v_blank);
        self.w = 0;
        result
    }

    // 0x2004
    fn read_oam_data(&self) -> u8 {
        self.oam_data[self.oam_addr as usize]
    }

    // 0x2007
    // When reading while the VRAM address is in the range 0-$3EFF (i.e., before the palettes),
    // the read will return the contents of an internal read buffer. This internal buffer is updated
    // only when reading PPUDATA, and so is preserved across frames. After the CPU reads and gets
    // the contents of the internal buffer, the PPU will immediately update the internal buffer
    // with the byte at the current VRAM address. Thus, after setting the VRAM address, one should
    // first read this register and discard the result.
    // Reading palette data from $3F00-$3FFF works differently. The palette data is placed immediately
    // on the data bus, and hence no dummy read is required. Reading the palettes still updates the
    // internal buffer though, but the data placed in it is the mirrored nametable data that would
    // appear "underneath" the palette. (Checking the PPU memory map should make this clearer.)
    fn read_data(&mut self) -> u8 {
        let result: u8;
        match self.v {
            0x0000..=0x3fff => {
                result = self.data;
                self.data = self.read8(self.v);
            }
            0x3f00..=0x3fff => {
                result = self.read8(self.v);
                self.data = self.read8(self.v - 0x1000);
            }
            _ => todo!(), // mirrored
        }
        if self.ctrl.contains(CtrlFlag::incr) {
            self.v += 32
        } else {
            self.v += 1
        }
        result
    }

    // 0x2000
    // Set nametable select to t
    // t: ...BA.. ........ = d: ......BA
    fn write_ctrl(&mut self, d: u8) {
        self.ctrl = CtrlFlag::from_bits(d).unwrap();
        self.t = (self.t & 0b11110011_11111111) | (((d & 0b00000011) as u16) << 10)
    }

    // 0x2001
    fn write_mask(&mut self, d: u8) {
        self.mask = MaskFlag::from_bits(d).unwrap()
    }

    // 0x2003
    fn write_oam_addr(&mut self, d: u8) {
        self.oam_addr = d
    }

    // 0x2004
    fn write_oam_data(&mut self, d: u8) {
        self.oam_data[self.oam_addr as usize] = d;
        self.oam_addr += 1
    }

    // 0x2005
    // first write (w is 0)
    // t: ....... ...HGFED = d: HGFED...
    // x:              CBA = d: .....CBA
    // w:                  = 1
    // second write (w is 1)
    // t: CBA..HG FED..... = d: HGFEDCBA
    // w:                  = 0
    fn write_scroll(&mut self, d: u8) {
        if self.w == 0 {
            self.t = (self.t & 0b11111111_11100000) | ((d >> 3) as u16);
            self.x = d & 0b00000111
        } else {
            self.t = (self.t & 0b10001100_00011111)
                | (((d & 0b00000111) as u16) << 12)
                | (((d & 0b11111000) as u16) << 2);
            self.w = 0
        }
    }

    // 0x2006
    // first write (w is 0)
    // t: .FEDCBA ........ = d: ..FEDCBA
    // t: X...... ........ = 0
    // w:                  = 1
    // second write (w is 1)
    // t: ....... HGFEDCBA = d: HGFEDCBA
    // v                   = t
    // w:                  = 0
    fn write_addr(&mut self, d: u8) {
        if self.w == 0 {
            self.t = (self.t & 0b11000000_11111111) | (((d & 0b00111111) as u16) << 8);
            self.t = self.t & 0b10111111_11111111;
            self.w = 0
        } else {
            self.t = (self.t & 0b11111111_00000000) | (d as u16);
            self.v = self.t;
            self.w = 0
        }
    }

    // 0x2007
    fn write_data(&mut self, d: u8) {
        self.write8(self.v, d);
        if self.ctrl.contains(CtrlFlag::incr) {
            self.v += 32
        } else {
            self.v += 1
        }
    }
}

#[test]
fn test_sf() {
    let mut sf = StatusFlag::empty();
    assert!(!sf.contains(StatusFlag::spr_ovf));
    sf.insert(StatusFlag::spr_ovf);
    assert!(sf.contains(StatusFlag::spr_ovf));
    sf.remove(StatusFlag::spr_ovf);
    assert!(!sf.contains(StatusFlag::spr_ovf));
}
