// http://wiki.nesdev.com/w/index.php/PPU_programmer_reference

use crate::{Bus, Mapper, PALETTE};
use sdl2::pixels::Color;
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

#[derive(Debug, Clone, Copy)]
pub struct Pixel {
    pub x: u32,
    pub y: u32,
    pub c: Color,
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

    // Each memory access takes 2 PPU cycles to complete, and 4 must be performed per tile
    name_table_byte: u8,         // cycle%8 = 1
    attribute_table_byte: u8,    // cycle%8 = 3
    pattern_table_tile_low: u8,  // cycle%8 = 5
    pattern_table_tile_high: u8, // cycle%8 = 7

    tiles: [u8; 8], // 2 bytes of pattern_table, represent 8 tiles color

    pub front: Pixel,
    pub back: Pixel,

    scanline: u32, // 0-261
    cycle: u32,    // 0-340

    delay: u8,

    bus: Option<Rc<RefCell<Bus<T>>>>,
}

impl<T: Mapper> PPU<T> {
    pub fn new() -> Self {
        let mut ppu = Self {
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
            name_table_byte: 0x00,
            attribute_table_byte: 0x00,
            pattern_table_tile_low: 0x00,
            pattern_table_tile_high: 0x00,
            tiles: [0; 8],
            front: Pixel {
                x: 0,
                y: 0,
                c: Color::RGB(0, 0, 0),
            },
            back: Pixel {
                x: 0,
                y: 0,
                c: Color::RGB(0, 0, 0),
            },
            scanline: 0,
            cycle: 0,
            delay: 0x00,
            bus: None,
        };
        ppu.reset();
        ppu
    }

    pub fn connect_bus(&mut self, bus: Rc<RefCell<Bus<T>>>) {
        self.bus = Some(bus);
    }

    pub fn read8(&self, addr: u16) -> u8 {
        self.bus.as_ref().unwrap().borrow_mut().ppu_read8(addr)
    }

    pub fn write8(&mut self, addr: u16, v: u8) {
        self.bus.as_ref().unwrap().borrow_mut().ppu_write8(addr, v);
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
            0x4014 => self.write_dma(v),
            _ => panic!(),
        }
    }

    fn tik(&mut self) {
        if self.delay > 0 {
            self.delay -= 1;
            if self.delay == 0
                && self.ctrl.contains(CtrlFlag::nmi)
                && self.status.contains(StatusFlag::v_blank)
            {
                self.bus
                    .as_ref()
                    .unwrap()
                    .borrow_mut()
                    .cpu
                    .borrow_mut()
                    .set_nmi();
            }
        }

        if self.enable_render() {
            if self.scanline == 261 && self.cycle == 339 {
                self.cycle = 0;
                self.scanline = 0;
                return;
            }
        }

        self.cycle += 1;
        if self.cycle > 340 {
            self.cycle = 0;
            self.scanline += 1;
            if self.scanline > 261 {
                self.scanline = 0;
            }
        }
    }

    // http://wiki.nesdev.com/w/index.php/PPU_rendering
    pub fn step(&mut self) {
        self.tik();
        // FIXME: just test
        println!("cycle: {:?}, scanline: {:?}, enable_render: {:?}, mask: {:?}, ctrl: {:?}, status: {:?} v: {:04X} t: {:04X}",
            self.cycle, self.scanline, self.enable_render(), self.mask, self.ctrl, self.status, self.v, self.t);

        match self.scanline {
            // Visible scanlines (0-239)
            0..=239 => {
                if self.enable_render() {
                    match self.cycle {
                        // Cycle 0: This is an idle cycle.
                        0 => (),
                        // Cycles 1-256: The data for each tile is fetched during this phase
                        1..=256 => {
                            self.render_pixel();
                            self.fetch_tile_process();
                            if self.cycle == 256 {
                                self.incr_y()
                            }
                        }
                        // Cycles 257-320: The tile data for the sprites on the next scanline are fetched here.
                        257..=320 => {
                            if self.cycle == 257 {
                                self.reload_x()
                            }
                        }
                        // Cycles 321-336: This is where the first two tiles for the next scanline are fetched, and loaded into the shift registers.
                        321..=336 => {
                            self.fetch_tile_process();
                        }
                        // Cycles 337-340: Two bytes are fetched, but the purpose for this is unknown.
                        337..=340 => todo!(),
                        _ => panic!(),
                    }
                }
            }
            //The PPU just idles during this scanline.
            240 => (),
            // Vertical blanking lines
            241..=260 => {
                if self.scanline == 241 && self.cycle == 1 {
                    self.set_v_blank();
                }
            }
            // Pre-render scanline
            261 => {
                if self.enable_render() {
                    match self.cycle {
                        1..=256 => {
                            self.fetch_tile_process();
                            if self.cycle == 256 {
                                self.incr_y()
                            }
                        }
                        257 => self.reload_x(),
                        // During pixels 280 through 304 of this scanline, the vertical scroll
                        // bits are reloaded if rendering is enabled.
                        280..=304 => self.reload_y(),
                        321..=336 => {
                            self.fetch_tile_process();
                        }
                        _ => (),
                    }
                }
                if self.cycle == 1 {
                    self.clear_v_blank();
                }
            }
            _ => panic!(),
        }
    }

    fn reset(&mut self) {
        self.cycle = 340;
        self.scanline = 240;
        self.write_ctrl(0);
        self.write_mask(0);
        self.write_oam_addr(0);
    }

    fn fetch_tile_process(&mut self) {
        match self.cycle % 8 {
            1 => self.name_table_byte(),
            3 => self.attribute_table_byte(),
            5 => self.pattern_table_tile_low(),
            7 => self.pattern_table_tile_high(),
            0 => {
                self.tiles();
                self.incr_x()
            }
            _ => (),
        }
    }

    fn enable_render(&self) -> bool {
        self.mask.contains(MaskFlag::bg) | self.mask.contains(MaskFlag::spr)
    }

    fn set_v_blank(&mut self) {
        let tmp = self.back;
        self.back = self.front;
        self.front = tmp;
        self.status.insert(StatusFlag::v_blank);
    }

    fn clear_v_blank(&mut self) {
        self.status.remove(StatusFlag::v_blank);
    }

    fn render_pixel(&mut self) {
        let x = self.cycle - 1;
        let y = self.scanline;
        let bg_color = self.bg_color();
        self.back = Pixel {
            x: x,
            y: y,
            c: bg_color,
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
        let addr = self.v & 0x3fff;
        match addr {
            0x0000..=0x3eff => {
                result = self.data;
                self.data = self.read8(addr);
            }
            0x3f00..=0x3fff => {
                result = self.read8(addr);
                self.data = self.read8(addr - 0x1000);
            }
            _ => panic!(), // unreachable
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
            self.t = (self.t & 0x80ff) | (((d & 0x3f) as u16) << 8);
            self.w = 1
        } else {
            self.t = (self.t & 0xff00) | (d as u16);
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

    // 0x4014
    // Writing $XX will upload 256 bytes of data from CPU page $XX00-$XXFF to the internal PPU OAM.
    //
    // The CPU is suspended during the transfer, which will take 513 or 514 cycles after the $4014
    // write tick. (1 dummy read cycle while waiting for writes to complete, +1 if on an odd CPU cycle,
    // then 256 alternating read/write cycles.)
    //
    // The DMA transfer will begin at the current OAM write address.
    fn write_dma(&mut self, d: u8) {
        let mut addr = (d as u16) << 8;
        let bus = self.bus.as_ref().unwrap();
        let cpu = &bus.borrow_mut().cpu;
        for _i in 0..256 {
            self.oam_data[self.oam_addr as usize] = bus.borrow_mut().cpu_read8(addr);
            self.oam_addr = self.oam_addr.wrapping_add(1);
            addr += 1;
        }
        cpu.borrow_mut().suspend += 513;
        if cpu.borrow_mut().cycles & 1 == 1 {
            cpu.borrow_mut().suspend += 1
        }
    }

    // The coarse X component of v needs to be incremented when the next tile is reached.
    // Bits 0-4 are incremented, with overflow toggling bit 10. This means that bits 0-4
    // count from 0 to 31 across a single nametable, and bit 10 selects the current nametable horizontally.
    fn incr_x(&mut self) {
        if (self.v & 0x001f) == 31 {
            self.v &= 0xffe0;
            self.v ^= 0x0400;
        } else {
            self.v += 1;
        }
    }

    // If rendering is enabled, fine Y is incremented at dot 256 of each scanline,
    // overflowing to coarse Y, and finally adjusted to wrap among the nametables vertically.
    // Bits 12-14 are fine Y. Bits 5-9 are coarse Y. Bit 11 selects the vertical nametable.
    fn incr_y(&mut self) {
        if (self.v & 0x7000) != 0x7000 {
            self.v += 0x1000
        } else {
            self.v &= !0x7000;
            let mut y = (self.v & 0x03e0) >> 5;
            match y {
                29 => {
                    y = 0;
                    self.v ^= 0x0800
                }
                31 => y = 0,
                _ => y += 1,
            }
            self.v = (self.v & !0x03e0) | (y << 5)
        }
    }

    fn reload_x(&mut self) {
        self.v = (self.v & 0xfbe0) | (self.t & 0x041f)
    }

    fn reload_y(&mut self) {
        self.v = (self.v & 0x841f) | (self.t & 0x7be0)
    }

    // The high bits of v are used for fine Y during rendering, and addressing nametable
    // data only requires 12 bits, with the high 2 CHR addres lines fixed to the 0x2000 region.
    fn name_table_addr(&self) -> u16 {
        0x2000 | (self.v & 0x0FFF)
    }

    fn name_table_byte(&mut self) {
        self.name_table_byte = self.read8(self.name_table_addr())
    }

    // The low 12 bits of the attribute address are composed in the following way:
    // NN 1111 YYY XXX
    // || |||| ||| +++-- high 3 bits of coarse X (x/4)
    // || |||| +++------ high 3 bits of coarse Y (y/4)
    // || ++++---------- attribute offset (960 bytes)
    // ++--------------- nametable select
    fn attribute_table_addr(&self) -> u16 {
        0x23C0 | (self.v & 0x0C00) | ((self.v >> 4) & 0x38) | ((self.v >> 2) & 0x07)
    }

    // ,---+---+---+---.
    // |   |   |   |   |
    // + D1-D0 + D3-D2 +
    // |   |   |   |   |
    // +---+---+---+---+
    // |   |   |   |   |
    // + D5-D4 + D7-D6 +
    // |   |   |   |   |
    // `---+---+---+---'
    //
    // 7654 3210
    // |||| ||++- Color bits 3-2 for top left quadrant of this byte
    // |||| ++--- Color bits 3-2 for top right quadrant of this byte
    // ||++------ Color bits 3-2 for bottom left quadrant of this byte
    // ++-------- Color bits 3-2 for bottom right quadrant of this byte
    //
    // Get Palette index high 2 bits: bit 3 and bit 2
    fn attribute_table_byte(&mut self) {
        let attribute = self.read8(self.attribute_table_addr());
        let shift = ((self.v >> 4) & 0x04) | (self.v & 0x02);
        self.attribute_table_byte = (attribute >> shift) & 0x03
    }

    // DCBA98 76543210
    // ---------------
    // 0HRRRR CCCCPTTT
    // |||||| |||||+++- T: Fine Y offset, the row number within a tile
    // |||||| ||||+---- P: Bit plane (0: "lower"; 1: "upper")
    // |||||| ++++----- C: Tile column
    // ||++++---------- R: Tile row
    // |+-------------- H: Half of sprite table (0: "left"; 1: "right")
    // +--------------- 0: Pattern table is at $0000-$1FFF
    fn pattern_table_addr(&self) -> u16 {
        let half = match self.ctrl.contains(CtrlFlag::bg_tbl) {
            true => 0x1000,
            false => 0x0000,
        };
        let tile = self.name_table_byte as u16;
        let fine_y = (self.v >> 12) & 0x07;
        (half << 12) | (tile << 4) | fine_y
    }

    // Get Palette index bit 0
    fn pattern_table_tile_low(&mut self) {
        self.pattern_table_tile_low = self.read8(self.pattern_table_addr());
    }

    // Get Palette index bit 1
    fn pattern_table_tile_high(&mut self) {
        self.pattern_table_tile_high = self.read8(self.pattern_table_addr() + 8)
    }

    fn tiles(&mut self) {
        let plane_0 = self.pattern_table_tile_low;
        let plane_1 = self.pattern_table_tile_high;
        for i in 0..=7 {
            let bit0 = (plane_0 >> (7 - i)) & 0x01;
            let bit1 = ((plane_1 >> (7 - i)) & 0x01) << 1;
            self.tiles[i] = bit1 | bit0;
        }
    }

    // Background palette address
    fn bg_palette_addr(&self) -> u16 {
        0x3F00 | (self.attribute_table_byte << 2) as u16 | self.tiles[self.x as usize] as u16
    }

    fn bg_color(&self) -> Color {
        PALETTE[self.read8(self.bg_palette_addr()) as usize]
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
