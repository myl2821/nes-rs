// http://wiki.nesdev.com/w/index.php/PPU_programmer_reference

use crate::{Interrupt, Mapper, PALETTE};
use sdl2::pixels::Color;

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

bitflags! {
    #[derive(Default)]
    pub struct SpriteAttr: u8 {
        const priority = 0x20;  // Sprite has priority over the background
        const horizontal = 0x40;  // Flip the sprite horizontally
        const vertical = 0x80;  // Flip the sprite vertically
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Pixel {
    pub x: u32,
    pub y: u32,
    pub c: Color,
}

#[derive(Default, Debug)]
struct Sprite {
    i: usize,         // index of oam_data
    x: u8,            // X-coordinate
    y: u8,            // Y-coordinate of the top left of the sprite minus 1
    tile: u8,         // Index number of the sprite in the pattern tables
    pp: u8,           // Most significant two bits of the colour
    attr: SpriteAttr, // Stores the attributes of the sprite
    pattern: u32,     // 8 pixels, 8*4bits, every 4 bits represent one palette color
}

// http://wiki.nesdev.com/w/index.php/PPU_programmer_reference
// http://wiki.nesdev.com/w/index.php/PPU_scrolling
// PPU Memory map
// -------------------------------------------------------------------------------------------------
// Address range |  Size   |  Device
// $0000-$1FFF   |  $2000  |  8KB Pattern Tables, VRAM, CHR ROM
// $2000-$23FF   |  $0400  |  Name Table 0
// $2400-$27FF   |  $0400  |  Name Table 1
// $2800-$2BFF   |  $0400  |  Name Table 2
// $2C00-$2FFF   |  $0400  |  Name Table 3
// $3000-$3EFF   |  $0800  |  Mirrors of $2000-$2EFF
// $3F00-$3F0F   |  $0010  |  Image Palette
// $3F10-$3F1F   |  $0010  |  Sprite Palette
// $3F20-$3FFF   |  $00E0  |  Mirrors of $3F00-$3F1F
// $4000-$FFFF   |  $C000  |  Mirrors of $0000-$3FFF
pub struct PPU<M: Mapper> {
    name_table: [u8; 0x1000], // 4KB
    palette: [u8; 0x0020],    // 32B

    sprites: [Sprite; 8], // Only eight sprites are allowed per scanline
    sprite_cnt: usize,    // Valid sprite count

    pub mapper: M,

    ctrl: CtrlFlag,         // 0x2000  PPU Control Register
    mask: MaskFlag,         // 0x2001  PPU Mask Register
    status: StatusFlag,     // 0x2002  PPU status register
    pub oam_addr: u8,       // 0x2003  OAM address port
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

    tiles: u64,

    pub front: Pixel,
    pub back: Pixel,

    scanline: u32, // 0-261
    cycle: u32,    // 0-340

    delay: u8,
    previous: bool,
}

impl<M: Mapper> PPU<M> {
    pub fn new(mapper: M) -> Self {
        let mut ppu = Self {
            name_table: [0; 0x1000],
            palette: [0; 0x0020],
            sprites: Default::default(),
            sprite_cnt: 0,
            mapper: mapper,
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
            tiles: 0,
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
            previous: false,
        };
        ppu.reset();
        ppu
    }

    pub fn n_change(&mut self) {
        let n = self.ctrl.contains(CtrlFlag::nmi) && self.status.contains(StatusFlag::v_blank);
        if n && !self.previous {
            self.delay = 20
        }
        self.previous = n
    }

    pub fn read8(&self, d: u16) -> u8 {
        let addr = d & 0x3fff;
        match addr {
            0..=0x1fff => self.mapper.read(addr),
            0x2000..=0x3eff => self.name_table[(addr & 0x0fff) as usize],
            0x3f00..=0x3fff => self.read_palette(addr % 32), //self.palette[(addr % 0x001f) as usize],
            _ => panic!(),
        }
    }

    pub fn write8(&mut self, d: u16, v: u8) {
        let addr = d & 0x3fff;
        match addr {
            0..=0x1fff => self.mapper.write(addr, v),
            0x2000..=0x3eff => self.name_table[(addr & 0x0fff) as usize] = v,
            0x3f00..=0x3fff => self.write_palette(addr % 32, v), //self.palette[(addr % 0x001f) as usize] = v,
            _ => panic!(),
        }
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

    fn read_palette(&self, addr: u16) -> u8 {
        let mut new_addr = addr;
        if addr >= 16 && addr % 4 == 0 {
            new_addr -= 16
        }
        self.palette[new_addr as usize]
    }

    fn write_palette(&mut self, addr: u16, v: u8) {
        let mut new_addr = addr;
        if addr >= 16 && addr % 4 == 0 {
            new_addr -= 16
        }
        self.palette[new_addr as usize] = v
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
            _ => panic!(),
        }
    }

    fn tick(&mut self) -> Interrupt {
        let mut interrupt = Interrupt::NONE;

        if self.delay > 0 {
            self.delay -= 1;
            if self.delay == 0
                && self.ctrl.contains(CtrlFlag::nmi)
                && self.status.contains(StatusFlag::v_blank)
            {
                interrupt = Interrupt::NMI;
            }
        }

        if self.enable_render() {
            if self.scanline == 261 && self.cycle == 339 {
                self.cycle = 0;
                self.scanline = 0;
                return interrupt;
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

        return interrupt;
    }

    // http://wiki.nesdev.com/w/index.php/PPU_rendering
    pub fn step(&mut self) -> Interrupt {
        let interrupt = self.tick();
        // FIXME: just test
        //println!("cycle: {:?}, scanline: {:?}, enable_render: {:?}, mask: {:?}, ctrl: {:?}, status: {:?} v: {:04X} t: {:04X}",
        //    self.cycle, self.scanline, self.enable_render(), self.mask, self.ctrl, self.status, self.v, self.t);

        match self.scanline {
            // Scanlines (0-240)
            0..=240 => {
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
                                self.reload_x();
                                self.sprite_evaluation();
                            }
                        }
                        // Cycles 321-336: This is where the first two tiles for the next scanline are fetched, and loaded into the shift registers.
                        321..=336 => {
                            self.fetch_tile_process();
                        }
                        // Cycles 337-340: Two bytes are fetched, but the purpose for this is unknown.
                        337..=340 => (),
                        _ => panic!(),
                    }
                }
            }
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
                        257 => {
                            self.reload_x();
                            self.sprite_cnt = 0;
                        }
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
                    self.status.remove(StatusFlag::spr_hit);
                    self.status.remove(StatusFlag::spr_ovf);
                }
            }
            _ => panic!(),
        }
        interrupt
    }

    fn reset(&mut self) {
        self.cycle = 340;
        self.scanline = 240;
        self.write_ctrl(0);
        self.write_mask(0);
        self.write_oam_addr(0);
    }

    fn fetch_tile_process(&mut self) {
        self.tiles <<= 4;
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
        self.n_change()
    }

    fn clear_v_blank(&mut self) {
        self.status.remove(StatusFlag::v_blank);
        self.n_change()
    }


    // The palette for the background runs from VRAM $3F00 to $3F0F;
    // the palette for the sprites runs from $3F10 to $3F1F. Each color takes up one byte.
    // Address      Purpose
    // $3F00        Universal background color
    // $3F01-$3F03  Background palette 0
    // $3F05-$3F07  Background palette 1
    // $3F09-$3F0B  Background palette 2
    // $3F0D-$3F0F  Background palette 3
    // $3F11-$3F13  Sprite palette 0
    // $3F15-$3F17  Sprite palette 1
    // $3F19-$3F1B  Sprite palette 2
    // $3F1D-$3F1F  Sprite palette 3
    //
    // The palette entry at $3F00 is the background colour and is used for transparency.
    //
    // Mirroring is used so that every four bytes in the palettes is a copy of $3F00.
    // Therefore $3F04, $3F08, $3F0C, $3F10, $3F14, $3F18 and $3F1C are just copies of $3F00
    // and the total number of colours in each palette is 13, not 16.
    fn render_pixel(&mut self) {
        let x = self.cycle - 1;
        let y = self.scanline;
        let mut bg_pixel = self.bg_pixel();
        let (idx, mut sprite_pixel) = self.sprite_pixel();

        if x < 8 && !self.mask.contains(MaskFlag::bg_left) {
            bg_pixel = 0
        }
        if x < 8 && !self.mask.contains(MaskFlag::spr_left) {
            sprite_pixel = 0
        }
        let b = bg_pixel%4 != 0;  // FIXME: not transparency
        let s = sprite_pixel%4 != 0;  // FIXME: not transparency
        let color: u8;
        if !b && !s {
            color = 0;
        } else if !b && s {
            color = sprite_pixel|0x10
        } else if b && !s {
            color = bg_pixel
        } else {
            if self.sprites[idx].i == 0 && x < 255 {
                self.status.insert(StatusFlag::spr_hit)
            }
            if self.sprites[idx].attr.contains(SpriteAttr::priority) {
                color = bg_pixel;
            } else {
                color = sprite_pixel;
            }
        }

        self.back = Pixel {
            x: x,
            y: y,
            c: PALETTE[self.read8(0x3f00|color as u16) as usize % 64],
        };
    }

    // 0x2002
    // Reading the status register will clear bit 7
    // w:= 0
    fn read_status(&mut self) -> u8 {
        let result = self.status.bits;
        self.status.remove(StatusFlag::v_blank);
        self.n_change();
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
        self.n_change();
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
    pub fn write_dma(&mut self, oam_addr: u8, oam_data: [u8; 256]) {
        self.oam_addr = oam_addr;
        self.oam_data = oam_data;
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
        self.attribute_table_byte = ((attribute >> shift) & 0x03) << 2;
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
        let bg_tbl = match self.ctrl.contains(CtrlFlag::bg_tbl) {
            true => 0x1000,
            false => 0x0000,
        };
        let tile = self.name_table_byte as u16;
        let fine_y = (self.v >> 12) & 0x07;
        bg_tbl | (tile << 4) | fine_y
    }

    // Get Palette index bit 0
    fn pattern_table_tile_low(&mut self) {
        self.pattern_table_tile_low = self.read8(self.pattern_table_addr());
    }

    // Get Palette index bit 1
    fn pattern_table_tile_high(&mut self) {
        self.pattern_table_tile_high = self.read8(self.pattern_table_addr() + 8);
    }

    fn tiles(&mut self) {
        let plane_0 = self.pattern_table_tile_low;
        let plane_1 = self.pattern_table_tile_high;
        let mut data: u32 = 0;
        for i in 0..=7 {
            let a = self.attribute_table_byte;
            let bit0 = (plane_0 >> (7 - i)) & 0x01;
            let bit1 = ((plane_1 >> (7 - i)) & 0x01) << 1;
            data <<= 4;
            data |= (a | bit1 | bit0) as u32;
        }
        self.tiles |= data as u64
    }

    fn bg_pixel(&self) -> u8 {
        let data = ((self.tiles >> 32) as u32) >> ((7 - self.x) * 4);
        (data & 0x0f) as u8
    }

    // 8x16 sprites use different pattern tables based on their index number.
    // If the index number is even the sprite data is in the first pattern table at $0000,
    // otherwise it is in the second pattern table at $1000.
    fn sprite_pattern(&self, spr: &Sprite, row: u32) -> u32 {
        let mut tile = spr.tile as u16;
        let mut row = row;
        let spr_tbl = match self.ctrl.contains(CtrlFlag::spr_tbl) {
            true => 0x1000,
            false => 0x0000,
        };

        let addr = if !self.ctrl.contains(CtrlFlag::spr_sz) {
            if spr.attr.contains(SpriteAttr::vertical) {
                row = 7 - row
            }
            spr_tbl | (tile << 4) | row as u16
        } else {
            if spr.attr.contains(SpriteAttr::vertical) {
                row = 15 - row
            }
            tile &= 0xfe;
            if row > 7 {
                tile += 1;
                row -= 8;
            }
            (spr_tbl & 1) | (tile << 4) | row as u16
        };
        let tile_low = self.read8(addr);
        let tile_high = self.read8(addr + 8);
        let mut data: u32 = 0;
        for i in 0..=7 {
            let bit1: u8;
            let bit0: u8;
            if spr.attr.contains(SpriteAttr::horizontal) {
                bit0 = (tile_low >> i) & 0x01;
                bit1 = ((tile_high >> i) & 0x01) << 1;
            } else {
                bit0 = (tile_low >> (7 - i)) & 0x01;
                bit1 = ((tile_high >> (7 - i)) & 0x01) << 1;
            }
            data <<= 4;
            data |= ((spr.pp<<2) | bit1 | bit0) as u32;
        }
        data
    }

    fn sprite_pixel(&self) -> (usize, u8) {
        let x = self.cycle - 1;
        for i in 0..self.sprite_cnt {
            let offset = x as i32 - self.sprites[i].x as i32;
            if offset < 0 || offset > 7 {
                continue;
            }
            let color = ((self.sprites[i].pattern >> ((7-offset)*4)) & 0x0f) as u8;
            if color % 4 == 0 { //FIXME
                continue
            }
            return (i, color);
        }
        return (0, 0);
    }

    fn sprite_size(&self) -> i32 {
        match self.ctrl.contains(CtrlFlag::spr_sz) {
            true => 16,
            false => 8,
        }
    }

    fn sprite_evaluation(&mut self) {
        let size = self.sprite_size();
        let mut sprite_cnt = 0;

        for i in 0..=63 {
            // sprite 8*8 or 8*16
            let y = self.oam_data[i * 4 + 0];
            let row = self.scanline as i32 - y as i32;
            if row < 0 || row >= size{
                continue;
            }
            if sprite_cnt < 8 {
                let mut spr = Sprite {
                    i: i,
                    x: self.oam_data[i * 4 + 3],
                    y: self.oam_data[i * 4 + 0],
                    tile: self.oam_data[i * 4 + 1],
                    pp: self.oam_data[i * 4 + 2] & 0x03,
                    attr: SpriteAttr::from_bits(self.oam_data[i * 4 + 2]&0xe0).unwrap(),
                    pattern: 0,
                };
                spr.pattern = self.sprite_pattern(&spr, row as u32);
                self.sprites[sprite_cnt] = spr
            }
            sprite_cnt += 1;
            if sprite_cnt > 8 {
                sprite_cnt = 8;
                self.status.insert(StatusFlag::spr_ovf);
                break;
            }
        }
        self.sprite_cnt = sprite_cnt
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
