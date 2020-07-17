// http://wiki.nesdev.com/w/index.php/PPU_programmer_reference

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

pub struct PPU {
    ctrl: CtrlFlag,
    mask: MaskFlag,
    status: StatusFlag,
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
