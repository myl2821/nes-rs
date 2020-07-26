use crate::{new_mapper, Bus, Mapper, CPU, PPU};
use crate::{Interrupt, Result};
use sdl2::pixels::PixelFormatEnum::RGB888;
use sdl2::rect::Rect;
use sdl2::surface::Surface;
use std::path::Path;

pub const SCAIL: u32 = 2;
pub const SCREEN_WIDTH: u32 = 256 * SCAIL;
pub const SCREEN_HEIGHT: u32 = 240 * SCAIL;

pub struct NES {
    cpu: CPU<dyn Mapper>,
}

impl NES {
    pub fn load_rom(rom_path: &Path) -> Result<Self> {
        let mapper = new_mapper(rom_path)?;
        let ppu = PPU::new(mapper);
        let bus = Bus::new(ppu);
        let mut cpu = CPU::new(bus);
        cpu.reset();
        Ok(NES { cpu })
    }

    pub fn next_surface(&mut self) -> Surface {
        let mut surface = Surface::new(SCREEN_WIDTH, SCREEN_HEIGHT, RGB888).unwrap();
        loop {
            let mut x: u32 = 0;
            let mut y: u32 = 0;
            let mut frame_done = false;
            let cpu_cycles = self.cpu.step();
            for _ in 0..(cpu_cycles * 3) {
                let interrupt = self.cpu.bus.ppu.borrow_mut().step();
                match interrupt {
                    Interrupt::NMI => {
                        self.cpu.set_nmi();
                        frame_done = true;
                    }
                    _ => (),
                }
                let pixel = self.cpu.bus.ppu.borrow_mut().back;
                x = pixel.x;
                y = pixel.y;
                if x >= 256 || y >= 240 {
                    continue;
                }
                surface
                    .fill_rect(
                        Rect::new(
                            (pixel.x * SCAIL) as i32,
                            (pixel.y * SCAIL) as i32,
                            SCAIL,
                            SCAIL,
                        ),
                        pixel.c,
                    )
                    .unwrap();
            }
            if frame_done {
                return surface;
            }
        }
    }
}
