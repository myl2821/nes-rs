use crate::{new_mapper, Bus, Controller, Mapper, CPU, PPU, SCAIL, SCREEN_HEIGHT, SCREEN_WIDTH};
use crate::{Button, Interrupt, Result};
use sdl2::keyboard::{KeyboardState, Scancode};
use sdl2::pixels::PixelFormatEnum::RGB888;
use sdl2::rect::Rect;
use sdl2::surface::Surface;
use std::collections::HashMap;
use std::path::Path;

pub struct NES {
    cpu: CPU<dyn Mapper>,
}

lazy_static! {
    pub static ref KEY_MAP_1: HashMap<Scancode, Button> = vec![
        (Scancode::W, Button::UP),
        (Scancode::S, Button::DOWN),
        (Scancode::A, Button::LEFT),
        (Scancode::D, Button::RIGHT),
        (Scancode::G, Button::A),
        (Scancode::F, Button::B),
        (Scancode::R, Button::SELECT),
        (Scancode::T, Button::START),
    ]
    .into_iter()
    .collect();
    pub static ref KEY_MAP_2: HashMap<Scancode, Button> = vec![
        (Scancode::U, Button::UP),
        (Scancode::J, Button::DOWN),
        (Scancode::H, Button::LEFT),
        (Scancode::K, Button::RIGHT),
        (Scancode::Semicolon, Button::A),
        (Scancode::L, Button::B),
        (Scancode::O, Button::SELECT),
        (Scancode::P, Button::START),
    ]
    .into_iter()
    .collect();
}

impl NES {
    pub fn load_rom(rom_path: &Path) -> Result<Self> {
        let mapper = new_mapper(rom_path)?;
        let controller1 = Controller::new();
        let controller2 = Controller::new();
        let ppu = PPU::new(mapper);
        let bus = Bus::new(ppu, controller1, controller2);
        let mut cpu = CPU::new(bus);
        cpu.reset();
        Ok(NES { cpu })
    }

    pub fn next_surface(&mut self) -> Surface {
        let mut surface = Surface::new(SCREEN_WIDTH, SCREEN_HEIGHT, RGB888).unwrap();
        loop {
            let mut x: u32;
            let mut y: u32;
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

    pub fn set_ctrl_1(&mut self, state: &KeyboardState) {
        for (&key, &val) in KEY_MAP_1.iter() {
            if state.is_scancode_pressed(key) {
                self.cpu.bus.controller1.borrow_mut().press(val)
            } else {
                self.cpu.bus.controller1.borrow_mut().release(val)
            }
        }
    }

    pub fn set_ctrl_2(&mut self, state: &KeyboardState) {
        for (&key, &val) in KEY_MAP_2.iter() {
            if state.is_scancode_pressed(key) {
                self.cpu.bus.controller2.borrow_mut().press(val)
            } else {
                self.cpu.bus.controller2.borrow_mut().release(val)
            }
        }
    }
}
