#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate lazy_static;
extern crate sdl2;

pub const SCAIL: u32 = 2;
pub const SCREEN_WIDTH: u32 = 256 * SCAIL;
pub const SCREEN_HEIGHT: u32 = 240 * SCAIL;

mod apu;
mod bus;
mod cartridge;
mod controller;
mod cpu;
mod mapper;
mod nes;
mod palette;
mod ppu;

pub use self::nes::NES;
pub use apu::FrameSequencer;
pub use bus::Bus;
pub use cartridge::Cartridge;
pub use controller::{Button, Controller};
pub use cpu::{Interrupt, CPU};
pub use mapper::new as new_mapper;
pub use mapper::{Mapper, Mapper0};
pub use palette::PALETTE;
pub use ppu::PPU;

error_chain! {
    foreign_links {
        Io(std::io::Error);
    }
}
