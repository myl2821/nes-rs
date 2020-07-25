#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate lazy_static;
extern crate sdl2;

mod bus;
mod cartridge;
mod cpu;
mod mapper;
mod palette;
mod ppu;

pub use bus::Bus;
pub use cartridge::{Cartridge, CartridgeHeader};
pub use cpu::{Interrupt, CPU};
pub use mapper::{Mapper, Mapper0};
pub use palette::PALETTE;
pub use ppu::PPU;

error_chain! {
    foreign_links {
        Io(std::io::Error);
    }
}
