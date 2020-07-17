#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate bitflags;

mod cartridge;
mod cpu;
mod mapper;
mod ppu;

pub use cartridge::{Cartridge, CartridgeHeader};
pub use cpu::CPU;
pub use mapper::{Mapper, Mapper0};

error_chain! {
    foreign_links {
        Io(std::io::Error);
    }
}
