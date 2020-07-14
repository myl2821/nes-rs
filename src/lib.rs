#[macro_use]
extern crate error_chain;

mod cartridge;
mod mapper;
mod cpu;

pub use cartridge::{Cartridge, CartridgeHeader};
pub use mapper::{Mapper, Mapper0};
pub use cpu::CPU;

error_chain! {
    foreign_links {
        Io(std::io::Error);
    }
}
