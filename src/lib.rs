#[macro_use]
extern crate error_chain;

mod cartridge;
mod mapper;

pub use cartridge::{Cartridge, CartridgeHeader};

error_chain! {
    foreign_links {
        Io(std::io::Error);
    }
}
