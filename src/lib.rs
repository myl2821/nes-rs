#[macro_use]
extern crate error_chain;

pub mod cartridge;

error_chain! {
    foreign_links {
        IoErr(std::io::Error);
    }
}
