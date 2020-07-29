# NES-RS
[![Build Status](https://travis-ci.org/myl2821/nes-rs.svg?branch=master)](https://travis-ci.org/myl2821/nes-rs)

Yet another NES emulator written in Rust.

## Installation

```
cargo build --release
```

## Usage

```
./nes $ROM_PATH
```

## Key Map

| Keyboard | NES Controller     |
| -------- | ------------------ |
| W,S,A,D  | Up,Down,Left,Right |
| G        | A                  |
| F        | B                  |
| R        | Select             |
| T        | Start              |
| Space    | Pause/Resume       |


## TODOs

- APU
- More mappers other than Mapper0

## License

MIT
