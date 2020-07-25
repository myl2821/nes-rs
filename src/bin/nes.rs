extern crate sdl2;

use nes::{Bus, Cartridge, Interrupt, Mapper0, CPU, PPU};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::rect::Rect;
use std::path::Path;
use std::time::{Duration, Instant, SystemTime};

use sdl2::pixels::Color;
use std::env::args;

const SCAIL: u32 = 2;
const SCREEN_WIDTH: u32 = 256;
const SCREEN_HEIGHT: u32 = 240;
const FPS: u32 = 60;
const INTERVAL: u32 = 1_000_000_000u32 / FPS;

fn main() {
    let ags: Vec<String> = args().collect();
    let path = match ags.get(1) {
        Some(s) => s.clone(),
        None => "tests/fixture/nestest.nes".to_owned(),
    };
    draw(path);
}

fn draw(rom_path: String) {
    let path = Path::new(&rom_path);
    let cartridge = Cartridge::new(path).unwrap();
    let mapper0 = Mapper0::new(cartridge);

    let ppu = PPU::new(mapper0);
    let bus = Bus::new(ppu);
    let mut cpu = CPU::new(bus);
    cpu.reset();

    let sdl_context = sdl2::init().unwrap();
    let ev = sdl_context.event().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window(
            "rust-sdl2 demo",
            SCREEN_WIDTH * SCAIL,
            SCREEN_HEIGHT * SCAIL,
        )
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    let mut event_pump = sdl_context.event_pump().unwrap();

    canvas.set_draw_color(Color::RGB(0, 0, 0));
    canvas.clear();
    let mut last_frame_ts = Instant::now();
    'running: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => {
                    break 'running;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => {
                    ev.push_event(Event::Quit { timestamp: 0 });
                    continue 'running;
                }
                _ => {}
            }
        }

        // The rest of the game loop goes here...

        let mut x: u32 = 0;
        let mut y: u32 = 0;
        loop {
            let cpu_cycles = cpu.step();
            //println!("{}", cpu.debug_info().0);

            for _ in 0..(cpu_cycles * 3) {
                let interrupt = cpu.bus.ppu.borrow_mut().step();
                match interrupt {
                    Interrupt::NMI => cpu.set_nmi(),
                    _ => (),
                }
                let pixel = cpu.bus.ppu.borrow_mut().back;
                x = pixel.x;
                y = pixel.y;
                if x >= 255 && y >= 240 {
                    break;
                }
                canvas.set_draw_color(pixel.c);
                canvas
                    .fill_rect(Rect::new(
                        (pixel.x * SCAIL) as i32,
                        (pixel.y * SCAIL) as i32,
                        SCAIL,
                        SCAIL,
                    ))
                    .unwrap();
            }
            if x >= 255 && y >= 240 {
                break;
            }
        }

        canvas.present();

        let elapsed = last_frame_ts.elapsed().as_nanos() as u32;
        if elapsed < INTERVAL {
            let time_to_sleep = INTERVAL - elapsed;
            std::thread::sleep(Duration::new(0, time_to_sleep));
        }
        last_frame_ts = Instant::now();
    }
}
