extern crate sdl2;

use nes::{Bus, Cartridge, Mapper0, CPU, PPU};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::rect::Point;
use std::path::Path;
use std::time::{Duration, Instant, SystemTime};

use sdl2::pixels::Color;
use std::cell::RefCell;
use std::env::args;
use std::rc::Rc;

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
    let mapper0 = Rc::new(RefCell::new(Mapper0::new(cartridge)));

    let cpu = Rc::new(RefCell::new(CPU::new()));
    let ppu = Rc::new(RefCell::new(PPU::new(mapper0.clone())));
    let bus = Rc::new(RefCell::new(Bus::new(
        mapper0.clone(),
        cpu.clone(),
        ppu.clone(),
    )));
    cpu.borrow_mut().connect_bus(bus.clone());
    ppu.borrow_mut().connect_bus(bus.clone());

    cpu.borrow_mut().reset();

    let sdl_context = sdl2::init().unwrap();
    let ev = sdl_context.event().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window("rust-sdl2 demo", SCREEN_WIDTH, SCREEN_HEIGHT)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    let mut event_pump = sdl_context.event_pump().unwrap();
    let mut i = 0;

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
            let cpu_cycles = cpu.borrow_mut().step();
            //println!("{}", cpu.borrow().debug_info().0);

            for _ in 0..(cpu_cycles * 3) {
                ppu.borrow_mut().step();
                let pixel = ppu.borrow().back;
                x = pixel.x;
                y = pixel.y;
                if x >= 255 && y >= 240 {
                    break;
                }
                canvas.set_draw_color(pixel.c);
                canvas.draw_point(Point::new(pixel.x as i32, pixel.y as i32));
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
