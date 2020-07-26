#[macro_use]
extern crate log;
extern crate sdl2;
extern crate simple_logger;

use log::*;
use nes::{NES, SCREEN_HEIGHT, SCREEN_WIDTH};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::path::Path;
use std::time::{Duration, Instant};

use sdl2::pixels::Color;
use std::env::args;
use std::error::Error;

const FPS: u32 = 60;
const INTERVAL: u32 = 1_000_000_000u32 / FPS;

fn main() -> Result<(), Box<dyn Error>> {
    simple_logger::init_with_level(Level::Debug)?;
    let ags: Vec<String> = args().collect();
    let rom_path = match ags.get(1) {
        Some(s) => s.clone(),
        None => "tests/fixture/nestest.nes".to_owned(),
    };
    let path = Path::new(&rom_path);
    let nes = NES::load_rom(path)?;
    Ok(play(nes))
}

fn play(mut nes: NES) {
    // init SDL2 renderer
    let sdl_context = sdl2::init().unwrap();
    let ev = sdl_context.event().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window("nes-rs", SCREEN_WIDTH, SCREEN_HEIGHT)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window
        .into_canvas()
        .accelerated()
        .present_vsync()
        .build()
        .unwrap();

    let mut event_pump = sdl_context.event_pump().unwrap();

    canvas.set_draw_color(Color::RGB(0, 0, 0));
    canvas.clear();
    let texture_creator = canvas.texture_creator();
    let mut last_frame_ts = Instant::now();

    // entering the SDL event loop!
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
        let surface = nes.next_surface();
        let texture = texture_creator
            .create_texture_from_surface(surface)
            .unwrap();
        canvas.copy(&texture, None, None).unwrap();
        canvas.present();

        // sleep a while to sync FPS
        let elapsed = last_frame_ts.elapsed().as_nanos() as u32;
        trace!("frame time: {} ms", elapsed / 1_000_000);
        if elapsed < INTERVAL {
            let time_to_sleep = INTERVAL - elapsed;
            std::thread::sleep(Duration::new(0, time_to_sleep));
        }
        last_frame_ts = Instant::now();
    }
}
