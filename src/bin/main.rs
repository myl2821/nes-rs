extern crate sdl2;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::Point;
use std::time::{Duration, Instant, SystemTime};

const SCREEN_WIDTH: u32 = 256;
const SCREEN_HEIGHT: u32 = 240;
const FPS: u32 = 60;
const INTERVAL: u32 = 1_000_000_000u32 / FPS;

fn main() {
    draw()
}

fn draw() {
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

        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();

        // The rest of the game loop goes here...

        i = (i + 1) % 255;
        canvas.set_draw_color(Color::RGB(0, 64, i));
        for x in 64..192 {
            for y in 60..180 {
                let pt = Point::new(x, y);
                canvas.draw_point(pt);
            }
        }

        canvas.present();

        let elapsed = last_frame_ts.elapsed().as_nanos() as u32;
        if elapsed < INTERVAL {
            let time_to_sleep = INTERVAL - last_frame_ts.elapsed().as_nanos() as u32;
            std::thread::sleep(Duration::new(0, time_to_sleep));
        }
        last_frame_ts = Instant::now();
    }
}
