use crate::chip8::{Chip8, Frame};
use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::sync::atomic::Ordering;

const INITIAL_SCALE: u32 = 8;
const WIDTH: usize = 64;
const HEIGHT: usize = 32;
const TEXTURE_SIZE: usize = WIDTH * HEIGHT * 4;

pub struct Frontend {
    canvas: sdl2::render::WindowCanvas,
    pixel_data: [u8; TEXTURE_SIZE],
    texture: sdl2::render::Texture,
    event_pump: sdl2::EventPump,
}

impl Frontend {
    pub fn new() -> Frontend {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("", 64 * INITIAL_SCALE, 32 * INITIAL_SCALE)
            .resizable()
            .build()
            .unwrap();

        let mut canvas = window.into_canvas().build().unwrap();
        canvas.set_scale(INITIAL_SCALE as f32, INITIAL_SCALE as f32);
        let texture = canvas
            .create_texture_streaming(sdl2::pixels::PixelFormatEnum::RGBA32, 64, 32)
            .unwrap();

        let event_pump = sdl_context.event_pump().unwrap();
        Frontend {
            canvas: canvas,
            pixel_data: [0; TEXTURE_SIZE],
            texture: texture,
            event_pump: event_pump,
        }
    }

    pub fn load_game(&mut self, game_path: &Path) {
        let (mut backend, frame_rx) = Chip8::new();
        let interface = backend.get_interface();

        let mut game = vec![];
        File::open(game_path).unwrap().read_to_end(&mut game);

        std::thread::spawn(move || {
            backend.run_game(game.as_slice());
        });

        loop {
            for event in self.event_pump.poll_iter() {
                use sdl2::event::Event;
                match event {
                    Event::KeyDown { keycode: k, .. } => {
                        let idx = Frontend::key_map(k);
                        if idx.is_some() {
                            interface.keypad_state[idx.unwrap()].store(true, Ordering::Relaxed);
                        }
                    }
                    Event::KeyUp { keycode: k, .. } => {
                        let idx = Frontend::key_map(k);
                        if idx.is_some() {
                            interface.keypad_state[idx.unwrap()].store(false, Ordering::Relaxed);
                        }
                    }
                    Event::Quit { .. } => std::process::exit(0),
                    _ => {}
                }
            }
            let frame = frame_rx.try_iter().last();
            if !frame.is_none() {
                self.render_frame(frame.unwrap());
            }

            let dt = interface.delay_timer.load(Ordering::Relaxed);
            if dt > 0 {
                interface.delay_timer.store(dt - 1, Ordering::Relaxed);
            }

            let st = interface.sound_timer.load(Ordering::Relaxed);
            if st > 0 {
                interface.sound_timer.store(st - 1, Ordering::Relaxed);
            }

            interface.send_frame.store(true, Ordering::Relaxed);
            const SLEEP_TIME: std::time::Duration = std::time::Duration::from_millis(16);
            std::thread::sleep(SLEEP_TIME);
        }
    }

    fn key_map(k: Option<sdl2::keyboard::Keycode>) -> Option<usize> {
        use sdl2::keyboard::Keycode::*;
        match k {
            Some(Kp0) => Some(0x0),
            Some(Kp1) => Some(0x1),
            Some(Kp2) => Some(0x2),
            Some(Kp3) => Some(0x3),
            Some(Kp4) => Some(0x4),
            Some(Kp5) => Some(0x5),
            Some(Kp6) => Some(0x6),
            Some(Kp7) => Some(0x7),
            Some(Kp8) => Some(0x8),
            Some(Kp9) => Some(0x9),
            Some(KpPeriod) => Some(0xA),
            Some(KpEnter) => Some(0xB),
            Some(KpPlus) => Some(0xC),
            Some(KpMinus) => Some(0xD),
            Some(KpMultiply) => Some(0xE),
            Some(KpDivide) => Some(0xF),

            _ => None,
        }
    }

    fn render_frame(&mut self, frame: (Frame, u64)) {
        self.explode_frame_buffer(&frame.0);
        self.canvas.window_mut().set_title(&frame.1.to_string());
        self.texture.update(None, &self.pixel_data, WIDTH * 4);
        self.canvas
            .copy_ex(&self.texture, None, None, 0.0, None, true, false);
        self.canvas.present();
    }

    fn explode_frame_buffer(&mut self, input_raw: &Frame) {
        let input =
            unsafe { std::mem::transmute::<&[u64; HEIGHT], &[u8; WIDTH * HEIGHT / 8]>(input_raw) };
        let output = unsafe {
            std::mem::transmute::<&mut [u8; TEXTURE_SIZE], &mut [u32; WIDTH * HEIGHT]>(
                &mut self.pixel_data,
            )
        };
        for i in 0..input.len() {
            let byte = input[i];
            for j in 0..8 {
                output[i * 8 + j] = if byte & (1 << j) == 0 {
                    std::u32::MIN
                } else {
                    std::u32::MAX
                }
            }
        }
    }
}
