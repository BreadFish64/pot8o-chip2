extern crate rand;

use rand::prelude::*;
use rand::FromEntropy;
use std::sync::{
	atomic::{AtomicBool, AtomicU8, Ordering},
	mpsc, Arc,
};

pub type Frame = [u64; 32];

#[derive(Default)]
pub struct Interface {
	pub keypad_state: [AtomicBool; 16],
	pub send_frame: AtomicBool,
	pub delay_timer: AtomicU8,
	pub sound_timer: AtomicU8,
}

#[derive(Default)]
struct Stack {
	internal: [usize; 16],
	stack_ptr: usize,
}

impl Stack {
	pub fn push(&mut self, val: usize) {
		self.internal[self.stack_ptr] = val;
		self.stack_ptr += 1;
	}

	pub fn pop(&mut self) -> usize {
		self.stack_ptr -= 1;
		self.internal[self.stack_ptr]
	}
}

pub struct Chip8 {
	frame_buffer: Frame,
	memory: [u8; 0x1000],
	v: [u8; 16],
	stack: Stack,
	pc: usize,
	opcode: u16,
	i: usize,

	rng: SmallRng,
	frame_tx: mpsc::Sender<(Frame, u64)>,
	interface: Arc<Interface>,
}

	const JUMP_TABLE: [fn(&mut Chip8); 16] = [
		|chip8| match chip8.opcode {
			0x00E0 => chip8.cls(),
			0x00EE => chip8.ret(),
			_ => chip8.step(),
		},
		Chip8::jp_addr,
		Chip8::call_addr,
		Chip8::se_vx_byte,
		Chip8::sne_vx_byte,
		Chip8::se_vx_vy,
		Chip8::ld_vx_byte,
		Chip8::add_vx_byte,
		|chip8| match chip8.opcode & 0x000F {
			0x0 => chip8.ld_vx_vy(),
			0x1 => chip8.or_vx_vy(),
			0x2 => chip8.and_vx_vy(),
			0x3 => chip8.xor_vx_vy(),
			0x4 => chip8.add_vx_vy(),
			0x5 => chip8.sub_vx_vy(),
			0x6 => chip8.shr_vx_vy(),
			0x7 => chip8.subn_vx_vy(),
			0xE => chip8.shl_vx_vy(),
			_ => (),
		},
		Chip8::sne_vx_vy,
		Chip8::ld_i_addr,
		Chip8::jp_0_addr,
		Chip8::rnd_vx_byte,
		Chip8::drw_vx_vy_nibble,
		|chip8| match chip8.opcode & 0x00FF {
			0x9E => chip8.skp_vx(),
			0xA1 => chip8.sknp_vx(),
			_ => (),
		},
		|chip8| match chip8.opcode & 0x00FF {
			0x07 => chip8.ld_vx_dt(),
			0x0A => chip8.ld_vx_k(),
			0x15 => chip8.ld_dt_vx(),
			0x18 => chip8.ld_st_vx(),
			0x1E => chip8.add_i_vx(),
			0x29 => chip8.ld_f_vx(),
			0x33 => chip8.ld_b_vx(),
			0x55 => chip8.ld_i_vx(),
			0x65 => chip8.ld_vx_i(),
			_ => (),
		},
	];

impl Chip8 {

	pub fn new() -> (Chip8, mpsc::Receiver<(Frame, u64)>) {
		let (frame_tx, frame_rx) = mpsc::channel();
		let mut chip8 = Chip8 {
			frame_buffer: Default::default(),
			memory: [0; 0x1000],
			v: Default::default(),
			stack: Default::default(),
			pc: 0x200,
			opcode: 0,
			i: Default::default(),

			rng: SmallRng::from_entropy(),
			frame_tx: frame_tx,
			interface: Arc::new(Default::default()),
		};
		chip8.memory[..FONT.len()].copy_from_slice(&FONT);
		(chip8, frame_rx)
	}

	pub fn get_interface(&self) -> Arc<Interface> {
		self.interface.clone()
	}

	pub fn run_game(&mut self, game: &[u8]) {
		self.memory[0x200..0x200 + game.len()].copy_from_slice(&game);

		loop {
			self.opcode = ((self.memory[self.pc] as u16) << 8) | self.memory[self.pc + 1] as u16;

			JUMP_TABLE[((self.opcode & 0xF000) >> 12) as usize](self);

			/*
			match self.op() {
				0x0000 => match self.opcode {
					0x00E0 => self.cls(),
					0x00EE => self.ret(),
					_ => self.step(),
				},
				0x1000 => self.jp_addr(),
				0x2000 => self.call_addr(),
				0x3000 => self.se_vx_byte(),
				0x4000 => self.sne_vx_byte(),
				0x5000 => self.se_vx_vy(),
				0x6000 => self.ld_vx_byte(),
				0x7000 => self.add_vx_byte(),
				0x8000 => match self.opcode & 0x000F {
					0x0 => self.ld_vx_vy(),
					0x1 => self.or_vx_vy(),
					0x2 => self.and_vx_vy(),
					0x3 => self.xor_vx_vy(),
					0x4 => self.add_vx_vy(),
					0x5 => self.sub_vx_vy(),
					0x6 => self.shr_vx_vy(),
					0x7 => self.subn_vx_vy(),
					0xE => self.shl_vx_vy(),
					_ => assert!(false, "unknown opcode: {:X?}", self.opcode),
				},
				0x9000 => self.sne_vx_vy(),
				0xA000 => self.ld_i_addr(),
				0xB000 => self.jp_0_addr(),
				0xC000 => self.rnd_vx_byte(),
				0xD000 => self.drw_vx_vy_nibble(),
				0xE000 => match self.opcode & 0x00FF {
					0x9E => self.skp_vx(),
					0xA1 => self.sknp_vx(),
					_ => assert!(false, "unknown opcode: {:X?}", self.opcode),
				},
				0xF000 => match self.opcode & 0x00FF {
					0x07 => self.ld_vx_dt(),
					0x0A => self.ld_vx_k(),
					0x15 => self.ld_dt_vx(),
					0x18 => self.ld_st_vx(),
					0x1E => self.add_i_vx(),
					0x29 => self.ld_f_vx(),
					0x33 => self.ld_b_vx(),
					0x55 => self.ld_i_vx(),
					0x65 => self.ld_vx_i(),
					_ => assert!(false, "unknown opcode: {:X?}", self.opcode),
				},
				_ => assert!(false, "unknown opcode: {:X?}", self.opcode),
			}
			*/
		}
	}

	// helpers

	#[inline]
	fn op(&self) -> u16 {
		self.opcode & 0xF000
	}

	#[inline]
	fn x(&self) -> usize {
		((self.opcode & 0x0F00) >> 8) as usize
	}

	#[inline]
	fn y(&self) -> usize {
		((self.opcode & 0x00F0) >> 4) as usize
	}

	#[inline]
	fn vx(&mut self) -> &mut u8 {
		&mut self.v[self.x()]
	}

	#[inline]
	fn vy(&mut self) -> &mut u8 {
		&mut self.v[self.y()]
	}

	#[inline]
	fn n(&self) -> u8 {
		(self.opcode & 0x000F) as u8
	}

	#[inline]
	fn kk(&self) -> u8 {
		(self.opcode & 0x00FF) as u8
	}

	#[inline]
	fn nnn(&self) -> usize {
		(self.opcode & 0x0FFF) as usize
	}

	#[inline]
	fn step(&mut self) {
		self.pc += 2;
	}

	// ops

	fn cls(&mut self) {
		self.frame_buffer = [0; 32];
		self.step();
	}

	fn ret(&mut self) {
		self.pc = self.stack.pop() + 2;
	}

	fn jp_addr(&mut self) {
		self.pc = self.nnn();
	}

	fn call_addr(&mut self) {
		self.stack.push(self.pc);
		self.pc = self.nnn();
	}

	fn se_vx_byte(&mut self) {
		self.pc += if *self.vx() == self.kk() { 4 } else { 2 };
	}

	fn sne_vx_byte(&mut self) {
		self.pc += if *self.vx() != self.kk() { 4 } else { 2 };
	}

	fn se_vx_vy(&mut self) {
		self.pc += if *self.vx() == *self.vy() { 4 } else { 2 };
	}

	fn ld_vx_byte(&mut self) {
		*self.vx() = self.kk();
		self.step();
	}

	fn add_vx_byte(&mut self) {
		*self.vx() = self.vx().wrapping_add(self.kk());
		self.step();
	}

	fn ld_vx_vy(&mut self) {
		*self.vx() = *self.vy();
		self.step();
	}

	fn or_vx_vy(&mut self) {
		*self.vx() |= *self.vy();
		self.step();
	}

	fn and_vx_vy(&mut self) {
		*self.vx() &= *self.vy();
		self.step();
	}

	fn xor_vx_vy(&mut self) {
		*self.vx() ^= *self.vy();
		self.step();
	}

	fn add_vx_vy(&mut self) {
		let (result, overflow) = self.vx().overflowing_add(*self.vy());
		*self.vx() = result;
		self.v[0xF] = overflow as u8;
		self.step();
	}

	fn sub_vx_vy(&mut self) {
		let (result, overflow) = self.vx().overflowing_sub(*self.vy());
		*self.vx() = result;
		self.v[0xF] = !overflow as u8;
		self.step();
	}

	fn shr_vx_vy(&mut self) {
		//self.v[0xF] = *self.vy() & 0b0000_0001;
		//*self.vx() = *self.vy() >> 1;
		self.v[0xF] = *self.vx() & 1;
		*self.vx() >>= 1;
		self.step();
	}

	fn subn_vx_vy(&mut self) {
		let (result, overflow) = self.vy().overflowing_sub(*self.vx());
		*self.vx() = result;
		self.v[0xF] = !overflow as u8;
		self.step();
	}

	fn shl_vx_vy(&mut self) {
		//self.v[0xF] = *self.vy() >> 7;
		//*self.vx() = *self.vy() << 1;
		self.v[0xF] = *self.vx() >> 7;
		*self.vx() <<= 1;
		self.step();
	}

	fn sne_vx_vy(&mut self) {
		self.pc += if *self.vx() != *self.vy() { 4 } else { 2 };
	}

	fn ld_i_addr(&mut self) {
		self.i = self.nnn();
		self.step();
	}

	fn jp_0_addr(&mut self) {
		self.pc = self.nnn() + self.v[0x0] as usize;
	}

	fn rnd_vx_byte(&mut self) {
		*self.vx() = self.rng.gen::<u8>() & self.kk();
		self.step()
	}

	fn drw_vx_vy_nibble(&mut self) {
		let x = self.vx().wrapping_add(8) as u32;
		let y = *self.vy() as usize;
		let mut vf = 0;
		let height = self.n() as usize;

		for row in 0..height {
			let fb_row = &mut self.frame_buffer[(y + row) % 32];
			let sprite_row = (self.memory[self.i as usize + row] as u64).rotate_right(x);
			vf |= *fb_row & sprite_row;
			*fb_row ^= sprite_row;
		}
		self.v[0xF] = (vf != 0) as u8;

		static mut FRAME_COUNT: u64 = 0;
		unsafe { FRAME_COUNT += 1 };
		if self.interface.send_frame.swap(false, Ordering::Relaxed) {
			self.frame_tx
				.send((self.frame_buffer, unsafe { FRAME_COUNT }));
			unsafe { FRAME_COUNT = 0 };
		}

		self.step()
	}

	fn skp_vx(&mut self) {
		let k = *self.vx() as usize;
		self.pc += if self.interface.keypad_state[k].load(Ordering::Relaxed) {
			4
		} else {
			2
		};
	}

	fn sknp_vx(&mut self) {
		let k = *self.vx() as usize;
		self.pc += if !self.interface.keypad_state[k].load(Ordering::Relaxed) {
			4
		} else {
			2
		};
	}

	fn ld_vx_dt(&mut self) {
		*self.vx() = self.interface.delay_timer.load(Ordering::Relaxed);
		self.step()
	}

	// todo: improve this somehow
	fn ld_vx_k(&mut self) {
		loop {
			for k in 0..self.interface.keypad_state.len() {
				if self.interface.keypad_state[k].load(Ordering::Relaxed) {
					*self.vx() = k as u8;
					self.step();
					return;
				}
			}
		}
	}

	fn ld_dt_vx(&mut self) {
		let dt = *self.vx();
		self.interface.delay_timer.store(dt, Ordering::Relaxed);
		self.step();
	}

	fn ld_st_vx(&mut self) {
		let st = *self.vx();
		self.interface.sound_timer.store(st, Ordering::Relaxed);
		self.step();
	}

	fn add_i_vx(&mut self) {
		self.i += *self.vx() as usize;
		self.step();
	}

	fn ld_f_vx(&mut self) {
		self.i = *self.vx() as usize * 5;
		self.step();
	}

	fn ld_b_vx(&mut self) {
		let mut num = *self.vx();
		let mut dec = [0; 3];
		dec[0] = num / 100;
		num %= 100;
		dec[1] = num / 10;
		num %= 10;
		dec[2] = num;
		for ptr in 0..dec.len() {
			self.memory[self.i + ptr] = dec[ptr];
		}
		self.step()
	}

	fn ld_i_vx(&mut self) {
		let width = self.x() + 1;
		self.memory[self.i..self.i + width].copy_from_slice(&self.v[..width]);
		self.step();
	}

	fn ld_vx_i(&mut self) {
		let width = self.x() + 1;
		self.v[..width].copy_from_slice(&self.memory[self.i..self.i + width]);
		self.step();
	}
}

#[rustfmt::skip]
const FONT:[u8; 80] = [
    //0
	0b11110000,
	0b10010000,
	0b10010000,
	0b10010000,
	0b11110000,
	//1
	0b00100000,
	0b01100000,
	0b00100000,
	0b00100000,
	0b01110000,
	//2
	0b11110000,
	0b00010000,
	0b11110000,
	0b10000000,
	0b11110000,
	//3
	0b11110000,
	0b00010000,
	0b11110000,
	0b00010000,
	0b11110000,
	//4
	0b10010000,
	0b10010000,
	0b11110000,
	0b00010000,
	0b00010000,
	//5
	0b11110000,
	0b10000000,
	0b11110000,
	0b00010000,
	0b11110000,
	//6
	0b11110000,
	0b10000000,
	0b11110000,
	0b10010000,
	0b11110000,
	//7
	0b11110000,
	0b00010000,
	0b00100000,
	0b01000000,
	0b01000000,
	//8
	0b11110000,
	0b10010000,
	0b11110000,
	0b10010000,
	0b11110000,
	//9
	0b11110000,
	0b10010000,
	0b11110000,
	0b00010000,
	0b11110000,
	//A
	0b11110000,
	0b10010000,
	0b11110000,
	0b10010000,
	0b10010000,
	//B
	0b11100000,
	0b10010000,
	0b11100000,
	0b10010000,
	0b11100000,
	//C
	0b11110000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b11110000,
	//D
	0b11100000,
	0b10010000,
	0b10010000,
	0b10010000,
	0b11100000,
	//E
	0b11110000,
	0b10000000,
	0b11110000,
	0b10000000,
	0b11110000,
	//F
	0b11110000,
	0b10000000,
	0b11110000,
	0b10000000,
	0b10000000,
];
