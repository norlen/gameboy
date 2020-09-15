use super::memory::MemoryBus;
use super::registers::{Flags, Registers};

const PREFIXED_INSTRUCTION: u8 = 0xcb;

#[derive(Default)]
pub struct CPU {
    registers: Registers,
    pc: u16,
    bus: MemoryBus,
}

impl CPU {
    fn step(&mut self) {
        let instruction = self.bus.read_u8(self.pc);

        let next_pc = if instruction != PREFIXED_INSTRUCTION {
            self.execute(instruction)
        } else {
            let instruction = self.bus.read_u8(self.pc + 1);
            self.execute_prefixed(instruction)
        };

        self.pc = next_pc;
    }

    /// And `a` with `value` and store the result in `a`.
    /// Flags: Z, N0, H1, C0.
    fn and(&mut self, value: u8) {
        self.registers.a &= value;
        self.registers.f.setc(self.registers.a == 0, Flags::ZERO);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.set(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::CARRY);
    }

    fn or(&mut self, value: u8) {
        self.registers.a |= value;
        self.registers.f.setc(self.registers.a == 0, Flags::ZERO);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.set(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::CARRY);
    }

    fn xor(&mut self, value: u8) {
        self.registers.a ^= value;
        self.registers.f.0 = 0u8;
        self.registers.f.setc(self.registers.a == 0, Flags::ZERO);
    }

    fn add(&mut self, value: u8) -> u8 {
        self.add_with_carry(value, 0)
    }

    fn adc(&mut self, value: u8) -> u8 {
        self.add_with_carry(value, self.registers.f.carry())
    }

    fn sub(&mut self, value: u8) -> u8 {
        let (new_value, overflow) = self.registers.a.overflowing_sub(value);

        let half_carry = hc_borrow(self.registers.a, value);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.setc(overflow, Flags::CARRY);
        new_value
    }

    fn sbc(&mut self, value: u8) -> u8 {
        let carry = value + self.registers.f.carry();
        let (new_value, overflow) = self.registers.a.overflowing_sub(value);
        let (new_value, overflow2) = new_value.overflowing_sub(carry);

        let half_carry = (self.registers.a & 0xF) < (value & 0xF) + carry;
        let overflow = overflow || overflow2;
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.setc(overflow, Flags::CARRY);
        new_value
    }

    /// Complement carry flag inverts the carry flag.
    fn ccf(&mut self) {
        let carry = self.registers.f.carry() != 0;
        self.registers.f.setc(carry, Flags::CARRY);
    }

    /// Complement accumulator (A = ~A)
    fn cpl(&mut self) {
        self.registers.a = !self.registers.a;
    }

    /// Set carry flag
    fn scf(&mut self) {
        self.registers.f.set(Flags::CARRY);
    }

    /// Subtract `a` with `value` and set flags, but don't store the results.
    fn cp(&mut self, value: u8) {
        let _ = self.sub(value);
    }

    fn dec_u8(&mut self, value: u8) -> u8 {
        let new_value = value.wrapping_sub(1);

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        let half_carry = (value & 0xf) < (new_value & 0xf);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        new_value
    }

    fn dec_u16(&mut self, value: u16) -> u16 {
        value.wrapping_sub(1)
        // No flags affected
    }

    fn inc_u8(&mut self, value: u8) -> u8 {
        let new_value = value.wrapping_add(value);

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        let half_carry = hc_overflow(self.registers.a, value);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        new_value
    }

    fn inc_u16(&mut self, value: u16) -> u16 {
        value.wrapping_add(1)
        // No flags affected
    }

    fn add_with_carry(&mut self, value: u8, carry: u8) -> u8 {
        let (new_value, did_overflow) = self.registers.a.overflowing_add(value + carry);

        // Set flags
        let half_carry = hc_overflow(self.registers.a, value);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.setc(did_overflow, Flags::CARRY);

        new_value
    }

    fn add_hl_r16(&mut self, value: u16) {
        let reg = self.registers.hl();
        let (new_value, overflow) = reg.overflowing_add(value);

        let half_carry = hc_overflow_u16(reg, value);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.setc(overflow, Flags::CARRY);

        self.registers.set_hl(new_value);
    }
    fn execute(&mut self, instruction: u8) -> u16 {
        match instruction {
            // 0x0X
            0x00 => {} // NOP
            0x01 => todo!("unimplemented instruction"),
            0x02 => todo!("unimplemented instruction"),
            0x03 => {
                let value = self.inc_u16(self.registers.bc());
                self.registers.set_bc(value)
            }
            0x04 => self.registers.b = self.inc_u8(self.registers.b),
            0x05 => self.registers.b = self.dec_u8(self.registers.b),
            0x06 => todo!("unimplemented instruction"),
            0x07 => todo!("unimplemented instruction"),
            0x08 => todo!("unimplemented instruction"),
            0x09 => self.add_hl_r16(self.registers.bc()), // ADD HL, BC
            0x0a => todo!("unimplemented instruction"),
            0x0b => {
                let value = self.dec_u16(self.registers.bc());
                self.registers.set_bc(value)
            }
            0x0c => self.registers.c = self.inc_u8(self.registers.c),
            0x0d => self.registers.c = self.dec_u8(self.registers.c),
            0x0e => todo!("unimplemented instruction"),
            0x0f => todo!("unimplemented instruction"),

            // 0x1X
            0x10 => todo!("unimplemented instruction"),
            0x11 => todo!("unimplemented instruction"),
            0x12 => todo!("unimplemented instruction"),
            0x13 => {
                let value = self.inc_u16(self.registers.de());
                self.registers.set_de(value)
            }
            0x14 => self.registers.d = self.inc_u8(self.registers.d),
            0x15 => self.registers.d = self.dec_u8(self.registers.d),
            0x16 => todo!("unimplemented instruction"),
            0x17 => todo!("unimplemented instruction"),
            0x18 => todo!("unimplemented instruction"),
            0x19 => self.add_hl_r16(self.registers.de()), // ADD HL, DE
            0x1a => todo!("unimplemented instruction"),
            0x1b => {
                let value = self.dec_u16(self.registers.de());
                self.registers.set_de(value)
            }
            0x1c => self.registers.e = self.inc_u8(self.registers.e), // INC E
            0x1d => self.registers.e = self.dec_u8(self.registers.e), // DEC E
            0x1e => todo!("unimplemented instruction"),
            0x1f => todo!("unimplemented instruction"),

            // 0x2X
            0x20 => todo!("unimplemented instruction"),
            0x21 => todo!("unimplemented instruction"),
            0x22 => todo!("unimplemented instruction"),
            0x23 => {
                let value = self.inc_u16(self.registers.hl());
                self.registers.set_hl(value)
            }
            0x24 => self.registers.h = self.inc_u8(self.registers.h),
            0x25 => self.registers.h = self.dec_u8(self.registers.h),
            0x26 => todo!("unimplemented instruction"),
            0x27 => todo!("unimplemented instruction"),
            0x28 => todo!("unimplemented instruction"),
            0x29 => self.add_hl_r16(self.registers.hl()), // ADD HL, HL
            0x2a => todo!("unimplemented instruction"),
            0x2b => {
                let value = self.dec_u16(self.registers.hl());
                self.registers.set_hl(value)
            }
            0x2c => self.registers.l = self.inc_u8(self.registers.l), // INC L
            0x2d => self.registers.l = self.dec_u8(self.registers.l), // DEC L
            0x2e => todo!("unimplemented instruction"),
            0x2f => todo!("unimplemented instruction"),

            // 0x3X
            0x30 => todo!("unimplemented instruction"),
            0x31 => todo!("unimplemented instruction"),
            0x32 => todo!("unimplemented instruction"),
            0x33 => self.registers.sp = self.inc_u16(self.registers.sp),
            0x34 => todo!("unimplemented instruction"), // INC (HL)
            0x35 => todo!("unimplemented instruction"), // DEC (HL)
            0x36 => todo!("unimplemented instruction"),
            0x37 => self.scf(),
            0x38 => todo!("unimplemented instruction"),
            0x39 => self.add_hl_r16(self.registers.sp), // ADD HL, SP
            0x3a => todo!("unimplemented instruction"),
            0x3b => self.registers.sp = self.dec_u16(self.registers.sp),
            0x3c => self.registers.a = self.inc_u8(self.registers.a), // INC A
            0x3d => self.registers.a = self.dec_u8(self.registers.a), // DEC A
            0x3e => self.cpl(),
            0x3f => self.ccf(), // CCF

            // 0x4X
            0x40 => todo!("unimplemented instruction"),
            0x41 => todo!("unimplemented instruction"),
            0x42 => todo!("unimplemented instruction"),
            0x43 => todo!("unimplemented instruction"),
            0x44 => todo!("unimplemented instruction"),
            0x45 => todo!("unimplemented instruction"),
            0x46 => todo!("unimplemented instruction"),
            0x47 => todo!("unimplemented instruction"),
            0x48 => todo!("unimplemented instruction"),
            0x49 => todo!("unimplemented instruction"),
            0x4a => todo!("unimplemented instruction"),
            0x4b => todo!("unimplemented instruction"),
            0x4c => todo!("unimplemented instruction"),
            0x4d => todo!("unimplemented instruction"),
            0x4e => todo!("unimplemented instruction"),
            0x4f => todo!("unimplemented instruction"),

            0x50 => todo!("unimplemented instruction"),
            0x51 => todo!("unimplemented instruction"),
            0x52 => todo!("unimplemented instruction"),
            0x53 => todo!("unimplemented instruction"),
            0x54 => todo!("unimplemented instruction"),
            0x55 => todo!("unimplemented instruction"),
            0x56 => todo!("unimplemented instruction"),
            0x57 => todo!("unimplemented instruction"),
            0x58 => todo!("unimplemented instruction"),
            0x59 => todo!("unimplemented instruction"),
            0x5a => todo!("unimplemented instruction"),
            0x5b => todo!("unimplemented instruction"),
            0x5c => todo!("unimplemented instruction"),
            0x5d => todo!("unimplemented instruction"),
            0x5e => todo!("unimplemented instruction"),
            0x5f => todo!("unimplemented instruction"),

            0x60 => todo!("unimplemented instruction"),
            0x61 => todo!("unimplemented instruction"),
            0x62 => todo!("unimplemented instruction"),
            0x63 => todo!("unimplemented instruction"),
            0x64 => todo!("unimplemented instruction"),
            0x65 => todo!("unimplemented instruction"),
            0x66 => todo!("unimplemented instruction"),
            0x67 => todo!("unimplemented instruction"),
            0x68 => todo!("unimplemented instruction"),
            0x69 => todo!("unimplemented instruction"),
            0x6a => todo!("unimplemented instruction"),
            0x6b => todo!("unimplemented instruction"),
            0x6c => todo!("unimplemented instruction"),
            0x6d => todo!("unimplemented instruction"),
            0x6e => todo!("unimplemented instruction"),
            0x6f => todo!("unimplemented instruction"),

            0x70 => todo!("unimplemented instruction"),
            0x71 => todo!("unimplemented instruction"),
            0x72 => todo!("unimplemented instruction"),
            0x73 => todo!("unimplemented instruction"),
            0x74 => todo!("unimplemented instruction"),
            0x75 => todo!("unimplemented instruction"),
            0x76 => todo!("unimplemented instruction"),
            0x77 => todo!("unimplemented instruction"),
            0x78 => todo!("unimplemented instruction"),
            0x79 => todo!("unimplemented instruction"),
            0x7a => todo!("unimplemented instruction"),
            0x7b => todo!("unimplemented instruction"),
            0x7c => todo!("unimplemented instruction"),
            0x7d => todo!("unimplemented instruction"),
            0x7e => todo!("unimplemented instruction"),
            0x7f => todo!("unimplemented instruction"),

            0x80 => self.registers.a = self.add(self.registers.b),
            0x81 => self.registers.a = self.add(self.registers.c),
            0x82 => self.registers.a = self.add(self.registers.d),
            0x83 => self.registers.a = self.add(self.registers.e),
            0x84 => self.registers.a = self.add(self.registers.h),
            0x85 => self.registers.a = self.add(self.registers.l),
            0x86 => self.registers.a = self.add(self.bus.read_u8(self.registers.hl())), // ADD A, (HL)
            0x87 => self.registers.a = self.add(self.registers.a),

            0x88 => self.registers.a = self.adc(self.registers.b),
            0x89 => self.registers.a = self.adc(self.registers.c),
            0x8a => self.registers.a = self.adc(self.registers.d),
            0x8b => self.registers.a = self.adc(self.registers.e),
            0x8c => self.registers.a = self.adc(self.registers.h),
            0x8d => self.registers.a = self.adc(self.registers.l),
            0x8e => self.registers.a = self.adc(self.bus.read_u8(self.registers.hl())), // ADC A, (HL)
            0x8f => self.registers.a = self.adc(self.registers.a),

            0x90 => self.registers.a = self.sub(self.registers.b),
            0x91 => self.registers.a = self.sub(self.registers.c),
            0x92 => self.registers.a = self.sub(self.registers.d),
            0x93 => self.registers.a = self.sub(self.registers.e),
            0x94 => self.registers.a = self.sub(self.registers.h),
            0x95 => self.registers.a = self.sub(self.registers.l),
            0x96 => self.registers.a = self.sub(self.bus.read_u8(self.registers.hl())), // SUB (HL)
            0x97 => self.registers.a = self.sub(self.registers.a),

            0x98 => self.registers.a = self.sbc(self.registers.b),
            0x99 => self.registers.a = self.sbc(self.registers.c),
            0x9a => self.registers.a = self.sbc(self.registers.d),
            0x9b => self.registers.a = self.sbc(self.registers.e),
            0x9c => self.registers.a = self.sbc(self.registers.h),
            0x9d => self.registers.a = self.sbc(self.registers.l),
            0x9e => self.registers.a = self.sbc(self.bus.read_u8(self.registers.hl())), // SBC (HL)
            0x9f => self.registers.a = self.sbc(self.registers.a),

            0xa0 => self.and(self.registers.b),
            0xa1 => self.and(self.registers.c),
            0xa2 => self.and(self.registers.d),
            0xa3 => self.and(self.registers.e),
            0xa4 => self.and(self.registers.h),
            0xa5 => self.and(self.registers.l),
            0xa6 => self.and(self.bus.read_u8(self.registers.hl())), // AND (HL)
            0xa7 => self.and(self.registers.a),

            0xa8 => self.xor(self.registers.b),
            0xa9 => self.xor(self.registers.c),
            0xaa => self.xor(self.registers.d),
            0xab => self.xor(self.registers.e),
            0xac => self.xor(self.registers.h),
            0xad => self.xor(self.registers.l),
            0xae => self.xor(self.bus.read_u8(self.registers.hl())), // XOR (HL)
            0xaf => self.xor(self.registers.a),

            0xb0 => self.or(self.registers.b),
            0xb1 => self.or(self.registers.c),
            0xb2 => self.or(self.registers.d),
            0xb3 => self.or(self.registers.e),
            0xb4 => self.or(self.registers.h),
            0xb5 => self.or(self.registers.l),
            0xb6 => self.or(self.bus.read_u8(self.registers.hl())), // OR (HL)
            0xb7 => self.or(self.registers.a),

            0xb8 => self.cp(self.registers.b),
            0xb9 => self.cp(self.registers.c),
            0xba => self.cp(self.registers.d),
            0xbb => self.cp(self.registers.e),
            0xbc => self.cp(self.registers.h),
            0xbd => self.cp(self.registers.l),
            0xbe => self.cp(self.bus.read_u8(self.registers.hl())), // CP (HL)
            0xbf => self.cp(self.registers.a),

            0xc0 => todo!("unimplemented instruction"),
            0xc1 => todo!("unimplemented instruction"),
            0xc2 => todo!("unimplemented instruction"),
            0xc3 => todo!("unimplemented instruction"),
            0xc4 => todo!("unimplemented instruction"),
            0xc5 => todo!("unimplemented instruction"),
            0xc6 => todo!("unimplemented instruction"),
            0xc7 => todo!("unimplemented instruction"),
            0xc8 => todo!("unimplemented instruction"),
            0xc9 => todo!("unimplemented instruction"),
            0xca => todo!("unimplemented instruction"),
            0xcb => todo!("unimplemented instruction"),
            0xcc => todo!("unimplemented instruction"),
            0xcd => todo!("unimplemented instruction"),
            0xce => todo!("unimplemented instruction"),
            0xcf => todo!("unimplemented instruction"),

            0xd0 => todo!("unimplemented instruction"),
            0xd1 => todo!("unimplemented instruction"),
            0xd2 => todo!("unimplemented instruction"),
            0xd3 => todo!("unimplemented instruction"),
            0xd4 => todo!("unimplemented instruction"),
            0xd5 => todo!("unimplemented instruction"),
            0xd6 => todo!("unimplemented instruction"),
            0xd7 => todo!("unimplemented instruction"),
            0xd8 => todo!("unimplemented instruction"),
            0xd9 => todo!("unimplemented instruction"),
            0xda => todo!("unimplemented instruction"),
            0xdb => todo!("unimplemented instruction"),
            0xdc => todo!("unimplemented instruction"),
            0xdd => todo!("unimplemented instruction"),
            0xde => todo!("unimplemented instruction"),
            0xdf => todo!("unimplemented instruction"),

            0xe0 => todo!("unimplemented instruction"),
            0xe1 => todo!("unimplemented instruction"),
            0xe2 => todo!("unimplemented instruction"),
            0xe3 => todo!("unimplemented instruction"),
            0xe4 => todo!("unimplemented instruction"),
            0xe5 => todo!("unimplemented instruction"),
            0xe6 => todo!("unimplemented instruction"),
            0xe7 => todo!("unimplemented instruction"),
            0xe8 => todo!("unimplemented instruction"),
            0xe9 => todo!("unimplemented instruction"),
            0xea => todo!("unimplemented instruction"),
            0xeb => todo!("unimplemented instruction"),
            0xec => todo!("unimplemented instruction"),
            0xed => todo!("unimplemented instruction"),
            0xee => todo!("unimplemented instruction"),
            0xef => todo!("unimplemented instruction"),

            0xf0 => todo!("unimplemented instruction"),
            0xf1 => todo!("unimplemented instruction"),
            0xf2 => todo!("unimplemented instruction"),
            0xf3 => todo!("unimplemented instruction"),
            0xf4 => todo!("unimplemented instruction"),
            0xf5 => todo!("unimplemented instruction"),
            0xf6 => todo!("unimplemented instruction"),
            0xf7 => todo!("unimplemented instruction"),
            0xf8 => todo!("unimplemented instruction"),
            0xf9 => todo!("unimplemented instruction"),
            0xfa => todo!("unimplemented instruction"),
            0xfb => todo!("unimplemented instruction"),
            0xfc => todo!("unimplemented instruction"),
            0xfd => todo!("unimplemented instruction"),
            0xfe => todo!("unimplemented instruction"),
            0xff => todo!("unimplemented instruction"),
        }

        todo!()
    }

    // Bit shift instructions

    // Rotate bits in register left through carry.
    fn rl(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn rla(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn rlc(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn rr(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn rra(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn rrc(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn sla(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn sra(&mut self, value: u8) -> u8 {
        todo!()
    }

    fn srl(&mut self, value: u8) -> u8 {
        todo!()
    }

    // Bit operation instructions.

    // Test bit in register, set zero flag is bit is not set.
    fn bit(&mut self, bit: u8, value: u8) {
        let mask = 1u8 << bit;
        let masked = value & mask;
        self.registers.f.setc(masked != 0, Flags::ZERO);
    }

    // Clear bit in in register.
    fn res(&mut self, bit: u8, value: u8) -> u8 {
        value & !(0b1 << bit)
    }

    // Set bit in in register.
    fn set(&mut self, bit: u8, value: u8) -> u8 {
        value | (0b1 << bit)
    }

    // Swap the upper and lower 4-bits in the register
    fn swap(&mut self, value: u8) -> u8 {
        let res = value << 4;
        res | (value >> 4)
    }

    fn execute_prefixed(&mut self, instruction: u8) -> u16 {
        match instruction {
            0x00 => todo!("unimplemented prefix instruction"),
            0x01 => todo!("unimplemented prefix instruction"),
            0x02 => todo!("unimplemented prefix instruction"),
            0x03 => todo!("unimplemented prefix instruction"),
            0x04 => todo!("unimplemented prefix instruction"),
            0x05 => todo!("unimplemented prefix instruction"),
            0x06 => todo!("unimplemented prefix instruction"),
            0x07 => todo!("unimplemented prefix instruction"),
            0x08 => todo!("unimplemented prefix instruction"),
            0x09 => todo!("unimplemented prefix instruction"),
            0x0a => todo!("unimplemented prefix instruction"),
            0x0b => todo!("unimplemented prefix instruction"),
            0x0c => todo!("unimplemented prefix instruction"),
            0x0d => todo!("unimplemented prefix instruction"),
            0x0e => todo!("unimplemented prefix instruction"),
            0x0f => todo!("unimplemented prefix instruction"),

            0x10 => todo!("unimplemented prefix instruction"),
            0x11 => todo!("unimplemented prefix instruction"),
            0x12 => todo!("unimplemented prefix instruction"),
            0x13 => todo!("unimplemented prefix instruction"),
            0x14 => todo!("unimplemented prefix instruction"),
            0x15 => todo!("unimplemented prefix instruction"),
            0x16 => todo!("unimplemented prefix instruction"),
            0x17 => todo!("unimplemented prefix instruction"),
            0x18 => todo!("unimplemented prefix instruction"),
            0x19 => todo!("unimplemented prefix instruction"),
            0x1a => todo!("unimplemented prefix instruction"),
            0x1b => todo!("unimplemented prefix instruction"),
            0x1c => todo!("unimplemented prefix instruction"),
            0x1d => todo!("unimplemented prefix instruction"),
            0x1e => todo!("unimplemented prefix instruction"),
            0x1f => todo!("unimplemented prefix instruction"),

            0x20 => todo!("unimplemented prefix instruction"),
            0x21 => todo!("unimplemented prefix instruction"),
            0x22 => todo!("unimplemented prefix instruction"),
            0x23 => todo!("unimplemented prefix instruction"),
            0x24 => todo!("unimplemented prefix instruction"),
            0x25 => todo!("unimplemented prefix instruction"),
            0x26 => todo!("unimplemented prefix instruction"),
            0x27 => todo!("unimplemented prefix instruction"),
            0x28 => todo!("unimplemented prefix instruction"),
            0x29 => todo!("unimplemented prefix instruction"),
            0x2a => todo!("unimplemented prefix instruction"),
            0x2b => todo!("unimplemented prefix instruction"),
            0x2c => todo!("unimplemented prefix instruction"),
            0x2d => todo!("unimplemented prefix instruction"),
            0x2e => todo!("unimplemented prefix instruction"),
            0x2f => todo!("unimplemented prefix instruction"),

            0x30 => todo!("unimplemented prefix instruction"),
            0x31 => todo!("unimplemented prefix instruction"),
            0x32 => todo!("unimplemented prefix instruction"),
            0x33 => todo!("unimplemented prefix instruction"),
            0x34 => todo!("unimplemented prefix instruction"),
            0x35 => todo!("unimplemented prefix instruction"),
            0x36 => todo!("unimplemented prefix instruction"),
            0x37 => todo!("unimplemented prefix instruction"),
            0x38 => todo!("unimplemented prefix instruction"),
            0x39 => todo!("unimplemented prefix instruction"),
            0x3a => todo!("unimplemented prefix instruction"),
            0x3b => todo!("unimplemented prefix instruction"),
            0x3c => todo!("unimplemented prefix instruction"),
            0x3d => todo!("unimplemented prefix instruction"),
            0x3e => todo!("unimplemented prefix instruction"),
            0x3f => todo!("unimplemented prefix instruction"),

            0x40 => todo!("unimplemented prefix instruction"),
            0x41 => todo!("unimplemented prefix instruction"),
            0x42 => todo!("unimplemented prefix instruction"),
            0x43 => todo!("unimplemented prefix instruction"),
            0x44 => todo!("unimplemented prefix instruction"),
            0x45 => todo!("unimplemented prefix instruction"),
            0x46 => todo!("unimplemented prefix instruction"),
            0x47 => todo!("unimplemented prefix instruction"),
            0x48 => todo!("unimplemented prefix instruction"),
            0x49 => todo!("unimplemented prefix instruction"),
            0x4a => todo!("unimplemented prefix instruction"),
            0x4b => todo!("unimplemented prefix instruction"),
            0x4c => todo!("unimplemented prefix instruction"),
            0x4d => todo!("unimplemented prefix instruction"),
            0x4e => todo!("unimplemented prefix instruction"),
            0x4f => todo!("unimplemented prefix instruction"),

            0x50 => todo!("unimplemented prefix instruction"),
            0x51 => todo!("unimplemented prefix instruction"),
            0x52 => todo!("unimplemented prefix instruction"),
            0x53 => todo!("unimplemented prefix instruction"),
            0x54 => todo!("unimplemented prefix instruction"),
            0x55 => todo!("unimplemented prefix instruction"),
            0x56 => todo!("unimplemented prefix instruction"),
            0x57 => todo!("unimplemented prefix instruction"),
            0x58 => todo!("unimplemented prefix instruction"),
            0x59 => todo!("unimplemented prefix instruction"),
            0x5a => todo!("unimplemented prefix instruction"),
            0x5b => todo!("unimplemented prefix instruction"),
            0x5c => todo!("unimplemented prefix instruction"),
            0x5d => todo!("unimplemented prefix instruction"),
            0x5e => todo!("unimplemented prefix instruction"),
            0x5f => todo!("unimplemented prefix instruction"),

            0x60 => todo!("unimplemented prefix instruction"),
            0x61 => todo!("unimplemented prefix instruction"),
            0x62 => todo!("unimplemented prefix instruction"),
            0x63 => todo!("unimplemented prefix instruction"),
            0x64 => todo!("unimplemented prefix instruction"),
            0x65 => todo!("unimplemented prefix instruction"),
            0x66 => todo!("unimplemented prefix instruction"),
            0x67 => todo!("unimplemented prefix instruction"),
            0x68 => todo!("unimplemented prefix instruction"),
            0x69 => todo!("unimplemented prefix instruction"),
            0x6a => todo!("unimplemented prefix instruction"),
            0x6b => todo!("unimplemented prefix instruction"),
            0x6c => todo!("unimplemented prefix instruction"),
            0x6d => todo!("unimplemented prefix instruction"),
            0x6e => todo!("unimplemented prefix instruction"),
            0x6f => todo!("unimplemented prefix instruction"),

            0x70 => todo!("unimplemented prefix instruction"),
            0x71 => todo!("unimplemented prefix instruction"),
            0x72 => todo!("unimplemented prefix instruction"),
            0x73 => todo!("unimplemented prefix instruction"),
            0x74 => todo!("unimplemented prefix instruction"),
            0x75 => todo!("unimplemented prefix instruction"),
            0x76 => todo!("unimplemented prefix instruction"),
            0x77 => todo!("unimplemented prefix instruction"),
            0x78 => todo!("unimplemented prefix instruction"),
            0x79 => todo!("unimplemented prefix instruction"),
            0x7a => todo!("unimplemented prefix instruction"),
            0x7b => todo!("unimplemented prefix instruction"),
            0x7c => todo!("unimplemented prefix instruction"),
            0x7d => todo!("unimplemented prefix instruction"),
            0x7e => todo!("unimplemented prefix instruction"),
            0x7f => todo!("unimplemented prefix instruction"),

            0x80 => todo!("unimplemented prefix instruction"),
            0x81 => todo!("unimplemented prefix instruction"),
            0x82 => todo!("unimplemented prefix instruction"),
            0x83 => todo!("unimplemented prefix instruction"),
            0x84 => todo!("unimplemented prefix instruction"),
            0x85 => todo!("unimplemented prefix instruction"),
            0x86 => todo!("unimplemented prefix instruction"),
            0x87 => todo!("unimplemented prefix instruction"),
            0x88 => todo!("unimplemented prefix instruction"),
            0x89 => todo!("unimplemented prefix instruction"),
            0x8a => todo!("unimplemented prefix instruction"),
            0x8b => todo!("unimplemented prefix instruction"),
            0x8c => todo!("unimplemented prefix instruction"),
            0x8d => todo!("unimplemented prefix instruction"),
            0x8e => todo!("unimplemented prefix instruction"),
            0x8f => todo!("unimplemented prefix instruction"),

            0x90 => todo!("unimplemented prefix instruction"),
            0x91 => todo!("unimplemented prefix instruction"),
            0x92 => todo!("unimplemented prefix instruction"),
            0x93 => todo!("unimplemented prefix instruction"),
            0x94 => todo!("unimplemented prefix instruction"),
            0x95 => todo!("unimplemented prefix instruction"),
            0x96 => todo!("unimplemented prefix instruction"),
            0x97 => todo!("unimplemented prefix instruction"),
            0x98 => todo!("unimplemented prefix instruction"),
            0x99 => todo!("unimplemented prefix instruction"),
            0x9a => todo!("unimplemented prefix instruction"),
            0x9b => todo!("unimplemented prefix instruction"),
            0x9c => todo!("unimplemented prefix instruction"),
            0x9d => todo!("unimplemented prefix instruction"),
            0x9e => todo!("unimplemented prefix instruction"),
            0x9f => todo!("unimplemented prefix instruction"),

            0xa0 => todo!("unimplemented prefix instruction"),
            0xa1 => todo!("unimplemented prefix instruction"),
            0xa2 => todo!("unimplemented prefix instruction"),
            0xa3 => todo!("unimplemented prefix instruction"),
            0xa4 => todo!("unimplemented prefix instruction"),
            0xa5 => todo!("unimplemented prefix instruction"),
            0xa6 => todo!("unimplemented prefix instruction"),
            0xa7 => todo!("unimplemented prefix instruction"),
            0xa8 => todo!("unimplemented prefix instruction"),
            0xa9 => todo!("unimplemented prefix instruction"),
            0xaa => todo!("unimplemented prefix instruction"),
            0xab => todo!("unimplemented prefix instruction"),
            0xac => todo!("unimplemented prefix instruction"),
            0xad => todo!("unimplemented prefix instruction"),
            0xae => todo!("unimplemented prefix instruction"),
            0xaf => todo!("unimplemented prefix instruction"),

            0xb0 => todo!("unimplemented prefix instruction"),
            0xb1 => todo!("unimplemented prefix instruction"),
            0xb2 => todo!("unimplemented prefix instruction"),
            0xb3 => todo!("unimplemented prefix instruction"),
            0xb4 => todo!("unimplemented prefix instruction"),
            0xb5 => todo!("unimplemented prefix instruction"),
            0xb6 => todo!("unimplemented prefix instruction"),
            0xb7 => todo!("unimplemented prefix instruction"),
            0xb8 => todo!("unimplemented prefix instruction"),
            0xb9 => todo!("unimplemented prefix instruction"),
            0xba => todo!("unimplemented prefix instruction"),
            0xbb => todo!("unimplemented prefix instruction"),
            0xbc => todo!("unimplemented prefix instruction"),
            0xbd => todo!("unimplemented prefix instruction"),
            0xbe => todo!("unimplemented prefix instruction"),
            0xbf => todo!("unimplemented prefix instruction"),

            0xc0 => todo!("unimplemented prefix instruction"),
            0xc1 => todo!("unimplemented prefix instruction"),
            0xc2 => todo!("unimplemented prefix instruction"),
            0xc3 => todo!("unimplemented prefix instruction"),
            0xc4 => todo!("unimplemented prefix instruction"),
            0xc5 => todo!("unimplemented prefix instruction"),
            0xc6 => todo!("unimplemented prefix instruction"),
            0xc7 => todo!("unimplemented prefix instruction"),
            0xc8 => todo!("unimplemented prefix instruction"),
            0xc9 => todo!("unimplemented prefix instruction"),
            0xca => todo!("unimplemented prefix instruction"),
            0xcb => todo!("unimplemented prefix instruction"),
            0xcc => todo!("unimplemented prefix instruction"),
            0xcd => todo!("unimplemented prefix instruction"),
            0xce => todo!("unimplemented prefix instruction"),
            0xcf => todo!("unimplemented prefix instruction"),

            0xd0 => todo!("unimplemented prefix instruction"),
            0xd1 => todo!("unimplemented prefix instruction"),
            0xd2 => todo!("unimplemented prefix instruction"),
            0xd3 => todo!("unimplemented prefix instruction"),
            0xd4 => todo!("unimplemented prefix instruction"),
            0xd5 => todo!("unimplemented prefix instruction"),
            0xd6 => todo!("unimplemented prefix instruction"),
            0xd7 => todo!("unimplemented prefix instruction"),
            0xd8 => todo!("unimplemented prefix instruction"),
            0xd9 => todo!("unimplemented prefix instruction"),
            0xda => todo!("unimplemented prefix instruction"),
            0xdb => todo!("unimplemented prefix instruction"),
            0xdc => todo!("unimplemented prefix instruction"),
            0xdd => todo!("unimplemented prefix instruction"),
            0xde => todo!("unimplemented prefix instruction"),
            0xdf => todo!("unimplemented prefix instruction"),

            0xe0 => todo!("unimplemented prefix instruction"),
            0xe1 => todo!("unimplemented prefix instruction"),
            0xe2 => todo!("unimplemented prefix instruction"),
            0xe3 => todo!("unimplemented prefix instruction"),
            0xe4 => todo!("unimplemented prefix instruction"),
            0xe5 => todo!("unimplemented prefix instruction"),
            0xe6 => todo!("unimplemented prefix instruction"),
            0xe7 => todo!("unimplemented prefix instruction"),
            0xe8 => todo!("unimplemented prefix instruction"),
            0xe9 => todo!("unimplemented prefix instruction"),
            0xea => todo!("unimplemented prefix instruction"),
            0xeb => todo!("unimplemented prefix instruction"),
            0xec => todo!("unimplemented prefix instruction"),
            0xed => todo!("unimplemented prefix instruction"),
            0xee => todo!("unimplemented prefix instruction"),
            0xef => todo!("unimplemented prefix instruction"),

            0xf0 => todo!("unimplemented prefix instruction"),
            0xf1 => todo!("unimplemented prefix instruction"),
            0xf2 => todo!("unimplemented prefix instruction"),
            0xf3 => todo!("unimplemented prefix instruction"),
            0xf4 => todo!("unimplemented prefix instruction"),
            0xf5 => todo!("unimplemented prefix instruction"),
            0xf6 => todo!("unimplemented prefix instruction"),
            0xf7 => todo!("unimplemented prefix instruction"),
            0xf8 => todo!("unimplemented prefix instruction"),
            0xf9 => todo!("unimplemented prefix instruction"),
            0xfa => todo!("unimplemented prefix instruction"),
            0xfb => todo!("unimplemented prefix instruction"),
            0xfc => todo!("unimplemented prefix instruction"),
            0xfd => todo!("unimplemented prefix instruction"),
            0xfe => todo!("unimplemented prefix instruction"),
            0xff => todo!("unimplemented prefix instruction"),
        }

        todo!()
    }
}

fn hc_overflow(old: u8, value: u8) -> bool {
    (old & 0xf) + (value & 0xf) > 0xf
}

fn hc_borrow(old: u8, value: u8) -> bool {
    (old & 0xF) < (value & 0xF)
}

fn hc_overflow_u16(old: u16, value: u16) -> bool {
    let bits = 0x0fff;
    (old & bits) + (value & bits) > bits
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bit_operations() {
        let mut cpu = CPU::default();

        cpu.bit(0x1, 0x2);
        assert_eq!(cpu.registers.f.zero_is_set(), true);
        cpu.bit(0x2, 0x2);
        assert_eq!(cpu.registers.f.zero_is_set(), false);

        let r = cpu.res(0x1, 0xff);
        assert_eq!(r, 0b1111_1101);

        let r = cpu.set(0x1, 0x0);
        assert_eq!(r, 0b0000_0010);

        let r = cpu.swap(0b1010_0101);
        assert_eq!(r, 0b0101_1010);
    }
}
