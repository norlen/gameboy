use super::registers::{Flags, Registers};

pub struct CPU {
    registers: Registers,
}

impl CPU {
    fn execute(&mut self, instruction: u8) {
        match instruction {
            0x00 => todo!("unimplemented instruction"),
            0x01 => todo!("unimplemented instruction"),
            0x02 => todo!("unimplemented instruction"),
            0x03 => todo!("unimplemented instruction"),
            0x04 => todo!("unimplemented instruction"),
            0x05 => todo!("unimplemented instruction"),
            0x06 => todo!("unimplemented instruction"),
            0x07 => todo!("unimplemented instruction"),
            0x08 => todo!("unimplemented instruction"),
            0x09 => self.add_hl_r16(self.registers.bc()), // ADD HL, BC
            0x0a => todo!("unimplemented instruction"),
            0x0b => todo!("unimplemented instruction"),
            0x0c => todo!("unimplemented instruction"),
            0x0d => todo!("unimplemented instruction"),
            0x0e => todo!("unimplemented instruction"),
            0x0f => todo!("unimplemented instruction"),

            0x10 => todo!("unimplemented instruction"),
            0x11 => todo!("unimplemented instruction"),
            0x12 => todo!("unimplemented instruction"),
            0x13 => todo!("unimplemented instruction"),
            0x14 => todo!("unimplemented instruction"),
            0x15 => todo!("unimplemented instruction"),
            0x16 => todo!("unimplemented instruction"),
            0x17 => todo!("unimplemented instruction"),
            0x18 => todo!("unimplemented instruction"),
            0x19 => self.add_hl_r16(self.registers.de()), // ADD HL, DE
            0x1a => todo!("unimplemented instruction"),
            0x1b => todo!("unimplemented instruction"),
            0x1c => todo!("unimplemented instruction"),
            0x1d => todo!("unimplemented instruction"),
            0x1e => todo!("unimplemented instruction"),
            0x1f => todo!("unimplemented instruction"),

            0x20 => todo!("unimplemented instruction"),
            0x21 => todo!("unimplemented instruction"),
            0x22 => todo!("unimplemented instruction"),
            0x23 => todo!("unimplemented instruction"),
            0x24 => todo!("unimplemented instruction"),
            0x25 => todo!("unimplemented instruction"),
            0x26 => todo!("unimplemented instruction"),
            0x27 => todo!("unimplemented instruction"),
            0x28 => todo!("unimplemented instruction"),
            0x29 => self.add_hl_r16(self.registers.hl()), // ADD HL, HL
            0x2a => todo!("unimplemented instruction"),
            0x2b => todo!("unimplemented instruction"),
            0x2c => todo!("unimplemented instruction"),
            0x2d => todo!("unimplemented instruction"),
            0x2e => todo!("unimplemented instruction"),
            0x2f => todo!("unimplemented instruction"),

            0x30 => todo!("unimplemented instruction"),
            0x31 => todo!("unimplemented instruction"),
            0x32 => todo!("unimplemented instruction"),
            0x33 => todo!("unimplemented instruction"),
            0x34 => todo!("unimplemented instruction"),
            0x35 => todo!("unimplemented instruction"),
            0x36 => todo!("unimplemented instruction"),
            0x37 => todo!("unimplemented instruction"),
            0x38 => todo!("unimplemented instruction"),
            0x39 => self.add_hl_r16(self.registers.sp), // ADD HL, SP
            0x3a => todo!("unimplemented instruction"),
            0x3b => todo!("unimplemented instruction"),
            0x3c => todo!("unimplemented instruction"),
            0x3d => todo!("unimplemented instruction"),
            0x3e => todo!("unimplemented instruction"),
            0x3f => todo!("unimplemented instruction"),

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
            0x86 => todo!("unimplemented instruction"), // ADD A, (HL)
            0x87 => self.registers.a = self.add(self.registers.a),

            0x88 => self.registers.a = self.adc(self.registers.b),
            0x89 => self.registers.a = self.adc(self.registers.c),
            0x8a => self.registers.a = self.adc(self.registers.d),
            0x8b => self.registers.a = self.adc(self.registers.e),
            0x8c => self.registers.a = self.adc(self.registers.h),
            0x8d => self.registers.a = self.adc(self.registers.l),
            0x8e => todo!("unimplemented instruction"), // ADC A, (HL)
            0x8f => self.registers.a = self.adc(self.registers.a),

            0x90 => self.registers.a = self.sub(self.registers.b),
            0x91 => self.registers.a = self.sub(self.registers.c),
            0x92 => self.registers.a = self.sub(self.registers.d),
            0x93 => self.registers.a = self.sub(self.registers.e),
            0x94 => self.registers.a = self.sub(self.registers.h),
            0x95 => self.registers.a = self.sub(self.registers.l),
            0x96 => todo!("unimplemented instruction"), // SUB (HL)
            0x97 => self.registers.a = self.sub(self.registers.a),

            0x98 => self.registers.a = self.sbc(self.registers.b),
            0x99 => self.registers.a = self.sbc(self.registers.c),
            0x9a => self.registers.a = self.sbc(self.registers.d),
            0x9b => self.registers.a = self.sbc(self.registers.e),
            0x9c => self.registers.a = self.sbc(self.registers.h),
            0x9d => self.registers.a = self.sbc(self.registers.l),
            0x9e => todo!("unimplemented instruction"), // SBC (HL)
            0x9f => self.registers.a = self.sbc(self.registers.a),

            0xa0 => self.and(self.registers.b),
            0xa1 => self.and(self.registers.c),
            0xa2 => self.and(self.registers.d),
            0xa3 => self.and(self.registers.e),
            0xa4 => self.and(self.registers.h),
            0xa5 => self.and(self.registers.l),
            0xa6 => todo!("unimplemented instruction"), // AND (HL)
            0xa7 => self.and(self.registers.a),

            0xa8 => self.xor(self.registers.b),
            0xa9 => self.xor(self.registers.c),
            0xaa => self.xor(self.registers.d),
            0xab => self.xor(self.registers.e),
            0xac => self.xor(self.registers.h),
            0xad => self.xor(self.registers.l),
            0xae => todo!("unimplemented instruction"), // XOR (HL)
            0xaf => self.xor(self.registers.a),

            0xb0 => self.or(self.registers.b),
            0xb1 => self.or(self.registers.c),
            0xb2 => self.or(self.registers.d),
            0xb3 => self.or(self.registers.e),
            0xb4 => self.or(self.registers.h),
            0xb5 => self.or(self.registers.l),
            0xb6 => todo!("unimplemented instruction"), // OR (HL)
            0xb7 => self.or(self.registers.a),

            0xb8 => self.cp(self.registers.b),
            0xb9 => self.cp(self.registers.c),
            0xba => self.cp(self.registers.d),
            0xbb => self.cp(self.registers.e),
            0xbc => self.cp(self.registers.h),
            0xbd => self.cp(self.registers.l),
            0xbe => todo!("unimplemented instruction"), // CP (HL)
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

        let half_carry = (self.registers.a & 0xF) < (value & 0xF);
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

    /// Subtract `a` with `value` and set flags, but don't store the results.
    fn cp(&mut self, value: u8) {
        let _ = self.sub(value);
    }

    fn add_with_carry(&mut self, value: u8, carry: u8) -> u8 {
        let (new_value, did_overflow) = self.registers.a.overflowing_add(value + carry);

        // Set flags
        let half_carry = (self.registers.a & 0xf) + (value & 0xf) > 0xf;
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.setc(did_overflow, Flags::CARRY);

        new_value
    }

    fn add_hl_r16(&mut self, value: u16) {
        let (new_value, overflow) = self.registers.hl().overflowing_add(value);

        let bits = 0x0fff;
        let half_carry = (self.registers.hl() & bits) + (value & bits) > bits;
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.setc(overflow, Flags::CARRY);

        self.registers.set_hl(new_value);
    }
}
