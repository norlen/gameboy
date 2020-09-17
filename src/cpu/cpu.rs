use super::memory::MemoryBus;
use super::registers::{Flags, Registers};

const PREFIXED_INSTRUCTION: u8 = 0xcb;

#[derive(Default)]
pub struct CPU {
    registers: Registers,
    pc: u16,
    bus: MemoryBus,
}

enum Register16 {
    BC,
    DE,
    HL,
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

    // Misc instructions.
    fn halt(&self) {
        todo!()
    }

    fn nop(&self) {
        todo!()
    }

    fn stop(&self) {
        todo!()
    }

    fn daa(&self) {
        todo!()
    }

    // Jump instructions.

    // Stack instructions.

    // Pushes a 16 big register to the stack.
    fn push_r16(&mut self, value: u16) {
        self.registers.sp -= 2;
        self.bus.write_u16(self.registers.sp, value);
    }

    fn pop_r16(&mut self) -> u16 {
        let value = self.bus.read_u16(self.registers.sp);
        self.registers.sp += 2;
        value
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

    fn dec_r16(&mut self, reg: Register16) {
        match reg {
            Register16::BC => {
                let value = self.dec_u16(self.registers.bc());
                self.registers.set_bc(value);
            }
            Register16::DE => {
                let value = self.dec_u16(self.registers.de());
                self.registers.set_de(value);
            }
            Register16::HL => {
                let value = self.dec_u16(self.registers.hl());
                self.registers.set_hl(value);
            }
        }
    }

    fn dec_a8(&mut self, addr: u16) {
        let value = self.dec_u8(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn inc_u8(&mut self, value: u8) -> u8 {
        let new_value = value.wrapping_add(value);

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        let half_carry = hc_overflow(self.registers.a, value);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        new_value
    }

    fn inc_a8(&mut self, addr: u16) {
        let value = self.inc_u8(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn inc_u16(&mut self, value: u16) -> u16 {
        value.wrapping_add(1)
        // No flags affected
    }

    fn inc_r16(&mut self, reg: Register16) {
        match reg {
            Register16::BC => {
                let value = self.inc_u16(self.registers.bc());
                self.registers.set_bc(value);
            }
            Register16::DE => {
                let value = self.inc_u16(self.registers.de());
                self.registers.set_de(value);
            }
            Register16::HL => {
                let value = self.inc_u16(self.registers.hl());
                self.registers.set_hl(value);
            }
        }
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

    // LD (r16), r8
    fn ld_a_r8(&mut self, addr: u16, value: u8) {
        self.bus.write_u8(addr, value);
    }

    // LD r8, d8
    fn ld_r8_d8(&self) -> u8 {
        self.bus.read_u8(self.pc + 1)
    }

    // LD r8, (r16)
    fn ld_r8_a16(&self, addr: u16) -> u8 {
        self.bus.read_u8(addr)
    }

    // LD r16, d16
    fn ld_r16_d16(&self) -> u16 {
        self.bus.read_u16(self.pc + 1)
    }

    // LD (a16), SP
    fn ld_a16_sp(&mut self) {
        // Store SP & $FF at address n16 and SP >> 8 at address n16 + 1.
        let addr = self.bus.read_u16(self.pc + 1);
        let lower = self.registers.sp as u8;
        let upper = (self.registers.sp >> 8) as u8;
        self.bus.write_u8(addr, lower);
        self.bus.write_u8(addr, upper);
    }

    fn execute(&mut self, instruction: u8) -> u16 {
        match instruction {
            // 0x0X
            0x00 => self.nop(),
            0x01 => self.registers.set_bc(self.ld_r16_d16()),
            0x02 => self.ld_a_r8(self.registers.bc(), self.registers.a),
            0x03 => self.inc_r16(Register16::BC),
            0x04 => self.registers.b = self.inc_u8(self.registers.b),
            0x05 => self.registers.b = self.dec_u8(self.registers.b),
            0x06 => self.registers.b = self.ld_r8_d8(),
            0x07 => self.rlca(),
            0x08 => self.ld_a16_sp(),
            0x09 => self.add_hl_r16(self.registers.bc()), // ADD HL, BC
            0x0a => self.registers.a = self.ld_r8_a16(self.registers.bc()),
            0x0b => self.dec_r16(Register16::BC),
            0x0c => self.registers.c = self.inc_u8(self.registers.c),
            0x0d => self.registers.c = self.dec_u8(self.registers.c),
            0x0e => self.registers.c = self.ld_r8_d8(),
            0x0f => self.rrca(),

            // 0x1X
            0x10 => self.stop(),
            0x11 => self.registers.set_de(self.ld_r16_d16()),
            0x12 => self.ld_a_r8(self.registers.de(), self.registers.a),
            0x13 => self.inc_r16(Register16::DE),
            0x14 => self.registers.d = self.inc_u8(self.registers.d),
            0x15 => self.registers.d = self.dec_u8(self.registers.d),
            0x16 => self.registers.d = self.ld_r8_d8(),
            0x17 => self.rla(),
            0x18 => todo!("unimplemented instruction"), // JR r8
            0x19 => self.add_hl_r16(self.registers.de()), // ADD HL, DE
            0x1a => self.registers.a = self.ld_r8_a16(self.registers.de()),
            0x1b => self.dec_r16(Register16::DE),
            0x1c => self.registers.e = self.inc_u8(self.registers.e), // INC E
            0x1d => self.registers.e = self.dec_u8(self.registers.e), // DEC E
            0x1e => self.registers.e = self.ld_r8_d8(),
            0x1f => self.rra(),

            // 0x2X
            0x20 => todo!("unimplemented instruction"), // JR NZ, r8
            0x21 => self.registers.set_hl(self.ld_r16_d16()),
            0x22 => {
                let addr = self.registers.hl();
                self.ld_a_r8(addr, self.registers.a);
                self.registers.set_hl(addr + 1);
            }
            0x23 => self.inc_r16(Register16::HL),
            0x24 => self.registers.h = self.inc_u8(self.registers.h),
            0x25 => self.registers.h = self.dec_u8(self.registers.h),
            0x26 => self.registers.h = self.ld_r8_d8(),
            0x27 => self.daa(),
            0x28 => todo!("unimplemented instruction"), // JR Z, r8
            0x29 => self.add_hl_r16(self.registers.hl()), // ADD HL, HL
            0x2a => {
                // LA A, (HL+)
                let addr = self.registers.hl();
                self.registers.a = self.ld_r8_a16(addr);
                self.registers.set_hl(addr + 1);
            }
            0x2b => self.dec_r16(Register16::HL),
            0x2c => self.registers.l = self.inc_u8(self.registers.l), // INC L
            0x2d => self.registers.l = self.dec_u8(self.registers.l), // DEC L
            0x2e => self.registers.l = self.ld_r8_d8(),
            0x2f => self.cpl(),

            // 0x3X
            0x30 => todo!("unimplemented instruction"), // JR NC, r8
            0x31 => self.registers.sp = self.ld_r16_d16(),
            0x32 => {
                let addr = self.registers.hl();
                self.ld_a_r8(addr, self.registers.a);
                self.registers.set_hl(addr - 1);
            }
            0x33 => self.registers.sp = self.inc_u16(self.registers.sp),
            0x34 => self.inc_a8(self.registers.hl()), // INC (HL)
            0x35 => self.dec_a8(self.registers.hl()), // DEC (HL)
            0x36 => self.bus.write_u8(self.registers.hl(), self.ld_r8_d8()),
            0x37 => self.scf(),
            0x38 => todo!("unimplemented instruction"), // JR C, r8
            0x39 => self.add_hl_r16(self.registers.sp), // ADD HL, SP
            0x3a => {
                // LD A, (HL-)
                let addr = self.registers.hl();
                self.registers.a = self.ld_r8_a16(addr);
                self.registers.set_hl(addr - 1);
            }
            0x3b => self.registers.sp = self.dec_u16(self.registers.sp),
            0x3c => self.registers.a = self.inc_u8(self.registers.a), // INC A
            0x3d => self.registers.a = self.dec_u8(self.registers.a), // DEC A
            0x3e => self.registers.a = self.ld_r8_d8(),
            0x3f => self.ccf(),

            // LD B, x
            0x40 => self.registers.b = self.registers.b,
            0x41 => self.registers.b = self.registers.c,
            0x42 => self.registers.b = self.registers.d,
            0x43 => self.registers.b = self.registers.e,
            0x44 => self.registers.b = self.registers.h,
            0x45 => self.registers.b = self.registers.l,
            0x46 => self.registers.b = self.bus.read_u8(self.registers.hl()),
            0x47 => self.registers.b = self.registers.a,

            // LD C, x
            0x48 => self.registers.c = self.registers.b,
            0x49 => self.registers.c = self.registers.c,
            0x4a => self.registers.c = self.registers.d,
            0x4b => self.registers.c = self.registers.e,
            0x4c => self.registers.c = self.registers.h,
            0x4d => self.registers.c = self.registers.l,
            0x4e => self.registers.c = self.bus.read_u8(self.registers.hl()),
            0x4f => self.registers.c = self.registers.a,

            // LD D, x
            0x50 => self.registers.d = self.registers.b,
            0x51 => self.registers.d = self.registers.c,
            0x52 => self.registers.d = self.registers.d,
            0x53 => self.registers.d = self.registers.e,
            0x54 => self.registers.d = self.registers.h,
            0x55 => self.registers.d = self.registers.l,
            0x56 => self.registers.d = self.bus.read_u8(self.registers.hl()),
            0x57 => self.registers.d = self.registers.a,

            // LD E, x
            0x58 => self.registers.e = self.registers.b,
            0x59 => self.registers.e = self.registers.c,
            0x5a => self.registers.e = self.registers.d,
            0x5b => self.registers.e = self.registers.e,
            0x5c => self.registers.e = self.registers.h,
            0x5d => self.registers.e = self.registers.l,
            0x5e => self.registers.e = self.bus.read_u8(self.registers.hl()),
            0x5f => self.registers.e = self.registers.a,

            // LD H, x
            0x60 => self.registers.h = self.registers.b,
            0x61 => self.registers.h = self.registers.c,
            0x62 => self.registers.h = self.registers.d,
            0x63 => self.registers.h = self.registers.e,
            0x64 => self.registers.h = self.registers.h,
            0x65 => self.registers.h = self.registers.l,
            0x66 => self.registers.h = self.bus.read_u8(self.registers.hl()),
            0x67 => self.registers.h = self.registers.a,

            // LD L, x
            0x68 => self.registers.l = self.registers.b,
            0x69 => self.registers.l = self.registers.c,
            0x6a => self.registers.l = self.registers.d,
            0x6b => self.registers.l = self.registers.e,
            0x6c => self.registers.l = self.registers.h,
            0x6d => self.registers.l = self.registers.l,
            0x6e => self.registers.l = self.bus.read_u8(self.registers.hl()),
            0x6f => self.registers.l = self.registers.a,

            // LD (HL), x
            0x70 => self.bus.write_u8(self.registers.hl(), self.registers.b),
            0x71 => self.bus.write_u8(self.registers.hl(), self.registers.c),
            0x72 => self.bus.write_u8(self.registers.hl(), self.registers.d),
            0x73 => self.bus.write_u8(self.registers.hl(), self.registers.e),
            0x74 => self.bus.write_u8(self.registers.hl(), self.registers.h),
            0x75 => self.bus.write_u8(self.registers.hl(), self.registers.l),
            0x76 => self.halt(), // HALT
            0x77 => self.bus.write_u8(self.registers.hl(), self.registers.a),

            // LD A, x
            0x78 => self.registers.a = self.registers.b,
            0x79 => self.registers.a = self.registers.c,
            0x7a => self.registers.a = self.registers.d,
            0x7b => self.registers.a = self.registers.e,
            0x7c => self.registers.a = self.registers.h,
            0x7d => self.registers.a = self.registers.l,
            0x7e => self.registers.a = self.bus.read_u8(self.registers.hl()),
            0x7f => self.registers.a = self.registers.a,

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
            0xc1 => {
                let value = self.pop_r16();
                self.registers.set_bc(value)
            }
            0xc2 => todo!("unimplemented instruction"),
            0xc3 => todo!("unimplemented instruction"),
            0xc4 => todo!("unimplemented instruction"),
            0xc5 => self.push_r16(self.registers.bc()),
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
            0xd1 => {
                let value = self.pop_r16();
                self.registers.set_de(value)
            }
            0xd2 => todo!("unimplemented instruction"),
            0xd3 => panic!("instruction does not exist"),
            0xd4 => todo!("unimplemented instruction"),
            0xd5 => self.push_r16(self.registers.de()),
            0xd6 => todo!("unimplemented instruction"),
            0xd7 => todo!("unimplemented instruction"),
            0xd8 => todo!("unimplemented instruction"),
            0xd9 => todo!("unimplemented instruction"),
            0xda => todo!("unimplemented instruction"),
            0xdb => panic!("instruction does not exist"),
            0xdc => todo!("unimplemented instruction"),
            0xdd => panic!("instruction does not exist"),
            0xde => todo!("unimplemented instruction"),
            0xdf => todo!("unimplemented instruction"),

            0xe0 => todo!("unimplemented instruction"),
            0xe1 => {
                let value = self.pop_r16();
                self.registers.set_hl(value)
            }
            0xe2 => todo!("unimplemented instruction"),
            0xe3 => panic!("instruction does not exist"),
            0xe4 => panic!("instruction does not exist"),
            0xe5 => self.push_r16(self.registers.hl()),
            0xe6 => todo!("unimplemented instruction"),
            0xe7 => todo!("unimplemented instruction"),
            0xe8 => todo!("unimplemented instruction"),
            0xe9 => todo!("unimplemented instruction"),
            0xea => todo!("unimplemented instruction"),
            0xeb => panic!("instruction does not exist"),
            0xec => panic!("instruction does not exist"),
            0xed => panic!("instruction does not exist"),
            0xee => todo!("unimplemented instruction"),
            0xef => todo!("unimplemented instruction"),

            0xf0 => todo!("unimplemented instruction"),
            0xf1 => {
                let value = self.pop_r16();
                self.registers.set_af(value)
            }
            0xf2 => todo!("unimplemented instruction"),
            0xf3 => todo!("unimplemented instruction"),
            0xf4 => panic!("instruction does not exist"),
            0xf5 => self.push_r16(self.registers.af()),
            0xf6 => todo!("unimplemented instruction"),
            0xf7 => todo!("unimplemented instruction"),
            0xf8 => todo!("unimplemented instruction"),
            0xf9 => todo!("unimplemented instruction"),
            0xfa => todo!("unimplemented instruction"),
            0xfb => todo!("unimplemented instruction"),
            0xfc => panic!("instruction does not exist"),
            0xfd => panic!("instruction does not exist"),
            0xfe => todo!("unimplemented instruction"),
            0xff => todo!("unimplemented instruction"),
        }

        todo!()
    }

    // Bit shift instructions

    // Rotate bits in register left through carry.
    fn rl(&mut self, value: u8) -> u8 {
        // The most significant bit is set to carry
        // and the carry is shifted in into the least significant bit
        let carry_mask = self.registers.f.carry() | 0b1111_1110;

        let new_value = value.rotate_left(1) & carry_mask;
        self.registers.f.setc(new_value & 0x1 == 1, Flags::CARRY);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
    }

    // Rorate register a left through carry.
    fn rla(&mut self) {
        self.registers.a = self.rl(self.registers.a);
        self.registers.f.set(Flags::ZERO); // This instruction always puts it to zero.
    }

    // Rotate register left.
    fn rlc(&mut self, value: u8) -> u8 {
        // Rotates C <- [7 <- 0] <- 7
        let new_value = value.rotate_left(1);
        self.registers.f.setc(new_value & 0x1 == 1, Flags::CARRY);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
    }

    // Rotate register a left.
    fn rlca(&mut self) {
        self.registers.a = self.rlc(self.registers.a);
        self.registers.f.set(Flags::ZERO); // This instruction always puts it to zero.
    }

    // Rotate register right through carry.
    fn rr(&mut self, value: u8) -> u8 {
        // C -> [7 -> 0] -> C
        let carry_mask = self.registers.f.carry() << 7 | 0b0111_1111;

        let new_value = value.rotate_right(1) & carry_mask;
        self.registers.f.setc(value & 0x1 == 1, Flags::CARRY);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
    }

    // Rotate register a right through carry.
    fn rra(&mut self) {
        self.registers.a = self.rr(self.registers.a);
        self.registers.f.set(Flags::ZERO); // This instruction always puts it to zero.
    }

    // Rotate register right.
    fn rrc(&mut self, value: u8) -> u8 {
        // [0] -> [7 -> 0] -> C
        let new_value = value.rotate_right(1);
        self.registers.f.setc(value & 0x1 == 1, Flags::CARRY);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
    }

    // Rotate register a right.
    fn rrca(&mut self) {
        self.registers.a = self.rrc(self.registers.a);
        self.registers.f.set(Flags::ZERO); // This instruction always puts it to zero.
    }

    // Shift left arithmetic for register
    fn sla(&mut self, value: u8) -> u8 {
        let carry = value >> 7 & 0x1;
        self.registers.f.setc(carry == 1, Flags::CARRY);
        let new_value = value << 1;

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
    }

    // Shift right arithmetic
    fn sra(&mut self, value: u8) -> u8 {
        self.registers.f.setc(value & 0x1 == 1, Flags::CARRY);
        let mask = value | 0b0111_1111;
        let new_value = value >> 1 & mask;

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
    }

    // Shift right logical. Zero is put as the most significant bit.
    fn srl(&mut self, value: u8) -> u8 {
        self.registers.f.setc(value & 0x1 == 1, Flags::CARRY);
        let new_value = value >> 1;

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.clear(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::SUBTRACT);

        new_value
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
            // RLC r8
            0x00 => self.registers.b = self.rlc(self.registers.b),
            0x01 => self.registers.c = self.rlc(self.registers.c),
            0x02 => self.registers.d = self.rlc(self.registers.d),
            0x03 => self.registers.e = self.rlc(self.registers.e),
            0x04 => self.registers.h = self.rlc(self.registers.h),
            0x05 => self.registers.l = self.rlc(self.registers.l),
            0x06 => {
                let addr = self.registers.hl();
                let value = self.rlc(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RLC (HL)
            0x07 => self.registers.a = self.rlc(self.registers.d),

            // RRC r8
            0x08 => self.registers.b = self.rrc(self.registers.b),
            0x09 => self.registers.c = self.rrc(self.registers.c),
            0x0a => self.registers.d = self.rrc(self.registers.d),
            0x0b => self.registers.e = self.rrc(self.registers.e),
            0x0c => self.registers.h = self.rrc(self.registers.h),
            0x0d => self.registers.l = self.rrc(self.registers.l),
            0x0e => {
                let addr = self.registers.hl();
                let value = self.rrc(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RRC (HL)
            0x0f => self.registers.a = self.rrc(self.registers.a),

            // RL r8
            0x10 => self.registers.b = self.rl(self.registers.b),
            0x11 => self.registers.c = self.rl(self.registers.c),
            0x12 => self.registers.d = self.rl(self.registers.d),
            0x13 => self.registers.e = self.rl(self.registers.e),
            0x14 => self.registers.h = self.rl(self.registers.h),
            0x15 => self.registers.l = self.rl(self.registers.l),
            0x16 => {
                let addr = self.registers.hl();
                let value = self.rl(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RL (HL)
            0x17 => self.registers.a = self.rl(self.registers.a),

            // RR r8
            0x18 => self.registers.b = self.rr(self.registers.b),
            0x19 => self.registers.c = self.rr(self.registers.c),
            0x1a => self.registers.d = self.rr(self.registers.d),
            0x1b => self.registers.e = self.rr(self.registers.e),
            0x1c => self.registers.h = self.rr(self.registers.h),
            0x1d => self.registers.l = self.rr(self.registers.l),
            0x1e => {
                let addr = self.registers.hl();
                let value = self.rr(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RR (HL)
            0x1f => self.registers.a = self.rr(self.registers.a),

            // SLA r8
            0x20 => self.registers.b = self.sla(self.registers.b),
            0x21 => self.registers.c = self.sla(self.registers.c),
            0x22 => self.registers.d = self.sla(self.registers.d),
            0x23 => self.registers.e = self.sla(self.registers.e),
            0x24 => self.registers.h = self.sla(self.registers.h),
            0x25 => self.registers.l = self.sla(self.registers.l),
            0x26 => {
                let addr = self.registers.hl();
                let value = self.sla(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SLA (HL)
            0x27 => self.registers.a = self.sla(self.registers.a),

            // SRA r8
            0x28 => self.registers.b = self.sra(self.registers.b),
            0x29 => self.registers.c = self.sra(self.registers.c),
            0x2a => self.registers.d = self.sra(self.registers.d),
            0x2b => self.registers.e = self.sra(self.registers.e),
            0x2c => self.registers.h = self.sra(self.registers.h),
            0x2d => self.registers.l = self.sra(self.registers.l),
            0x2e => {
                let addr = self.registers.hl();
                let value = self.sra(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SRA (HL)
            0x2f => self.registers.a = self.sra(self.registers.a),

            // SWAP r8
            0x30 => self.registers.b = self.swap(self.registers.b),
            0x31 => self.registers.c = self.swap(self.registers.c),
            0x32 => self.registers.d = self.swap(self.registers.d),
            0x33 => self.registers.e = self.swap(self.registers.e),
            0x34 => self.registers.h = self.swap(self.registers.h),
            0x35 => self.registers.l = self.swap(self.registers.l),
            0x36 => {
                let addr = self.registers.hl();
                let value = self.swap(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SWAP (HL)
            0x37 => self.registers.a = self.swap(self.registers.a),

            // SRL r8
            0x38 => self.registers.b = self.srl(self.registers.b),
            0x39 => self.registers.c = self.srl(self.registers.c),
            0x3a => self.registers.d = self.srl(self.registers.d),
            0x3b => self.registers.e = self.srl(self.registers.e),
            0x3c => self.registers.h = self.srl(self.registers.h),
            0x3d => self.registers.l = self.srl(self.registers.l),
            0x3e => {
                let addr = self.registers.hl();
                let value = self.srl(self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SRL (HL)
            0x3f => self.registers.a = self.srl(self.registers.a),

            // BIT bit, r8
            0x40 => self.bit(0, self.registers.b),
            0x41 => self.bit(0, self.registers.c),
            0x42 => self.bit(0, self.registers.d),
            0x43 => self.bit(0, self.registers.e),
            0x44 => self.bit(0, self.registers.h),
            0x45 => self.bit(0, self.registers.l),
            0x46 => self.bit(0, self.bus.read_u8(self.registers.hl())),
            0x47 => self.bit(0, self.registers.a),

            0x48 => self.bit(1, self.registers.b),
            0x49 => self.bit(1, self.registers.c),
            0x4a => self.bit(1, self.registers.d),
            0x4b => self.bit(1, self.registers.e),
            0x4c => self.bit(1, self.registers.h),
            0x4d => self.bit(1, self.registers.l),
            0x4e => self.bit(1, self.bus.read_u8(self.registers.hl())),
            0x4f => self.bit(1, self.registers.a),

            0x50 => self.bit(2, self.registers.b),
            0x51 => self.bit(2, self.registers.c),
            0x52 => self.bit(2, self.registers.d),
            0x53 => self.bit(2, self.registers.e),
            0x54 => self.bit(2, self.registers.h),
            0x55 => self.bit(2, self.registers.l),
            0x56 => self.bit(2, self.bus.read_u8(self.registers.hl())),
            0x57 => self.bit(2, self.registers.a),

            0x58 => self.bit(3, self.registers.b),
            0x59 => self.bit(3, self.registers.c),
            0x5a => self.bit(3, self.registers.d),
            0x5b => self.bit(3, self.registers.e),
            0x5c => self.bit(3, self.registers.h),
            0x5d => self.bit(3, self.registers.l),
            0x5e => self.bit(3, self.bus.read_u8(self.registers.hl())),
            0x5f => self.bit(3, self.registers.a),

            0x60 => self.bit(4, self.registers.b),
            0x61 => self.bit(4, self.registers.c),
            0x62 => self.bit(4, self.registers.d),
            0x63 => self.bit(4, self.registers.e),
            0x64 => self.bit(4, self.registers.h),
            0x65 => self.bit(4, self.registers.l),
            0x66 => self.bit(4, self.bus.read_u8(self.registers.hl())),
            0x67 => self.bit(4, self.registers.a),

            0x68 => self.bit(5, self.registers.b),
            0x69 => self.bit(5, self.registers.c),
            0x6a => self.bit(5, self.registers.d),
            0x6b => self.bit(5, self.registers.e),
            0x6c => self.bit(5, self.registers.h),
            0x6d => self.bit(5, self.registers.l),
            0x6e => self.bit(5, self.bus.read_u8(self.registers.hl())),
            0x6f => self.bit(5, self.registers.a),

            0x70 => self.bit(6, self.registers.b),
            0x71 => self.bit(6, self.registers.c),
            0x72 => self.bit(6, self.registers.d),
            0x73 => self.bit(6, self.registers.e),
            0x74 => self.bit(6, self.registers.h),
            0x75 => self.bit(6, self.registers.l),
            0x76 => self.bit(6, self.bus.read_u8(self.registers.hl())),
            0x77 => self.bit(6, self.registers.a),

            0x78 => self.bit(7, self.registers.b),
            0x79 => self.bit(7, self.registers.c),
            0x7a => self.bit(7, self.registers.d),
            0x7b => self.bit(7, self.registers.e),
            0x7c => self.bit(7, self.registers.h),
            0x7d => self.bit(7, self.registers.l),
            0x7e => self.bit(7, self.bus.read_u8(self.registers.hl())),
            0x7f => self.bit(7, self.registers.a),

            // RES bit, r8
            0x80 => self.registers.b = self.res(0, self.registers.b),
            0x81 => self.registers.c = self.res(0, self.registers.c),
            0x82 => self.registers.d = self.res(0, self.registers.d),
            0x83 => self.registers.e = self.res(0, self.registers.e),
            0x84 => self.registers.h = self.res(0, self.registers.h),
            0x85 => self.registers.l = self.res(0, self.registers.l),
            0x86 => {
                let addr = self.registers.hl();
                let value = self.res(0, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0x87 => self.registers.a = self.res(0, self.registers.a),

            0x88 => self.registers.b = self.res(1, self.registers.b),
            0x89 => self.registers.c = self.res(1, self.registers.c),
            0x8a => self.registers.d = self.res(1, self.registers.d),
            0x8b => self.registers.e = self.res(1, self.registers.e),
            0x8c => self.registers.h = self.res(1, self.registers.h),
            0x8d => self.registers.l = self.res(1, self.registers.l),
            0x8e => {
                let addr = self.registers.hl();
                let value = self.res(1, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0x8f => self.registers.a = self.res(1, self.registers.a),

            0x90 => self.registers.b = self.res(2, self.registers.b),
            0x91 => self.registers.c = self.res(2, self.registers.c),
            0x92 => self.registers.d = self.res(2, self.registers.d),
            0x93 => self.registers.e = self.res(2, self.registers.e),
            0x94 => self.registers.h = self.res(2, self.registers.h),
            0x95 => self.registers.l = self.res(2, self.registers.l),
            0x96 => {
                let addr = self.registers.hl();
                let value = self.res(2, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0x97 => self.registers.a = self.res(2, self.registers.a),

            0x98 => self.registers.b = self.res(3, self.registers.b),
            0x99 => self.registers.c = self.res(3, self.registers.c),
            0x9a => self.registers.d = self.res(3, self.registers.d),
            0x9b => self.registers.e = self.res(3, self.registers.e),
            0x9c => self.registers.h = self.res(3, self.registers.h),
            0x9d => self.registers.l = self.res(3, self.registers.l),
            0x9e => {
                let addr = self.registers.hl();
                let value = self.res(3, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0x9f => self.registers.a = self.res(3, self.registers.a),

            0xa0 => self.registers.b = self.res(4, self.registers.b),
            0xa1 => self.registers.c = self.res(4, self.registers.c),
            0xa2 => self.registers.d = self.res(4, self.registers.d),
            0xa3 => self.registers.e = self.res(4, self.registers.e),
            0xa4 => self.registers.h = self.res(4, self.registers.h),
            0xa5 => self.registers.l = self.res(4, self.registers.l),
            0xa6 => {
                let addr = self.registers.hl();
                let value = self.res(4, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0xa7 => self.registers.a = self.res(4, self.registers.a),

            0xa8 => self.registers.b = self.res(5, self.registers.b),
            0xa9 => self.registers.c = self.res(5, self.registers.c),
            0xaa => self.registers.d = self.res(5, self.registers.d),
            0xab => self.registers.e = self.res(5, self.registers.e),
            0xac => self.registers.h = self.res(5, self.registers.h),
            0xad => self.registers.l = self.res(5, self.registers.l),
            0xae => {
                let addr = self.registers.hl();
                let value = self.res(5, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0xaf => self.registers.a = self.res(5, self.registers.a),

            0xb0 => self.registers.b = self.res(6, self.registers.b),
            0xb1 => self.registers.c = self.res(6, self.registers.c),
            0xb2 => self.registers.d = self.res(6, self.registers.d),
            0xb3 => self.registers.e = self.res(6, self.registers.e),
            0xb4 => self.registers.h = self.res(6, self.registers.h),
            0xb5 => self.registers.l = self.res(6, self.registers.l),
            0xb6 => {
                let addr = self.registers.hl();
                let value = self.res(6, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0xb7 => self.registers.a = self.res(6, self.registers.a),

            0xb8 => self.registers.b = self.res(7, self.registers.b),
            0xb9 => self.registers.c = self.res(7, self.registers.c),
            0xba => self.registers.d = self.res(7, self.registers.d),
            0xbb => self.registers.e = self.res(7, self.registers.e),
            0xbc => self.registers.h = self.res(7, self.registers.h),
            0xbd => self.registers.l = self.res(7, self.registers.l),
            0xbe => {
                let addr = self.registers.hl();
                let value = self.res(7, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // RES x, (HL)
            0xbf => self.registers.a = self.res(7, self.registers.a),

            // SET bit, r8
            0xc0 => self.registers.b = self.set(0, self.registers.b),
            0xc1 => self.registers.c = self.set(0, self.registers.c),
            0xc2 => self.registers.d = self.set(0, self.registers.d),
            0xc3 => self.registers.e = self.set(0, self.registers.e),
            0xc4 => self.registers.h = self.set(0, self.registers.h),
            0xc5 => self.registers.l = self.set(0, self.registers.l),
            0xc6 => {
                let addr = self.registers.hl();
                let value = self.set(0, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xc7 => self.registers.a = self.set(0, self.registers.a),

            0xc8 => self.registers.b = self.set(1, self.registers.b),
            0xc9 => self.registers.c = self.set(1, self.registers.c),
            0xca => self.registers.d = self.set(1, self.registers.d),
            0xcb => self.registers.e = self.set(1, self.registers.e),
            0xcc => self.registers.h = self.set(1, self.registers.h),
            0xcd => self.registers.l = self.set(1, self.registers.l),
            0xce => {
                let addr = self.registers.hl();
                let value = self.set(1, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xcf => self.registers.a = self.set(1, self.registers.a),

            0xd0 => self.registers.b = self.set(2, self.registers.b),
            0xd1 => self.registers.c = self.set(2, self.registers.c),
            0xd2 => self.registers.d = self.set(2, self.registers.d),
            0xd3 => self.registers.e = self.set(2, self.registers.e),
            0xd4 => self.registers.h = self.set(2, self.registers.h),
            0xd5 => self.registers.l = self.set(2, self.registers.l),
            0xd6 => {
                let addr = self.registers.hl();
                let value = self.set(2, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xd7 => self.registers.a = self.set(2, self.registers.a),

            0xd8 => self.registers.b = self.set(3, self.registers.b),
            0xd9 => self.registers.c = self.set(3, self.registers.c),
            0xda => self.registers.d = self.set(3, self.registers.d),
            0xdb => self.registers.e = self.set(3, self.registers.e),
            0xdc => self.registers.h = self.set(3, self.registers.h),
            0xdd => self.registers.l = self.set(3, self.registers.l),
            0xde => {
                let addr = self.registers.hl();
                let value = self.set(3, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xdf => self.registers.a = self.set(3, self.registers.a),

            0xe0 => self.registers.b = self.set(4, self.registers.b),
            0xe1 => self.registers.c = self.set(4, self.registers.c),
            0xe2 => self.registers.d = self.set(4, self.registers.d),
            0xe3 => self.registers.e = self.set(4, self.registers.e),
            0xe4 => self.registers.h = self.set(4, self.registers.h),
            0xe5 => self.registers.l = self.set(4, self.registers.l),
            0xe6 => {
                let addr = self.registers.hl();
                let value = self.set(4, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xe7 => self.registers.a = self.set(4, self.registers.a),

            0xe8 => self.registers.b = self.set(5, self.registers.b),
            0xe9 => self.registers.c = self.set(5, self.registers.c),
            0xea => self.registers.d = self.set(5, self.registers.d),
            0xeb => self.registers.e = self.set(5, self.registers.e),
            0xec => self.registers.h = self.set(5, self.registers.h),
            0xed => self.registers.l = self.set(5, self.registers.l),
            0xee => {
                let addr = self.registers.hl();
                let value = self.set(5, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xef => self.registers.a = self.set(5, self.registers.a),

            0xf0 => self.registers.b = self.set(6, self.registers.b),
            0xf1 => self.registers.c = self.set(6, self.registers.c),
            0xf2 => self.registers.d = self.set(6, self.registers.d),
            0xf3 => self.registers.e = self.set(6, self.registers.e),
            0xf4 => self.registers.h = self.set(6, self.registers.h),
            0xf5 => self.registers.l = self.set(6, self.registers.l),
            0xf6 => {
                let addr = self.registers.hl();
                let value = self.set(6, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xf7 => self.registers.a = self.set(6, self.registers.a),

            0xf8 => self.registers.b = self.set(7, self.registers.b),
            0xf9 => self.registers.c = self.set(7, self.registers.c),
            0xfa => self.registers.d = self.set(7, self.registers.d),
            0xfb => self.registers.e = self.set(7, self.registers.e),
            0xfc => self.registers.h = self.set(7, self.registers.h),
            0xfd => self.registers.l = self.set(7, self.registers.l),
            0xfe => {
                let addr = self.registers.hl();
                let value = self.set(7, self.bus.read_u8(addr));
                self.bus.write_u8(addr, value);
            } // SET x, (HL)
            0xff => self.registers.a = self.set(7, self.registers.a),
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
