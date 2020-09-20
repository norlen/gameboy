use super::memory::MemoryBus;
use super::registers::{Flags, Registers};

const PREFIXED_INSTRUCTION: u8 = 0xcb;

#[derive(Default)]
pub struct CPU {
    registers: Registers,
    pc: u16,
    bus: MemoryBus,
    cycles: usize,
}

enum Register16 {
    BC,
    DE,
    HL,
}

macro_rules! op {
    ($cycles:expr, $bytes:expr, $op:expr) => {{
        $op;
        ($cycles, $bytes)
    }};
    ($cycles:expr, $bytes:expr, $op:expr => $set:expr) => {{
        $set = $op;
        ($cycles, $bytes)
    }};
    ($cycles:expr, $bytes:expr, $op:expr; $set_fn:expr) => {{
        let value = $op;
        $set_fn(value);
        ($cycles, $bytes)
    }};
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
    fn and_r8(&mut self, value: u8) {
        self.registers.a &= value;
        self.registers.f.setc(self.registers.a == 0, Flags::ZERO);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.set(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::CARRY);
    }

    fn and_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.and_r8(value);
    }

    fn or_r8(&mut self, value: u8) {
        self.registers.a |= value;
        self.registers.f.setc(self.registers.a == 0, Flags::ZERO);
        self.registers.f.clear(Flags::SUBTRACT);
        self.registers.f.set(Flags::HALF_CARRY);
        self.registers.f.clear(Flags::CARRY);
    }

    fn or_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.or_r8(value);
    }

    fn xor_r8(&mut self, value: u8) {
        self.registers.a ^= value;
        self.registers.f.0 = 0u8;
        self.registers.f.setc(self.registers.a == 0, Flags::ZERO);
    }

    fn xor_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.xor_r8(value);
    }

    fn add_r8(&mut self, value: u8) {
        self.registers.a = self.add_with_carry(value, 0);
    }

    fn add_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.registers.a = self.add_with_carry(value, 0);
    }

    fn adc_r8(&mut self, value: u8) {
        self.registers.a = self.add_with_carry(value, self.registers.f.carry());
    }

    fn adc_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.registers.a = self.add_with_carry(value, self.registers.f.carry());
    }

    fn sub_r8(&mut self, value: u8) {
        self.registers.a = self.subtract(value);
    }

    fn sub_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.registers.a = self.subtract(value);
    }

    fn sbc_r8(&mut self, value: u8) {
        let carry = self.registers.f.carry();
        self.registers.a = self.subtract_with_carry(value, carry);
    }

    fn sbc_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        let carry = self.registers.f.carry();
        self.registers.a = self.subtract_with_carry(value, carry);
    }

    fn subtract(&mut self, value: u8) -> u8 {
        let (new_value, overflow) = self.registers.a.overflowing_sub(value);

        let half_carry = hc_borrow(self.registers.a, value);
        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        self.registers.f.setc(overflow, Flags::CARRY);
        new_value
    }

    fn subtract_with_carry(&mut self, value: u8, carry: u8) -> u8 {
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
    fn cp_r8(&mut self, value: u8) {
        let _ = self.subtract(value);
    }

    /// Subtract `a` with `value` and set flags, but don't store the results.
    fn cp_a8(&mut self, addr: u16) {
        let value = self.bus.read_u8(addr);
        let _ = self.subtract(value);
    }

    fn dec_u8(&mut self, value: u8) -> u8 {
        let new_value = value.wrapping_sub(1);

        self.registers.f.setc(new_value == 0, Flags::ZERO);
        self.registers.f.set(Flags::SUBTRACT);
        let half_carry = (value & 0xf) < (new_value & 0xf);
        self.registers.f.setc(half_carry, Flags::HALF_CARRY);
        new_value
    }

    fn dec_r16(&mut self, value: u16) -> u16 {
        value.wrapping_sub(1)
        // No flags affected
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

    fn inc_r16(&mut self, value: u16) -> u16 {
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

    // LD (r16), r8
    fn ld_a8_r8(&mut self, addr: u16, value: u8) {
        self.bus.write_u8(addr, value)
    }

    fn ld_a8_r8_hlplus(&mut self, value: u8) {
        let addr = self.registers.hl();
        self.ld_a8_r8(addr, value);
        self.registers.set_hl(addr + 1);
    }

    fn ld_a8_r8_hlminus(&mut self, value: u8) {
        let addr = self.registers.hl();
        self.ld_a8_r8(addr, value);
        self.registers.set_hl(addr - 1);
    }

    // LD r8, d8
    fn ld_r8_d8(&self) -> u8 {
        self.bus.read_u8(self.pc + 1)
    }
    
    fn ld_a8_d8(&mut self, addr: u16) {
        let value = self.bus.read_u8(self.pc + 1);
        self.bus.write_u8(addr, value);
    }

    // LD r8, (r16)
    fn ld_r8_a8(&self, addr: u16) -> u8 {
        self.bus.read_u8(addr)
    }
    
    fn ld_r8_a8_hlplus(&mut self) -> u8 {
        let addr = self.registers.hl();
        self.registers.set_hl(addr + 1);
        self.ld_r8_a8(addr)
    }
    
    fn ld_r8_a8_hlminus(&mut self) -> u8 {
        let addr = self.registers.hl();
        self.registers.set_hl(addr - 1);
        self.ld_r8_a8(addr)
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

    fn ld_r8(&mut self, value: u8) -> u8 {
        value
    }

    fn ld_a8(&mut self, addr: u16) -> u8 {
        self.bus.read_u8(addr)
    }

    fn execute(&mut self, instruction: u8) -> u16 {
        let (_bytes, _cycles) = match instruction {
            // 0x0X
            0x00 => op!(1, 4, self.nop()),
            0x01 => op!(3, 12, self.ld_r16_d16(); |v| self.registers.set_bc(v)),
            0x02 => op!(1, 8, self.ld_a8_r8(self.registers.bc(), self.registers.a)),
            0x03 => op!(1, 8, self.inc_r16(self.registers.bc()); |v| self.registers.set_bc(v)),
            0x04 => op!(1, 4, self.inc_u8(self.registers.b) => self.registers.b),
            0x05 => op!(1, 4, self.dec_u8(self.registers.b) => self.registers.b),
            0x06 => op!(2, 8, self.ld_r8_d8() => self.registers.b),
            0x07 => op!(1, 4, self.rlca()),
            0x08 => op!(3, 20, self.ld_a16_sp()),
            0x09 => op!(1, 8, self.add_hl_r16(self.registers.bc())), // ADD HL, BC
            0x0a => op!(1, 8, self.ld_r8_a8(self.registers.bc()) => self.registers.a),
            0x0b => op!(1, 8, self.dec_r16(self.registers.bc()); |v| self.registers.set_bc(v)),
            0x0c => op!(1, 4, self.inc_u8(self.registers.c) => self.registers.c),
            0x0d => op!(1, 4, self.dec_u8(self.registers.c) => self.registers.c),
            0x0e => op!(2, 8, self.ld_r8_d8() => self.registers.c),
            0x0f => op!(1, 4, self.rrca()),

            // 0x1X
            0x10 => op!(1, 4, self.stop()),
            0x11 => op!(3, 12, self.ld_r16_d16(); |v| self.registers.set_de(v)),
            0x12 => op!(1, 8, self.ld_a8_r8(self.registers.de(), self.registers.a)),
            0x13 => op!(1, 8, self.inc_r16(self.registers.de()); |v| self.registers.set_de(v)),
            0x14 => op!(1, 4, self.inc_u8(self.registers.d) => self.registers.d),
            0x15 => op!(1, 4, self.dec_u8(self.registers.d) => self.registers.d),
            0x16 => op!(2, 8, self.ld_r8_d8() => self.registers.d),
            0x17 => op!(1, 4, self.rla()),
            0x18 => op!(2, 12, || todo!("unimplemented instruction")), // JR r8
            0x19 => op!(1, 8, self.add_hl_r16(self.registers.de())), // ADD HL, DE
            0x1a => op!(1, 8, self.ld_r8_a8(self.registers.de()) => self.registers.a),
            0x1b => op!(1, 8, self.dec_r16(self.registers.de()); |v| self.registers.set_de(v)),
            0x1c => op!(1, 4, self.inc_u8(self.registers.e) => self.registers.e), // INC E
            0x1d => op!(1, 4, self.dec_u8(self.registers.e) => self.registers.e), // DEC E
            0x1e => op!(2, 8, self.ld_r8_d8() => self.registers.e),
            0x1f => op!(1, 4, self.rra()),

            // 0x2X
            0x20 => todo!("unimplemented instruction"), // JR NZ, r8
            0x21 => op!(3, 12, self.ld_r16_d16(); |v| self.registers.set_hl(v)),
            0x22 => op!(1, 8, self.ld_a8_r8_hlplus(self.registers.a)),
            0x23 => op!(1, 8, self.inc_r16(self.registers.hl()); |v| self.registers.set_hl(v)),
            0x24 => op!(1, 4, self.inc_u8(self.registers.h) => self.registers.h),
            0x25 => op!(1, 4, self.dec_u8(self.registers.h) => self.registers.h),
            0x26 => op!(2, 8, self.ld_r8_d8() => self.registers.h),
            0x27 => op!(1, 4, self.daa()),
            0x28 => todo!("unimplemented instruction"), // JR Z, r8
            0x29 => op!(1, 8, self.add_hl_r16(self.registers.hl())), // ADD HL, HL
            0x2a => op!(1, 8, self.ld_r8_a8_hlplus() => self.registers.a),
            0x2b => op!(1, 8, self.dec_r16(self.registers.hl()); |v| self.registers.set_hl(v)),
            0x2c => op!(1, 4, self.inc_u8(self.registers.l) => self.registers.l), // INC L
            0x2d => op!(1, 4, self.dec_u8(self.registers.l) => self.registers.l), // DEC L
            0x2e => op!(2, 8, self.ld_r8_d8() => self.registers.l),
            0x2f => op!(1, 4, self.cpl()),

            // 0x3X
            0x30 => todo!("unimplemented instruction"), // JR NC, r8
            0x31 => op!(3, 12, self.ld_r16_d16() => self.registers.sp),
            0x32 => op!(1, 8, self.ld_a8_r8_hlminus(self.registers.a)),
            0x33 => op!(1, 8, self.inc_r16(self.registers.sp) => self.registers.sp),
            0x34 => op!(1, 12, self.inc_a8(self.registers.hl())), // INC (HL)
            0x35 => op!(1, 12, self.dec_a8(self.registers.hl())), // DEC (HL)
            0x36 => op!(2, 12, self.ld_a8_d8(self.registers.hl())),
            0x37 => op!(1, 4, self.scf()),
            0x38 => todo!("unimplemented instruction"), // JR C, r8
            0x39 => op!(1, 8, self.add_hl_r16(self.registers.sp)), // ADD HL, SP
            0x3a => op!(1, 8, self.ld_r8_a8_hlminus() => self.registers.a),
            0x3b => op!(1, 8, self.dec_r16(self.registers.sp) => self.registers.sp),
            0x3c => op!(1, 4,self.inc_u8(self.registers.a) => self.registers.a), // INC A
            0x3d => op!(1, 4,self.dec_u8(self.registers.a) => self.registers.a), // DEC A
            0x3e => op!(2, 8,self.ld_r8_d8() => self.registers.a),
            0x3f => op!(1, 4, self.ccf()),

            // LD B, x
            0x40 => op!(1, 4, self.ld_r8(self.registers.b) => self.registers.b),
            0x41 => op!(1, 4, self.ld_r8(self.registers.c) => self.registers.b),
            0x42 => op!(1, 4, self.ld_r8(self.registers.d) => self.registers.b),
            0x43 => op!(1, 4, self.ld_r8(self.registers.e) => self.registers.b),
            0x44 => op!(1, 4, self.ld_r8(self.registers.h) => self.registers.b),
            0x45 => op!(1, 4, self.ld_r8(self.registers.l) => self.registers.b),
            0x46 => op!(1, 8, self.ld_a8(self.registers.hl()) => self.registers.b),
            0x47 => op!(1, 4, self.ld_r8(self.registers.a) => self.registers.b),

            // LD C, x
            0x48 => op!(1, 4, self.ld_r8(self.registers.b) => self.registers.c),
            0x49 => op!(1, 4, self.ld_r8(self.registers.c) => self.registers.c),
            0x4a => op!(1, 4, self.ld_r8(self.registers.d) => self.registers.c),
            0x4b => op!(1, 4, self.ld_r8(self.registers.e) => self.registers.c),
            0x4c => op!(1, 4, self.ld_r8(self.registers.h) => self.registers.c),
            0x4d => op!(1, 4, self.ld_r8(self.registers.l) => self.registers.c),
            0x4e => op!(1, 8, self.ld_a8(self.registers.hl()) => self.registers.c),
            0x4f => op!(1, 4, self.ld_r8(self.registers.a) => self.registers.c),

            // LD D, x
            0x50 => op!(1, 4, self.ld_r8(self.registers.b) => self.registers.d),
            0x51 => op!(1, 4, self.ld_r8(self.registers.c) => self.registers.d),
            0x52 => op!(1, 4, self.ld_r8(self.registers.d) => self.registers.d),
            0x53 => op!(1, 4, self.ld_r8(self.registers.e) => self.registers.d),
            0x54 => op!(1, 4, self.ld_r8(self.registers.h) => self.registers.d),
            0x55 => op!(1, 4, self.ld_r8(self.registers.l) => self.registers.d),
            0x56 => op!(1, 8, self.ld_a8(self.registers.hl()) => self.registers.d),
            0x57 => op!(1, 4, self.ld_r8(self.registers.a) => self.registers.d),

            // LD E, x
            0x58 => op!(1, 4, self.ld_r8(self.registers.b) => self.registers.e),
            0x59 => op!(1, 4, self.ld_r8(self.registers.c) => self.registers.e),
            0x5a => op!(1, 4, self.ld_r8(self.registers.d) => self.registers.e),
            0x5b => op!(1, 4, self.ld_r8(self.registers.e) => self.registers.e),
            0x5c => op!(1, 4, self.ld_r8(self.registers.h) => self.registers.e),
            0x5d => op!(1, 4, self.ld_r8(self.registers.l) => self.registers.e),
            0x5e => op!(1, 8, self.ld_a8(self.registers.hl()) => self.registers.e),
            0x5f => op!(1, 4, self.ld_r8(self.registers.a) => self.registers.e),

            // LD H, x
            0x60 => op!(1, 4, self.ld_r8(self.registers.b) => self.registers.h),
            0x61 => op!(1, 4, self.ld_r8(self.registers.c) => self.registers.h),
            0x62 => op!(1, 4, self.ld_r8(self.registers.d) => self.registers.h),
            0x63 => op!(1, 4, self.ld_r8(self.registers.e) => self.registers.h),
            0x64 => op!(1, 4, self.ld_r8(self.registers.h) => self.registers.h),
            0x65 => op!(1, 4, self.ld_r8(self.registers.l) => self.registers.h),
            0x66 => op!(1, 8, self.ld_a8(self.registers.hl()) => self.registers.h),
            0x67 => op!(1, 4, self.ld_r8(self.registers.a) => self.registers.h),

            // LD L, x
            0x68 => op!(1, 4, self.ld_r8(self.registers.b) => self.registers.l),
            0x69 => op!(1, 4, self.ld_r8(self.registers.c) => self.registers.l),
            0x6a => op!(1, 4, self.ld_r8(self.registers.d) => self.registers.l),
            0x6b => op!(1, 4, self.ld_r8(self.registers.e) => self.registers.l),
            0x6c => op!(1, 4, self.ld_r8(self.registers.h) => self.registers.l),
            0x6d => op!(1, 4, self.ld_r8(self.registers.l) => self.registers.l),
            0x6e => op!(1, 8, self.ld_a8(self.registers.hl()) => self.registers.l),
            0x6f => op!(1, 4, self.ld_r8(self.registers.a) => self.registers.l),

            // LD (HL), x
            0x70 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.b)),
            0x71 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.c)),
            0x72 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.d)),
            0x73 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.e)),
            0x74 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.h)),
            0x75 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.l)),
            0x76 => op!(1, 4, self.halt()),
            0x77 => op!(1, 8, self.ld_a8_r8(self.registers.hl(), self.registers.a)),

            // LD A, x
            0x78 => op!(1, 4, self.ld_r8(self.registers.b)),
            0x79 => op!(1, 4, self.ld_r8(self.registers.c)),
            0x7a => op!(1, 4, self.ld_r8(self.registers.d)),
            0x7b => op!(1, 4, self.ld_r8(self.registers.e)),
            0x7c => op!(1, 4, self.ld_r8(self.registers.h)),
            0x7d => op!(1, 4, self.ld_r8(self.registers.l)),
            0x7e => op!(1, 8, self.ld_a8(self.registers.hl())),
            0x7f => op!(1, 4, self.ld_r8(self.registers.a)),

            0x80 => op!(1, 4, self.add_r8(self.registers.b)),
            0x81 => op!(1, 4, self.add_r8(self.registers.c)),
            0x82 => op!(1, 4, self.add_r8(self.registers.d)),
            0x83 => op!(1, 4, self.add_r8(self.registers.e)),
            0x84 => op!(1, 4, self.add_r8(self.registers.h)),
            0x85 => op!(1, 4, self.add_r8(self.registers.l)),
            0x86 => op!(1, 8, self.add_a8(self.registers.hl())),
            0x87 => op!(1, 4, self.add_r8(self.registers.a)),

            0x88 => op!(1, 4, self.adc_r8(self.registers.b)),
            0x89 => op!(1, 4, self.adc_r8(self.registers.c)),
            0x8a => op!(1, 4, self.adc_r8(self.registers.d)),
            0x8b => op!(1, 4, self.adc_r8(self.registers.e)),
            0x8c => op!(1, 4, self.adc_r8(self.registers.h)),
            0x8d => op!(1, 4, self.adc_r8(self.registers.l)),
            0x8e => op!(1, 8, self.adc_a8(self.registers.hl())),
            0x8f => op!(1, 4, self.adc_r8(self.registers.a)),

            0x90 => op!(1, 4, self.sub_r8(self.registers.b)),
            0x91 => op!(1, 4, self.sub_r8(self.registers.c)),
            0x92 => op!(1, 4, self.sub_r8(self.registers.d)),
            0x93 => op!(1, 4, self.sub_r8(self.registers.e)),
            0x94 => op!(1, 4, self.sub_r8(self.registers.h)),
            0x95 => op!(1, 4, self.sub_r8(self.registers.l)),
            0x96 => op!(1, 8, self.sub_a8(self.registers.hl())),
            0x97 => op!(1, 4, self.sub_r8(self.registers.a)),

            0x98 => op!(1, 4, self.sbc_r8(self.registers.b)),
            0x99 => op!(1, 4, self.sbc_r8(self.registers.c)),
            0x9a => op!(1, 4, self.sbc_r8(self.registers.d)),
            0x9b => op!(1, 4, self.sbc_r8(self.registers.e)),
            0x9c => op!(1, 4, self.sbc_r8(self.registers.h)),
            0x9d => op!(1, 4, self.sbc_r8(self.registers.l)),
            0x9e => op!(1, 8, self.sbc_a8(self.registers.hl())),
            0x9f => op!(1, 4, self.sbc_r8(self.registers.a)),

            0xa0 => op!(1, 4, self.and_r8(self.registers.b)),
            0xa1 => op!(1, 4, self.and_r8(self.registers.c)),
            0xa2 => op!(1, 4, self.and_r8(self.registers.d)),
            0xa3 => op!(1, 4, self.and_r8(self.registers.e)),
            0xa4 => op!(1, 4, self.and_r8(self.registers.h)),
            0xa5 => op!(1, 4, self.and_r8(self.registers.l)),
            0xa6 => op!(1, 8, self.and_a8(self.registers.hl())),
            0xa7 => op!(1, 4, self.and_r8(self.registers.a)),

            0xa8 => op!(1, 4, self.xor_r8(self.registers.b)),
            0xa9 => op!(1, 4, self.xor_r8(self.registers.c)),
            0xaa => op!(1, 4, self.xor_r8(self.registers.d)),
            0xab => op!(1, 4, self.xor_r8(self.registers.e)),
            0xac => op!(1, 4, self.xor_r8(self.registers.h)),
            0xad => op!(1, 4, self.xor_r8(self.registers.l)),
            0xae => op!(1, 8, self.xor_a8(self.registers.hl())),
            0xaf => op!(1, 4, self.xor_r8(self.registers.a)),

            0xb0 => op!(1, 4, self.or_r8(self.registers.b)),
            0xb1 => op!(1, 4, self.or_r8(self.registers.c)),
            0xb2 => op!(1, 4, self.or_r8(self.registers.d)),
            0xb3 => op!(1, 4, self.or_r8(self.registers.e)),
            0xb4 => op!(1, 4, self.or_r8(self.registers.h)),
            0xb5 => op!(1, 4, self.or_r8(self.registers.l)),
            0xb6 => op!(1, 8, self.or_a8(self.registers.hl())),
            0xb7 => op!(1, 4, self.or_r8(self.registers.a)),

            0xb8 => op!(1, 4, self.cp_r8(self.registers.b)),
            0xb9 => op!(1, 4, self.cp_r8(self.registers.c)),
            0xba => op!(1, 4, self.cp_r8(self.registers.d)),
            0xbb => op!(1, 4, self.cp_r8(self.registers.e)),
            0xbc => op!(1, 4, self.cp_r8(self.registers.h)),
            0xbd => op!(1, 4, self.cp_r8(self.registers.l)),
            0xbe => op!(1, 8, self.cp_a8(self.registers.hl())),
            0xbf => op!(1, 4, self.cp_r8(self.registers.a)),

            0xc0 => todo!("unimplemented instruction"),
            0xc1 => op!(1, 12, self.pop_r16(); |v| self.registers.set_bc(v)),
            0xc2 => todo!("unimplemented instruction"),
            0xc3 => todo!("unimplemented instruction"),
            0xc4 => todo!("unimplemented instruction"),
            0xc5 => op!(1, 16, self.push_r16(self.registers.bc())),
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
            0xd1 => op!(1, 12, self.pop_r16(); |v| self.registers.set_de(v)),
            0xd2 => todo!("unimplemented instruction"),
            0xd3 => panic!("instruction does not exist"),
            0xd4 => todo!("unimplemented instruction"),
            0xd5 => op!(1, 16, self.push_r16(self.registers.de())),
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
            0xe1 => op!(1, 12, self.pop_r16(); |v| self.registers.set_hl(v)),
            0xe2 => todo!("unimplemented instruction"),
            0xe3 => panic!("instruction does not exist"),
            0xe4 => panic!("instruction does not exist"),
            0xe5 => op!(1, 16, self.push_r16(self.registers.hl())),
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
            0xf1 => op!(1, 12, self.pop_r16(); |v| self.registers.set_af(v)),
            0xf2 => todo!("unimplemented instruction"),
            0xf3 => todo!("unimplemented instruction"),
            0xf4 => panic!("instruction does not exist"),
            0xf5 => op!(1, 16, self.push_r16(self.registers.af())),
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
        };

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

    fn bit_a8(&mut self, bit: u8, addr: u16) {
        let value = self.bus.read_u8(addr);
        self.bit(bit, value);
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

    fn rlc_a8(&mut self, addr: u16) {
        let value = self.rlc(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn rrc_a8(&mut self, addr: u16) {
        let value = self.rrc(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn rl_a8(&mut self, addr: u16) {
        let value = self.rl(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn rr_a8(&mut self, addr: u16) {
        let value = self.rr(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn sla_a8(&mut self, addr: u16) {
        let value = self.sla(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn sra_a8(&mut self, addr: u16) {
        let value = self.sra(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }
    
    fn srl_a8(&mut self, addr: u16) {
        let value = self.srl(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn swap_a8(&mut self, addr: u16) {
        let value = self.swap(self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn res_a8(&mut self, bit: u8, addr: u16) {
        let value = self.res(bit, self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn set_a8(&mut self, bit: u8, addr: u16) {
        let value = self.set(bit, self.bus.read_u8(addr));
        self.bus.write_u8(addr, value);
    }

    fn execute_prefixed(&mut self, instruction: u8) -> u16 {
        let (_cycles, _bytes) = match instruction {
            // RLC r8
            0x00 => op!(2, 8, self.rlc(self.registers.b) => self.registers.b),
            0x01 => op!(2, 8, self.rlc(self.registers.c) => self.registers.c),
            0x02 => op!(2, 8, self.rlc(self.registers.d) => self.registers.d),
            0x03 => op!(2, 8, self.rlc(self.registers.e) => self.registers.e),
            0x04 => op!(2, 8, self.rlc(self.registers.h) => self.registers.h),
            0x05 => op!(2, 8, self.rlc(self.registers.l) => self.registers.l),
            0x06 => op!(2, 16, self.rlc_a8(self.registers.hl())),
            0x07 => op!(2, 8, self.rlc(self.registers.a) => self.registers.a),

            // RRC r8
            0x08 => op!(2, 8, self.rrc(self.registers.b) => self.registers.b),
            0x09 => op!(2, 8, self.rrc(self.registers.c) => self.registers.c),
            0x0a => op!(2, 8, self.rrc(self.registers.d) => self.registers.d),
            0x0b => op!(2, 8, self.rrc(self.registers.e) => self.registers.e),
            0x0c => op!(2, 8, self.rrc(self.registers.h) => self.registers.h),
            0x0d => op!(2, 8, self.rrc(self.registers.l) => self.registers.l),
            0x0e => op!(2, 16, self.rrc_a8(self.registers.hl())),
            0x0f => op!(2, 8, self.rrc(self.registers.a) => self.registers.a),

            // RL r8
            0x10 => op!(2, 8, self.rl(self.registers.b) => self.registers.b),
            0x11 => op!(2, 8, self.rl(self.registers.c) => self.registers.c),
            0x12 => op!(2, 8, self.rl(self.registers.d) => self.registers.d),
            0x13 => op!(2, 8, self.rl(self.registers.e) => self.registers.e),
            0x14 => op!(2, 8, self.rl(self.registers.h) => self.registers.h),
            0x15 => op!(2, 8, self.rl(self.registers.l) => self.registers.l),
            0x16 => op!(2, 16, self.rl_a8(self.registers.hl())),
            0x17 => op!(2, 8, self.rl(self.registers.a) => self.registers.a),

            // RR r8
            0x18 => op!(2, 8, self.rr(self.registers.b) => self.registers.b),
            0x19 => op!(2, 8, self.rr(self.registers.c) => self.registers.c),
            0x1a => op!(2, 8, self.rr(self.registers.d) => self.registers.d),
            0x1b => op!(2, 8, self.rr(self.registers.e) => self.registers.e),
            0x1c => op!(2, 8, self.rr(self.registers.h) => self.registers.h),
            0x1d => op!(2, 8, self.rr(self.registers.l) => self.registers.l),
            0x1e => op!(2, 16, self.rr_a8(self.registers.hl())),
            0x1f => op!(2, 8, self.rr(self.registers.a) => self.registers.a),

            // SLA r8
            0x20 => op!(2, 8, self.sla(self.registers.b) => self.registers.b),
            0x21 => op!(2, 8, self.sla(self.registers.c) => self.registers.c),
            0x22 => op!(2, 8, self.sla(self.registers.d) => self.registers.d),
            0x23 => op!(2, 8, self.sla(self.registers.e) => self.registers.e),
            0x24 => op!(2, 8, self.sla(self.registers.h) => self.registers.h),
            0x25 => op!(2, 8, self.sla(self.registers.l) => self.registers.l),
            0x26 => op!(2, 16, self.sla_a8(self.registers.hl())),
            0x27 => op!(2, 8, self.sla(self.registers.a) => self.registers.a),

            // SRA r8
            0x28 => op!(2, 8, self.sra(self.registers.b) => self.registers.b),
            0x29 => op!(2, 8, self.sra(self.registers.c) => self.registers.c),
            0x2a => op!(2, 8, self.sra(self.registers.d) => self.registers.d),
            0x2b => op!(2, 8, self.sra(self.registers.e) => self.registers.e),
            0x2c => op!(2, 8, self.sra(self.registers.h) => self.registers.h),
            0x2d => op!(2, 8, self.sra(self.registers.l) => self.registers.l),
            0x2e => op!(2, 16, self.sra_a8(self.registers.hl())),
            0x2f => op!(2, 8, self.sra(self.registers.a) => self.registers.a),

            // SWAP r8
            0x30 => op!(2, 8, self.swap(self.registers.b) => self.registers.b),
            0x31 => op!(2, 8, self.swap(self.registers.c) => self.registers.c),
            0x32 => op!(2, 8, self.swap(self.registers.d) => self.registers.d),
            0x33 => op!(2, 8, self.swap(self.registers.e) => self.registers.e),
            0x34 => op!(2, 8, self.swap(self.registers.h) => self.registers.h),
            0x35 => op!(2, 8, self.swap(self.registers.l) => self.registers.l),
            0x36 => op!(2, 16, self.swap_a8(self.registers.hl())),
            0x37 => op!(2, 8, self.swap(self.registers.a) => self.registers.a),

            // SRL r8
            0x38 => op!(2, 8, self.srl(self.registers.b) => self.registers.b),
            0x39 => op!(2, 8, self.srl(self.registers.c) => self.registers.c),
            0x3a => op!(2, 8, self.srl(self.registers.d) => self.registers.d),
            0x3b => op!(2, 8, self.srl(self.registers.e) => self.registers.e),
            0x3c => op!(2, 8, self.srl(self.registers.h) => self.registers.h),
            0x3d => op!(2, 8, self.srl(self.registers.l) => self.registers.l),
            0x3e => op!(2, 16, self.srl_a8(self.registers.hl())),
            0x3f => op!(2, 8, self.srl(self.registers.a) => self.registers.a),

            // BIT bit, r8
            0x40 => op!(2, 8, self.bit(0, self.registers.b)),
            0x41 => op!(2, 8, self.bit(0, self.registers.c)),
            0x42 => op!(2, 8, self.bit(0, self.registers.d)),
            0x43 => op!(2, 8, self.bit(0, self.registers.e)),
            0x44 => op!(2, 8, self.bit(0, self.registers.h)),
            0x45 => op!(2, 8, self.bit(0, self.registers.l)),
            0x46 => op!(2, 12, self.bit_a8(0, self.registers.hl())),
            0x47 => op!(2, 8, self.bit(0, self.registers.a)),

            0x48 => op!(2, 8, self.bit(1, self.registers.b)),
            0x49 => op!(2, 8, self.bit(1, self.registers.c)),
            0x4a => op!(2, 8, self.bit(1, self.registers.d)),
            0x4b => op!(2, 8, self.bit(1, self.registers.e)),
            0x4c => op!(2, 8, self.bit(1, self.registers.h)),
            0x4d => op!(2, 8, self.bit(1, self.registers.l)),
            0x4e => op!(2, 12, self.bit_a8(1, self.registers.hl())),
            0x4f => op!(2, 8, self.bit(1, self.registers.a)),

            0x50 => op!(2, 8, self.bit(2, self.registers.b)),
            0x51 => op!(2, 8, self.bit(2, self.registers.c)),
            0x52 => op!(2, 8, self.bit(2, self.registers.d)),
            0x53 => op!(2, 8, self.bit(2, self.registers.e)),
            0x54 => op!(2, 8, self.bit(2, self.registers.h)),
            0x55 => op!(2, 8, self.bit(2, self.registers.l)),
            0x56 => op!(2, 12, self.bit_a8(2, self.registers.hl())),
            0x57 => op!(2, 8, self.bit(2, self.registers.a)),

            0x58 => op!(2, 8, self.bit(3, self.registers.b)),
            0x59 => op!(2, 8, self.bit(3, self.registers.c)),
            0x5a => op!(2, 8, self.bit(3, self.registers.d)),
            0x5b => op!(2, 8, self.bit(3, self.registers.e)),
            0x5c => op!(2, 8, self.bit(3, self.registers.h)),
            0x5d => op!(2, 8, self.bit(3, self.registers.l)),
            0x5e => op!(2, 12, self.bit_a8(3, self.registers.hl())),
            0x5f => op!(2, 8, self.bit(3, self.registers.a)),

            0x60 => op!(2, 8, self.bit(4, self.registers.b)),
            0x61 => op!(2, 8, self.bit(4, self.registers.c)),
            0x62 => op!(2, 8, self.bit(4, self.registers.d)),
            0x63 => op!(2, 8, self.bit(4, self.registers.e)),
            0x64 => op!(2, 8, self.bit(4, self.registers.h)),
            0x65 => op!(2, 8, self.bit(4, self.registers.l)),
            0x66 => op!(2, 12, self.bit_a8(4, self.registers.hl())),
            0x67 => op!(2, 8, self.bit(4, self.registers.a)),

            0x68 => op!(2, 8, self.bit(5, self.registers.b)),
            0x69 => op!(2, 8, self.bit(5, self.registers.c)),
            0x6a => op!(2, 8, self.bit(5, self.registers.d)),
            0x6b => op!(2, 8, self.bit(5, self.registers.e)),
            0x6c => op!(2, 8, self.bit(5, self.registers.h)),
            0x6d => op!(2, 8, self.bit(5, self.registers.l)),
            0x6e => op!(2, 12, self.bit_a8(5, self.registers.hl())),
            0x6f => op!(2, 8, self.bit(5, self.registers.a)),

            0x70 => op!(2, 8, self.bit(6, self.registers.b)),
            0x71 => op!(2, 8, self.bit(6, self.registers.c)),
            0x72 => op!(2, 8, self.bit(6, self.registers.d)),
            0x73 => op!(2, 8, self.bit(6, self.registers.e)),
            0x74 => op!(2, 8, self.bit(6, self.registers.h)),
            0x75 => op!(2, 8, self.bit(6, self.registers.l)),
            0x76 => op!(2, 12, self.bit_a8(6, self.registers.hl())),
            0x77 => op!(2, 8, self.bit(6, self.registers.a)),

            0x78 => op!(2, 8, self.bit(7, self.registers.b)),
            0x79 => op!(2, 8, self.bit(7, self.registers.c)),
            0x7a => op!(2, 8, self.bit(7, self.registers.d)),
            0x7b => op!(2, 8, self.bit(7, self.registers.e)),
            0x7c => op!(2, 8, self.bit(7, self.registers.h)),
            0x7d => op!(2, 8, self.bit(7, self.registers.l)),
            0x7e => op!(2, 12, self.bit_a8(7, self.registers.hl())),
            0x7f => op!(2, 8, self.bit(7, self.registers.a)),

            // RES bit, r8
            0x80 => op!(2, 8, self.res(0, self.registers.b) => self.registers.b),
            0x81 => op!(2, 8, self.res(0, self.registers.c) => self.registers.c),
            0x82 => op!(2, 8, self.res(0, self.registers.d) => self.registers.d),
            0x83 => op!(2, 8, self.res(0, self.registers.e) => self.registers.e),
            0x84 => op!(2, 8, self.res(0, self.registers.h) => self.registers.h),
            0x85 => op!(2, 8, self.res(0, self.registers.l) => self.registers.l),
            0x86 => op!(2, 16, self.res_a8(0, self.registers.hl())),
            0x87 => op!(2, 8, self.res(0, self.registers.a) => self.registers.a),

            0x88 => op!(2, 8, self.res(1, self.registers.b) => self.registers.b),
            0x89 => op!(2, 8, self.res(1, self.registers.c) => self.registers.c),
            0x8a => op!(2, 8, self.res(1, self.registers.d) => self.registers.d),
            0x8b => op!(2, 8, self.res(1, self.registers.e) => self.registers.e),
            0x8c => op!(2, 8, self.res(1, self.registers.h) => self.registers.h),
            0x8d => op!(2, 8, self.res(1, self.registers.l) => self.registers.l),
            0x8e => op!(2, 16, self.res_a8(1, self.registers.hl())),
            0x8f => op!(2, 8, self.res(1, self.registers.a) => self.registers.a),

            0x90 => op!(2, 8, self.res(2, self.registers.b) => self.registers.b),
            0x91 => op!(2, 8, self.res(2, self.registers.c) => self.registers.c),
            0x92 => op!(2, 8, self.res(2, self.registers.d) => self.registers.d),
            0x93 => op!(2, 8, self.res(2, self.registers.e) => self.registers.e),
            0x94 => op!(2, 8, self.res(2, self.registers.h) => self.registers.h),
            0x95 => op!(2, 8, self.res(2, self.registers.l) => self.registers.l),
            0x96 => op!(2, 16, self.res_a8(2, self.registers.hl())),
            0x97 => op!(2, 8, self.res(2, self.registers.a) => self.registers.a),

            0x98 => op!(2, 8, self.res(3, self.registers.b) => self.registers.b),
            0x99 => op!(2, 8, self.res(3, self.registers.c) => self.registers.c),
            0x9a => op!(2, 8, self.res(3, self.registers.d) => self.registers.d),
            0x9b => op!(2, 8, self.res(3, self.registers.e) => self.registers.e),
            0x9c => op!(2, 8, self.res(3, self.registers.h) => self.registers.h),
            0x9d => op!(2, 8, self.res(3, self.registers.l) => self.registers.l),
            0x9e => op!(2, 16, self.res_a8(3, self.registers.hl())),
            0x9f => op!(2, 8, self.res(3, self.registers.a) => self.registers.a),

            0xa0 => op!(2, 8, self.res(4, self.registers.b) => self.registers.b),
            0xa1 => op!(2, 8, self.res(4, self.registers.c) => self.registers.c),
            0xa2 => op!(2, 8, self.res(4, self.registers.d) => self.registers.d),
            0xa3 => op!(2, 8, self.res(4, self.registers.e) => self.registers.e),
            0xa4 => op!(2, 8, self.res(4, self.registers.h) => self.registers.h),
            0xa5 => op!(2, 8, self.res(4, self.registers.l) => self.registers.l),
            0xa6 => op!(2, 16, self.res_a8(4, self.registers.hl())),
            0xa7 => op!(2, 8, self.res(4, self.registers.a) => self.registers.a),

            0xa8 => op!(2, 8, self.res(5, self.registers.b) => self.registers.b),
            0xa9 => op!(2, 8, self.res(5, self.registers.c) => self.registers.c),
            0xaa => op!(2, 8, self.res(5, self.registers.d) => self.registers.d),
            0xab => op!(2, 8, self.res(5, self.registers.e) => self.registers.e),
            0xac => op!(2, 8, self.res(5, self.registers.h) => self.registers.h),
            0xad => op!(2, 8, self.res(5, self.registers.l) => self.registers.l),
            0xae => op!(2, 16, self.res_a8(5, self.registers.hl())),
            0xaf => op!(2, 8, self.res(5, self.registers.a) => self.registers.a),

            0xb0 => op!(2, 8, self.res(6, self.registers.b) => self.registers.b),
            0xb1 => op!(2, 8, self.res(6, self.registers.c) => self.registers.c),
            0xb2 => op!(2, 8, self.res(6, self.registers.d) => self.registers.d),
            0xb3 => op!(2, 8, self.res(6, self.registers.e) => self.registers.e),
            0xb4 => op!(2, 8, self.res(6, self.registers.h) => self.registers.h),
            0xb5 => op!(2, 8, self.res(6, self.registers.l) => self.registers.l),
            0xb6 => op!(2, 16, self.res_a8(6, self.registers.hl())),
            0xb7 => op!(2, 8, self.res(6, self.registers.a) => self.registers.a),

            0xb8 => op!(2, 8, self.res(7, self.registers.b) => self.registers.b),
            0xb9 => op!(2, 8, self.res(7, self.registers.c) => self.registers.c),
            0xba => op!(2, 8, self.res(7, self.registers.d) => self.registers.d),
            0xbb => op!(2, 8, self.res(7, self.registers.e) => self.registers.e),
            0xbc => op!(2, 8, self.res(7, self.registers.h) => self.registers.h),
            0xbd => op!(2, 8, self.res(7, self.registers.l) => self.registers.l),
            0xbe => op!(2, 16, self.res_a8(7, self.registers.hl())),
            0xbf => op!(2, 8, self.res(7, self.registers.a) => self.registers.a),

            // SET bit, r8
            0xc0 => op!(2, 8, self.set(0, self.registers.b) => self.registers.b),
            0xc1 => op!(2, 8, self.set(0, self.registers.c) => self.registers.c),
            0xc2 => op!(2, 8, self.set(0, self.registers.d) => self.registers.d),
            0xc3 => op!(2, 8, self.set(0, self.registers.e) => self.registers.e),
            0xc4 => op!(2, 8, self.set(0, self.registers.h) => self.registers.h),
            0xc5 => op!(2, 8, self.set(0, self.registers.l) => self.registers.l),
            0xc6 => op!(2, 16, self.set_a8(0, self.registers.hl())),
            0xc7 => op!(2, 8, self.set(0, self.registers.a) => self.registers.a),

            0xc8 => op!(2, 8, self.set(1, self.registers.b) => self.registers.b),
            0xc9 => op!(2, 8, self.set(1, self.registers.c) => self.registers.c),
            0xca => op!(2, 8, self.set(1, self.registers.d) => self.registers.d),
            0xcb => op!(2, 8, self.set(1, self.registers.e) => self.registers.e),
            0xcc => op!(2, 8, self.set(1, self.registers.h) => self.registers.h),
            0xcd => op!(2, 8, self.set(1, self.registers.l) => self.registers.l),
            0xce => op!(2, 16, self.set_a8(1, self.registers.hl())),
            0xcf => op!(2, 8, self.set(1, self.registers.a) => self.registers.a),

            0xd0 => op!(2, 8, self.set(2, self.registers.b) => self.registers.b),
            0xd1 => op!(2, 8, self.set(2, self.registers.c) => self.registers.c),
            0xd2 => op!(2, 8, self.set(2, self.registers.d) => self.registers.d),
            0xd3 => op!(2, 8, self.set(2, self.registers.e) => self.registers.e),
            0xd4 => op!(2, 8, self.set(2, self.registers.h) => self.registers.h),
            0xd5 => op!(2, 8, self.set(2, self.registers.l) => self.registers.l),
            0xd6 => op!(2, 16, self.set_a8(2, self.registers.hl())),
            0xd7 => op!(2, 8, self.set(2, self.registers.a) => self.registers.a),

            0xd8 => op!(2, 8, self.set(3, self.registers.b) => self.registers.b),
            0xd9 => op!(2, 8, self.set(3, self.registers.c) => self.registers.c),
            0xda => op!(2, 8, self.set(3, self.registers.d) => self.registers.d),
            0xdb => op!(2, 8, self.set(3, self.registers.e) => self.registers.e),
            0xdc => op!(2, 8, self.set(3, self.registers.h) => self.registers.h),
            0xdd => op!(2, 8, self.set(3, self.registers.l) => self.registers.l),
            0xde => op!(2, 16, self.set_a8(3, self.registers.hl())),
            0xdf => op!(2, 8, self.set(3, self.registers.a) => self.registers.a),

            0xe0 => op!(2, 8, self.set(4, self.registers.b) => self.registers.b),
            0xe1 => op!(2, 8, self.set(4, self.registers.c) => self.registers.c),
            0xe2 => op!(2, 8, self.set(4, self.registers.d) => self.registers.d),
            0xe3 => op!(2, 8, self.set(4, self.registers.e) => self.registers.e),
            0xe4 => op!(2, 8, self.set(4, self.registers.h) => self.registers.h),
            0xe5 => op!(2, 8, self.set(4, self.registers.l) => self.registers.l),
            0xe6 => op!(2, 16, self.set_a8(4, self.registers.hl())),
            0xe7 => op!(2, 8, self.set(4, self.registers.a) => self.registers.a),

            0xe8 => op!(2, 8, self.set(5, self.registers.b) => self.registers.b),
            0xe9 => op!(2, 8, self.set(5, self.registers.c) => self.registers.c),
            0xea => op!(2, 8, self.set(5, self.registers.d) => self.registers.d),
            0xeb => op!(2, 8, self.set(5, self.registers.e) => self.registers.e),
            0xec => op!(2, 8, self.set(5, self.registers.h) => self.registers.h),
            0xed => op!(2, 8, self.set(5, self.registers.l) => self.registers.l),
            0xee => op!(2, 16, self.set_a8(5, self.registers.hl())),
            0xef => op!(2, 8, self.set(5, self.registers.a) => self.registers.a),

            0xf0 => op!(2, 8, self.set(6, self.registers.b) => self.registers.b),
            0xf1 => op!(2, 8, self.set(6, self.registers.c) => self.registers.c),
            0xf2 => op!(2, 8, self.set(6, self.registers.d) => self.registers.d),
            0xf3 => op!(2, 8, self.set(6, self.registers.e) => self.registers.e),
            0xf4 => op!(2, 8, self.set(6, self.registers.h) => self.registers.h),
            0xf5 => op!(2, 8, self.set(6, self.registers.l) => self.registers.l),
            0xf6 => op!(2, 16, self.set_a8(6, self.registers.hl())),
            0xf7 => op!(2, 8, self.set(6, self.registers.a) => self.registers.a),

            0xf8 => op!(2, 8, self.set(7, self.registers.b) => self.registers.b),
            0xf9 => op!(2, 8, self.set(7, self.registers.c) => self.registers.c),
            0xfa => op!(2, 8, self.set(7, self.registers.d) => self.registers.d),
            0xfb => op!(2, 8, self.set(7, self.registers.e) => self.registers.e),
            0xfc => op!(2, 8, self.set(7, self.registers.h) => self.registers.h),
            0xfd => op!(2, 8, self.set(7, self.registers.l) => self.registers.l),
            0xfe => op!(2, 16, self.set_a8(7, self.registers.hl())),
            0xff => op!(2, 8, self.set(7, self.registers.a) => self.registers.a),
        };

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
