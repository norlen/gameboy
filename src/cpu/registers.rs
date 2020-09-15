#[derive(Default)]
pub struct Registers {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,

    pub f: Flags,

    pub sp: u16,
    pub pc: u16,
}

impl Registers {
    pub fn af(&self) -> u16 {
        combine(self.a, self.f.0)
    }

    pub fn set_af(&mut self, value: u16) {
        let (a, f) = split(value);
        self.a = a;
        self.f.0 = f;
    }

    pub fn bc(&self) -> u16 {
        combine(self.b, self.c)
    }

    pub fn set_bc(&mut self, value: u16) {
        let (b, c) = split(value);
        self.b = b;
        self.c = c;
    }

    pub fn de(&self) -> u16 {
        combine(self.d, self.e)
    }

    pub fn set_de(&mut self, value: u16) {
        let (d, e) = split(value);
        self.d = d;
        self.e = e;
    }

    pub fn hl(&self) -> u16 {
        combine(self.h, self.l)
    }

    pub fn set_hl(&mut self, value: u16) {
        let (h, l) = split(value);
        self.h = h;
        self.l = l;
    }
}

/// Combines two 8-bit values into a single 16-bit value. With high in the higher
/// 8-bits, and low in the lower 8-bits.
fn combine(high: u8, low: u8) -> u16 {
    (high as u16) << 8 | low as u16
}

/// Splits a 16-bit value into two 8-bit values, the first value in the tuple
/// returned is the highest 8-bits, and the second value the lower 8-bits.
fn split(value: u16) -> (u8, u8) {
    let high = ((value & 0xff00) >> 8) as u8;
    let low = (value & 0x00ff) as u8;
    (high, low)
}

#[derive(Debug, Default)]
pub struct Flags(pub u8);

impl Flags {
    pub const ZERO: u8 = 7;
    pub const SUBTRACT: u8 = 6;
    pub const HALF_CARRY: u8 = 5;
    pub const CARRY: u8 = 4;

    pub fn zero(&self) -> u8 {
        (self.0 >> Self::ZERO) & 0b1
    }

    pub fn subtract(&self) -> u8 {
        (self.0 >> Self::SUBTRACT) & 0b1
    }

    pub fn half_carry(&self) -> u8 {
        (self.0 >> Self::HALF_CARRY) & 0b1
    }

    pub fn carry(&self) -> u8 {
        (self.0 >> Self::CARRY) & 0b1
    }

    pub fn zero_is_set(&self) -> bool {
        (self.0 >> Self::ZERO) & 0b1 != 0
    }

    pub fn subtract_is_set(&self) -> bool {
        (self.0 >> Self::SUBTRACT) & 0b1 != 0
    }

    pub fn half_carry_is_set(&self) -> bool {
        (self.0 >> Self::HALF_CARRY) & 0b1 != 0
    }

    pub fn carry_is_set(&self) -> bool {
        (self.0 >> Self::CARRY) & 0b1 != 0
    }

    pub fn setc(&mut self, condition: bool, bit: u8) {
        if condition {
            self.set(bit);
        } else {
            self.clear(bit);
        }
    }

    pub fn set(&mut self, bit: u8) {
        self.0 |= 0b1 << bit;
    }

    pub fn clear(&mut self, bit: u8) {
        self.0 &= !(0b1 << bit);
    }
}

impl From<u8> for Flags {
    fn from(v: u8) -> Self {
        Self(v)
    }
}

impl Into<u8> for Flags {
    fn into(self) -> u8 {
        self.0
    }
}
