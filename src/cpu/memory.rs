pub struct MemoryBus {
    memory: [u8; 0xFFFF],
}

impl Default for MemoryBus {
    fn default() -> Self {
        Self {
            memory: [0u8; 0xFFFF],
        }
    }
}

impl MemoryBus {
    pub fn read_u8(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    pub fn write_u8(&mut self, _addr: u16, _value: u8) {
        todo!()
    }

    /// Reads a u16 from memory, this function will swap the byte order from
    /// the way it is stored in memory. E.g. if [0x12, 0x34] is stored in memory
    /// the returned u16 will be 0x3412
    pub fn read_u16(&self, addr: u16) -> u16 {
        let upper = self.memory[addr as usize] as u16;
        let lower = self.memory[(addr + 1) as usize] as u16;
        (upper << 8) | lower
    }

    /// Writes a u16 value to memory. It will swap the order of the bytes
    /// before writing. E.g. 0x1234 will be written as 0x3412 to memory.
    pub fn write_u16(&mut self, addr: u16, value: u16) {
        let addr = addr as usize;
        let lower = value as u8;
        let upper = (value >> 8) as u8;
        self.memory[addr + 1] = upper;
        self.memory[addr] = lower;
    }
}
