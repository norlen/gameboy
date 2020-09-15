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

    pub fn read_u16(&self, _addr: u16) -> u16 {
        todo!()
    }
}
