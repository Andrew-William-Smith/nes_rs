/// Interface necessary for a readable memory device.
pub trait Readable {
    /// Attempt to read a byte of data from the specified memory address.  If the address is outside
    /// the range handled by this device, return `None`; otherwise, return the requested byte.
    fn read_byte(&self, address: u16) -> Option<u8>;

    /// Attempt to read a 16-bit word of data starting at the specified memory address.  If the
    /// address of either byte in the word is outside the range handled by this device, return
    /// `None`; otherwise, return the requested byte.
    fn read_word(&self, address: u16) -> Option<u16>;
}

/// Interface necessary for a writable memory device.
pub trait Writable {
    /// Attempt to write the specified byte of data to the specified memory address.  If the address
    /// is outside the range handled by this device, return `false`; otherwise, return `true` to
    /// indicate a successful write.
    fn write_byte(&mut self, address: u16, data: u8) -> bool;
}
