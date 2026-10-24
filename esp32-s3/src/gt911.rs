//! Minimal GT911 capacitive touch controller driver (blocking I2C).
//!
//! Register map (GT911 datasheet, chapter "Register Map"):
//!   0x814E  Status: bit 7 = data ready, bits 0..3 = number of points
//!   0x814F  Data: reading here clears the "data ready" bit
//!   0x8150  Point coordinates, 8 bytes per point
//!
//! The Waveshare board leaves INT and RST unconnected, so the controller is
//! polled and its address is probed at run time.

use esp_hal::i2c::master::I2c;

pub const ADDRESS_DEFAULT: u8 = 0x5d;
pub const ADDRESS_BACKUP: u8 = 0x14;

const REG_STATUS: [u8; 2] = [0x81, 0x4e];
const REG_DATA: [u8; 2] = [0x81, 0x4f];

const STATUS_READY: u8 = 0x80;
const MAX_POINTS: usize = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
}

pub struct Gt911<'d> {
    i2c: I2c<'d, esp_hal::Blocking>,
    address: u8,
}

impl<'d> Gt911<'d> {
    /// Probes both documented addresses and returns a driver for the one that
    /// acknowledges.
    pub fn new(mut i2c: I2c<'d, esp_hal::Blocking>) -> Option<Self> {
        let mut probe = [0u8; 1];
        for address in [ADDRESS_DEFAULT, ADDRESS_BACKUP] {
            if i2c.write_read(address, &REG_STATUS, &mut probe).is_ok() {
                return Some(Self { i2c, address });
            }
        }
        None
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    /// Reads the current touch points. An empty slice means "no touch".
    ///
    /// Returns the number of points written into `out`.
    pub fn read(&mut self, out: &mut [TouchPoint]) -> usize {
        let mut status = [0u8; 1];
        if self.i2c.write_read(self.address, &REG_STATUS, &mut status).is_err() {
            return 0;
        }
        if status[0] & STATUS_READY == 0 {
            return 0;
        }

        let count = (status[0] & 0x0f) as usize;
        if count == 0 {
            return 0;
        }
        let count = count.min(out.len()).min(MAX_POINTS);

        // Reading the data register also acknowledges the interrupt-less
        // "data ready" flag, so this doubles as the clear.
        let mut raw = [0u8; 1 + 8 * MAX_POINTS];
        let read = self
            .i2c
            .write_read(self.address, &REG_DATA, &mut raw[..1 + count * 8]);
        if read.is_err() {
            return 0;
        }

        for (slot, chunk) in out.iter_mut().zip(raw[1..1 + count * 8].chunks_exact(8)) {
            *slot = TouchPoint {
                x: u16::from_le_bytes([chunk[1], chunk[2]]),
                y: u16::from_le_bytes([chunk[3], chunk[4]]),
            };
        }
        count
    }
}
