//! Adapter bridging [`PacketRadio`] (packet I/O) to [`embedded_io`]
//! (byte-stream I/O) for use with `embedded-nano-mesh`.
//!
//! `embedded-nano-mesh` expects a byte-stream interface (designed for
//! UART-like transports).  LoRa radios are packet-oriented.  This
//! adapter buffers writes and flushes them as a single radio packet,
//! and buffers a received packet so it can be drained via `Read`.

use crate::radio::PacketRadio;
use embedded_io::{ErrorType, Read, ReadReady, Write};

/// Internal buffer size.  Must be >= the serialized nano-mesh packet
/// size (~40 bytes for 32-byte payload).
const BUF_SIZE: usize = 64;

/// Error type for the adapter.
#[derive(Debug)]
pub enum IoError {
    /// The underlying radio operation failed.
    Radio,
    /// Write buffer overflow — packet too large.
    BufferFull,
}

impl embedded_io::Error for IoError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// Bridges a [`PacketRadio`] to the [`Read`] + [`Write`] + [`ReadReady`]
/// traits that `embedded-nano-mesh` requires.
///
/// **Write side:** bytes are buffered; [`Write::flush`] transmits them
/// as a single radio packet.
///
/// **Read side:** [`ReadReady::read_ready`] polls the radio for a
/// packet and buffers it; [`Read::read`] drains from that buffer.
pub struct LoraIo<R: PacketRadio> {
    radio: R,
    // RX buffering
    rx_buf: [u8; BUF_SIZE],
    rx_len: usize,
    rx_pos: usize,
    last_rssi: i16,
    // `platform::millis()` timestamp of the last received packet.
    last_rx_ms: u32,
    have_rx: bool,
    // TX buffering
    tx_buf: [u8; BUF_SIZE],
    tx_len: usize,
}

impl<R: PacketRadio> LoraIo<R> {
    /// Wrap a packet radio in the byte-stream adapter.
    pub fn new(radio: R) -> Self {
        Self {
            radio,
            rx_buf: [0u8; BUF_SIZE],
            rx_len: 0,
            rx_pos: 0,
            last_rssi: 0,
            last_rx_ms: 0,
            have_rx: false,
            tx_buf: [0u8; BUF_SIZE],
            tx_len: 0,
        }
    }

    /// RSSI of the last successfully received packet (dBm).
    pub fn last_rssi(&self) -> i16 {
        self.last_rssi
    }

    /// `platform::millis()` timestamp of the last received packet, or
    /// `None` if nothing has been received yet.
    pub fn last_rx_ms(&self) -> Option<u32> {
        self.have_rx.then_some(self.last_rx_ms)
    }

    /// Record the RSSI and arrival time of a freshly received packet.
    fn note_rx(&mut self, rssi: i16) {
        self.last_rssi = rssi;
        self.last_rx_ms = crate::platform::millis();
        self.have_rx = true;
    }

    /// Borrow the inner radio for diagnostics or direct access.
    pub fn inner(&mut self) -> &mut R {
        &mut self.radio
    }
}

impl<R: PacketRadio> ErrorType for LoraIo<R> {
    type Error = IoError;
}

impl<R: PacketRadio> ReadReady for LoraIo<R> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        // Still have buffered data from a previous packet.
        if self.rx_pos < self.rx_len {
            return Ok(true);
        }

        // Try to poll the radio for a new packet.
        match self.radio.poll_recv(&mut self.rx_buf) {
            Ok(Some((len, rssi))) => {
                self.rx_len = len;
                self.rx_pos = 0;
                self.note_rx(rssi);
                Ok(true)
            }
            Ok(None) => Ok(false),
            Err(_) => Err(IoError::Radio),
        }
    }
}

impl<R: PacketRadio> Read for LoraIo<R> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // If no buffered data, try one non-blocking poll.
        if self.rx_pos >= self.rx_len {
            match self.radio.poll_recv(&mut self.rx_buf) {
                Ok(Some((len, rssi))) => {
                    self.rx_len = len;
                    self.rx_pos = 0;
                    self.note_rx(rssi);
                }
                Ok(None) => return Ok(0),
                Err(_) => return Err(IoError::Radio),
            }
        }

        let available = self.rx_len - self.rx_pos;
        let n = available.min(buf.len());
        buf[..n].copy_from_slice(&self.rx_buf[self.rx_pos..self.rx_pos + n]);
        self.rx_pos += n;
        Ok(n)
    }
}

impl<R: PacketRadio> Write for LoraIo<R> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let space = BUF_SIZE - self.tx_len;
        if space == 0 {
            return Err(IoError::BufferFull);
        }
        let n = buf.len().min(space);
        self.tx_buf[self.tx_len..self.tx_len + n].copy_from_slice(&buf[..n]);
        self.tx_len += n;
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        if self.tx_len > 0 {
            self.radio
                .send(&self.tx_buf[..self.tx_len])
                .map_err(|_| IoError::Radio)?;
            self.tx_len = 0;
        }
        Ok(())
    }
}
