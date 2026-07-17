//! UART link to the ESP32-C6: USART2, PA2 = TX / PA3 = RX, 115200 8N1
//! (the ESP side is UART0). Frames use midair-proto's link format.
//!
//! This module owns the byte transport and the peer's radio-busy flag;
//! frame semantics are handled by the main loop.

use cortex_m::interrupt::CriticalSection;
use midair_proto::link::{self, FrameBuf, FrameParser};
use stm32wlxx_hal::{
    embedded_hal::serial::{Read, Write},
    gpio::pins,
    pac,
    uart::{self, Uart2},
};

pub struct EspLink {
    uart: Uart2<pins::A3, pins::A2>,
    parser: FrameParser,
    out: FrameBuf,
    /// Last time the ESP flagged its radio busy; None when cleared.
    busy_since_ms: Option<u32>,
}

/// Fixed-size `core::fmt::Write` sink for [`EspLink::send_status`]; silently
/// drops anything past [`link::LOG_MAX`] so a long line just truncates.
struct StatusWriter {
    buf: [u8; link::LOG_MAX],
    len: usize,
}

impl core::fmt::Write for StatusWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

impl EspLink {
    pub fn new(
        usart2: pac::USART2,
        a2: pins::A2,
        a3: pins::A3,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        // HSI16 keeps 115200 exact and independent of the MSI sysclk.
        let uart = Uart2::new(usart2, link::BAUD, uart::Clk::Hsi16, rcc)
            .enable_rx(a3, cs)
            .enable_tx(a2, cs);
        Self {
            uart,
            parser: FrameParser::new(),
            out: FrameBuf::new(),
            busy_since_ms: None,
        }
    }

    /// Clear sticky UART error flags (overrun keeps erroring until
    /// acknowledged in ICR).
    fn clear_errors(&mut self) {
        // The HAL keeps the register block private; USART2 is owned by
        // `self.uart` so this access is exclusive.
        unsafe {
            let usart2 = &*pac::USART2::PTR;
            usart2.icr.write(|w| {
                w.orecf()
                    .set_bit()
                    .fecf()
                    .set_bit()
                    .pecf()
                    .set_bit()
                    .ncf()
                    .set_bit()
            });
        }
    }

    /// Pump the receiver. Returns `Some((cmd, payload_len))` when a
    /// complete frame arrived; fetch it with [`payload`](Self::payload)
    /// before the next poll.
    pub fn poll(&mut self) -> Option<(u8, usize)> {
        loop {
            let byte = match self.uart.read() {
                Ok(b) => b,
                Err(nb::Error::WouldBlock) => return None,
                Err(nb::Error::Other(_)) => {
                    self.clear_errors();
                    continue;
                }
            };
            if self.parser.feed(byte) {
                let f = self.parser.frame();
                return Some((f.cmd, f.payload.len()));
            }
        }
    }

    /// Payload of the frame returned by the last [`poll`](Self::poll).
    pub fn payload(&self) -> &[u8] {
        self.parser.frame().payload
    }

    /// Blocking frame write (a full 197-byte data frame takes ~17 ms at
    /// 115200 baud).
    pub fn send(&mut self, cmd: u8, payload: &[u8]) {
        self.out.build(cmd, payload);
        for i in 0..self.out.len {
            let byte = self.out.buf[i];
            let _ = nb::block!(self.uart.write(byte));
        }
        let _ = nb::block!(Write::flush(&mut self.uart));
    }

    pub fn send_ack(&mut self, cmd: u8, value: u16) {
        let mut p = [0u8; 3];
        p[0] = cmd;
        p[1..3].copy_from_slice(&value.to_le_bytes());
        self.send(link::resp::ACK, &p);
    }

    /// Send a human-readable status line to the ESP ([`link::msg::LOG`]).
    /// The text is formatted into a stack buffer and truncated to
    /// [`link::LOG_MAX`] bytes; the ESP prints it and notifies it over BLE.
    pub fn send_status(&mut self, args: core::fmt::Arguments) {
        use core::fmt::Write as _;
        let mut w = StatusWriter {
            buf: [0u8; link::LOG_MAX],
            len: 0,
        };
        let _ = w.write_fmt(args);
        self.send(link::msg::LOG, &w.buf[..w.len]);
    }

    pub fn send_nak(&mut self, cmd: u8, err: u8) {
        self.send(link::resp::NAK, &[cmd, err]);
    }

    /// Record a RADIO_BUSY flag from the ESP.
    pub fn set_peer_busy(&mut self, busy: bool, now_ms: u32) {
        self.busy_since_ms = busy.then_some(now_ms);
    }

    /// Whether the ESP radio is currently flagged busy (with staleness
    /// timeout, so a lost clear frame cannot wedge LoRa TX forever).
    pub fn peer_busy(&self, now_ms: u32) -> bool {
        match self.busy_since_ms {
            Some(t) => now_ms.wrapping_sub(t) < link::RADIO_BUSY_TIMEOUT_MS,
            None => false,
        }
    }
}
