//! MAX-M10 GPS receiver on USART1: PB7 = RX (module TX -> MCU), PB6 = TX
//! (MCU -> module, used for UBX power commands), 9600 8N1 factory default.
//! PB10 drives the module's EXTINT pin for wake-from-backup.
//!
//! NMEA parsing is shared with the ESP32-C3 beacon via gps-proto: RMC owns
//! the fix flag, position, motion and time; GGA contributes altitude and
//! satellite count. The folded state is a wire-ready [`PositionPacket`].

use cortex_m::interrupt::CriticalSection;
use gps_proto::nmea::{self, Sentence};
use gps_proto::packet::{PositionPacket, FLAG_FIX};
use stm32wlxx_hal::{
    embedded_hal::serial::{Read, Write},
    gpio::{pins, Output, OutputArgs, PinState},
    pac,
    uart::{self, Uart1},
};

/// NMEA line rate - the u-blox M10 factory default.
pub const BAUD: u32 = 9_600;

/// Longest NMEA sentence: 82 characters including "$" and CRLF.
const NMEA_MAX: usize = 82;

pub struct Gps {
    uart: Uart1<pins::B7, pins::B6>,
    extint: Output<pins::B10>,
    line: [u8; NMEA_MAX],
    len: usize,
    in_line: bool,
    /// Folded position state, wire-ready.
    packet: PositionPacket,
    /// Set when an accepted sentence updated `packet` since the last take.
    updated: bool,
    /// Whether the module was put into backup mode.
    pub sleeping: bool,
}

impl Gps {
    pub fn new(
        usart1: pac::USART1,
        b6: pins::B6,
        b7: pins::B7,
        b10: pins::B10,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        // HSI16 keeps the baud rate exact and independent of the MSI
        // system clock.
        let uart = Uart1::new(usart1, BAUD, uart::Clk::Hsi16, rcc)
            .enable_rx(b7, cs)
            .enable_tx(b6, cs);
        const ARGS: OutputArgs = OutputArgs {
            level: PinState::Low,
            ..OutputArgs::new()
        };
        Self {
            uart,
            extint: Output::new(b10, &ARGS, cs),
            line: [0; NMEA_MAX],
            len: 0,
            in_line: false,
            packet: PositionPacket::default(),
            updated: false,
            sleeping: false,
        }
    }

    /// Latest folded position snapshot.
    pub fn packet(&self) -> PositionPacket {
        self.packet
    }

    /// Whether the position state changed since the last call.
    pub fn take_updated(&mut self) -> bool {
        core::mem::take(&mut self.updated)
    }

    pub fn has_fix(&self) -> bool {
        self.packet.flags & FLAG_FIX != 0
    }

    /// Clear sticky UART error flags (overrun keeps erroring until
    /// acknowledged in ICR). Overruns are expected: other work in the
    /// main loop can block longer than one character time.
    fn clear_errors(&mut self) {
        // The HAL keeps the register block private; USART1 is owned by
        // `self.uart` so this access is exclusive.
        unsafe {
            let usart1 = &*pac::USART1::PTR;
            usart1.icr.write(|w| {
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

    /// Drain the UART and fold complete sentences into the position
    /// state. Call every main-loop iteration.
    pub fn poll(&mut self) {
        loop {
            let byte = match self.uart.read() {
                Ok(b) => b,
                Err(nb::Error::WouldBlock) => return,
                Err(nb::Error::Other(_)) => {
                    // Lost bytes: drop the partial line and resync.
                    self.clear_errors();
                    self.in_line = false;
                    self.len = 0;
                    continue;
                }
            };
            match byte {
                b'$' => {
                    self.line[0] = b'$';
                    self.len = 1;
                    self.in_line = true;
                }
                b'\r' | b'\n' => {
                    if self.in_line {
                        if let Ok(s) = core::str::from_utf8(&self.line[..self.len]) {
                            if let Some(sentence) = nmea::parse(s) {
                                self.fold(sentence);
                                self.updated = true;
                            }
                        }
                    }
                    self.in_line = false;
                    self.len = 0;
                }
                _ if self.in_line => {
                    if self.len < NMEA_MAX {
                        self.line[self.len] = byte;
                        self.len += 1;
                    } else {
                        // Overlong garbage: resync on the next '$'.
                        self.in_line = false;
                        self.len = 0;
                    }
                }
                _ => {}
            }
        }
    }

    fn fold(&mut self, s: Sentence) {
        let p = &mut self.packet;
        match s {
            Sentence::Rmc(rmc) => {
                if rmc.valid {
                    if let (Some(lat), Some(lon)) = (rmc.lat_e7, rmc.lon_e7) {
                        p.lat_e7 = lat;
                        p.lon_e7 = lon;
                        p.flags |= FLAG_FIX;
                    }
                } else {
                    p.flags &= !FLAG_FIX;
                }
                if let Some(v) = rmc.speed_cms {
                    p.speed_cms = v;
                }
                if let Some(v) = rmc.course_cdeg {
                    p.course_cdeg = v;
                }
                if let Some(v) = rmc.tod_ms {
                    p.tod_ms = v;
                }
            }
            Sentence::Gga(gga) => {
                if let Some(v) = gga.alt_dm {
                    p.alt_dm = v;
                }
                if let Some(v) = gga.sats {
                    p.sats = v;
                }
            }
        }
    }

    fn write_all(&mut self, bytes: &[u8]) {
        for &b in bytes {
            let _ = nb::block!(self.uart.write(b));
        }
        let _ = nb::block!(self.uart.flush());
    }

    /// Put the module into backup mode (UBX-RXM-PMREQ, indefinite, wake
    /// on EXTINT or UART RX activity).
    pub fn sleep(&mut self) {
        // Version-0 16-byte payload: version, reserved[3], duration (0 =
        // until wake source), flags (bit1 = backup), wakeupSources
        // (bit3 = uartrx, bit5 = extint0).
        let mut frame = [0u8; 8 + 16];
        frame[0] = 0xB5;
        frame[1] = 0x62;
        frame[2] = 0x02; // class RXM
        frame[3] = 0x41; // id PMREQ
        frame[4] = 16; // length lo
        frame[5] = 0;
        // payload[0..4]: version + reserved = 0
        // payload[4..8]: duration = 0
        frame[6 + 8..6 + 12].copy_from_slice(&2u32.to_le_bytes()); // flags: backup
        frame[6 + 12..6 + 16].copy_from_slice(&((1u32 << 3) | (1u32 << 5)).to_le_bytes());
        // 8-bit Fletcher checksum over class..payload.
        let (mut ck_a, mut ck_b) = (0u8, 0u8);
        for &b in &frame[2..6 + 16] {
            ck_a = ck_a.wrapping_add(b);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        frame[6 + 16] = ck_a;
        frame[7 + 16] = ck_b;
        self.write_all(&{ frame });
        self.extint.set_level_low();
        self.sleeping = true;
        self.packet.flags &= !FLAG_FIX;
    }

    /// Wake the module from backup: EXTINT pulse plus UART traffic.
    pub fn wake(&mut self) {
        self.extint.set_level_high();
        // Hold EXTINT high a few ms so the edge is registered.
        cortex_m::asm::delay(crate::platform::SYSCLK_HZ / 1000 * 5);
        self.extint.set_level_low();
        self.write_all(&[0xFF, 0xFF]);
        self.sleeping = false;
    }
}
