//! WIO-E5 (STM32WLE5JC) firmware library for the telemetry-in-midair board.
//!
//! The module owns the MAX-M10 GPS (USART1), the LoRa radio (integrated
//! SX1262), the SD card (SPI1, FAT), and the UART link to the ESP32-C6
//! (USART2). See `src/bin/main.rs` for the RTIC application tying it
//! together.

#![no_std]

/// Prints only when the `debug` cargo feature is enabled.
#[macro_export]
macro_rules! debug_println {
    ($($arg:tt)*) => {
        if cfg!(feature = "debug") {
            rtt_target::rprintln!($($arg)*);
        }
    };
}

/// Print a status line to RTT and forward it to the ESP over the link
/// ([`crate::esplink::EspLink::send_status`]), where it reaches the USB
/// console and BLE. `$esp` is the [`crate::esplink::EspLink`].
#[macro_export]
macro_rules! status_println {
    ($esp:expr, $($arg:tt)*) => {{
        rtt_target::rprintln!($($arg)*);
        $esp.send_status(::core::format_args!($($arg)*));
    }};
}

pub mod boot_state;
pub mod cfgstore;
pub mod cfgxfer;
pub mod esplink;
pub mod fwupdate;
pub mod gps;
pub mod leds;
pub mod node;
pub mod platform;
pub mod radio;
pub mod sdcard;
pub mod sdlog;
pub mod watchdog;

pub use node::{Node, Received, TxError};

/// Firmware version reported over the link and used by the DFU handshake.
/// Set at compile time via the `FW_VERSION` environment variable.
pub const FIRMWARE_VERSION: u16 = {
    match option_env!("FW_VERSION") {
        Some(s) => {
            let bytes = s.as_bytes();
            assert!(!bytes.is_empty(), "FW_VERSION must not be empty");
            let mut i = 0;
            let mut n: u16 = 0;
            while i < bytes.len() {
                let d = bytes[i];
                assert!(d >= b'0' && d <= b'9', "FW_VERSION must be a number 0-65535");
                let next = n as u32 * 10 + (d - b'0') as u32;
                assert!(next <= 65535, "FW_VERSION must be 0-65535");
                n = next as u16;
                i += 1;
            }
            n
        }
        None => 1,
    }
};
