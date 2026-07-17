//! Shared definitions for the telemetry-in-midair board.
//!
//! Three consumers depend on this crate so the wire formats cannot drift:
//! the ESP32-C6 firmware (`esp/`), the WIO-E5 firmware (`wio/`), and host
//! tests (`cargo test` in this directory).
//!
//! - [`link`]: the framed UART protocol between the ESP32-C6 and the WIO-E5
//!   (USART2 on the WIO side), including firmware-update and radio-config
//!   transfer commands.
//! - [`lora`]: payload formats sent over the LoRa mesh.
//! - [`ble`]: BLE GATT extensions on top of the gps-proto service (extra
//!   characteristic UUIDs and config command ids).
//! - [`radiocfg`]: the radio TOML configuration file format and its parser.
//!
//! The BLE position/ack protocol itself lives in the shared `gps-proto`
//! crate (re-exported here) so the existing gps-gui-rs app keeps working.

#![cfg_attr(not(test), no_std)]

pub use gps_proto;

pub mod ble;
pub mod link;
pub mod lora;
pub mod radiocfg;
