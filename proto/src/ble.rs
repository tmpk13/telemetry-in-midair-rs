//! BLE GATT extensions served by the ESP32-C6.
//!
//! The board reuses the gps-proto service and its position/config/ack
//! characteristics, so the existing gps-gui-rs app connects and streams
//! positions unchanged. This module adds board-specific characteristics
//! (continuing the same UUID sequence) and new config command ids on the
//! existing config characteristic.

/// Name the ESP32-C6 advertises under. The service UUID (which the app
/// filters scans by) stays `gps_proto::packet::SERVICE_UUID`.
pub const DEVICE_NAME: &str = "GPS-C6";

/// [`crate::link::Telemetry`] wire format, notify + read.
pub const TELEMETRY_UUID: &str = "c3a10005-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Bulk transfer (radio TOML config / WIO firmware), write.
pub const BULK_UUID: &str = "c3a10006-9f6e-4b2c-8f5a-2e32c3b1e5d0";
/// Remote position: `[src u8, rssi i16le, PositionPacket 20B]`, notify + read.
pub const REMOTE_UUID: &str = "c3a10007-9f6e-4b2c-8f5a-2e32c3b1e5d0";

pub const TELEMETRY_UUID_U128: u128 = 0xc3a10005_9f6e_4b2c_8f5a_2e32c3b1e5d0;
pub const BULK_UUID_U128: u128 = 0xc3a10006_9f6e_4b2c_8f5a_2e32c3b1e5d0;
pub const REMOTE_UUID_U128: u128 = 0xc3a10007_9f6e_4b2c_8f5a_2e32c3b1e5d0;

/// Remote position value length (src + rssi + packet).
pub const REMOTE_LEN: usize = 1 + 2 + gps_proto::packet::POSITION_PACKET_LEN;

// -- Config command ids (on the gps-proto config characteristic) -------------
//
// Ids 0x01-0x0F are reserved for gps-proto (0x01 = notify interval).
// Payload format is gps-proto's `[id, len, value]`; acks come back on the
// ack characteristic with the applied value.

/// `u8` 0/1: enable the GPS/LoRa power rail (AP2112K LDO on GPIO2).
/// 0 powers off both the WIO-E5 and the GPS entirely.
pub const CFG_PWR_EN: u8 = 0x10;
/// `u8` 0/1: 1 sends WIO_SLEEP(1) over the link; 0 wakes it (WIO_SLEEP(0),
/// with a reset pulse as fallback when the WIO does not ack).
pub const CFG_WIO_SLEEP: u8 = 0x11;
/// `u8` 0/1: GPS backup mode on/off (forwarded to the WIO).
pub const CFG_GPS_SLEEP: u8 = 0x12;
/// `u32` seconds: ESP deep-sleep wake-check interval. While set (non-zero)
/// the ESP deep-sleeps whenever no central is connected, waking every
/// interval to advertise for a short window. 0 disables sleep mode.
pub const CFG_ESP_SLEEP_S: u8 = 0x13;

/// Clamp range for [`CFG_ESP_SLEEP_S`].
pub const ESP_SLEEP_MIN_S: u32 = 5;
pub const ESP_SLEEP_MAX_S: u32 = 24 * 3600;

// -- Bulk transfer protocol (writes on [`BULK_UUID`]) -------------------------
//
// Each write is one op. Status comes back on the ack characteristic with
// id [`ACK_ID_BULK`]: status ACK_OK and the next expected seq as the value,
// or a non-zero status on error.

/// Bulk op: `[OP_BEGIN, kind u8, total_len u32le, crc32 u32le,
/// version u16le]` (version is 0 for TOML config).
pub const OP_BEGIN: u8 = 0x01;
/// Bulk op: `[OP_DATA, seq u16le, bytes...]`.
pub const OP_DATA: u8 = 0x02;
/// Bulk op: `[OP_END]`.
pub const OP_END: u8 = 0x03;
/// Bulk op: `[OP_ABORT]`.
pub const OP_ABORT: u8 = 0x04;

/// Bulk kind: radio TOML config, forwarded to the WIO and saved to SD.
pub const KIND_TOML: u8 = 1;
/// Bulk kind: WIO-E5 firmware image for the DFU partition.
pub const KIND_FIRMWARE: u8 = 2;

/// Ack id used for bulk transfer status on the ack characteristic.
pub const ACK_ID_BULK: u8 = 0x20;

/// Ack statuses beyond gps-proto's ACK_OK/ACK_UNKNOWN_ID/ACK_BAD_VALUE.
pub const ACK_WIO_ERROR: u8 = 0x10;
pub const ACK_WIO_TIMEOUT: u8 = 0x11;
pub const ACK_BAD_STATE: u8 = 0x12;

/// Max data bytes per OP_DATA write. Fits a 251-byte ATT payload after the
/// 3-byte op header while staying under the UART link chunk size.
pub const BULK_DATA_MAX: usize = crate::link::DATA_CHUNK;

#[cfg(test)]
mod tests {
    use gps_proto::str_eq;

    /// The trouble `#[gatt_service]` macro needs const u128s; make sure the
    /// string forms cannot drift from them (mirrors gps-proto's own test).
    #[test]
    fn uuid_strings_match_u128() {
        fn to_u128(s: &str) -> u128 {
            s.bytes()
                .filter(|&b| b != b'-')
                .fold(0u128, |v, b| v << 4 | (b as char).to_digit(16).unwrap() as u128)
        }
        assert_eq!(to_u128(super::TELEMETRY_UUID), super::TELEMETRY_UUID_U128);
        assert_eq!(to_u128(super::BULK_UUID), super::BULK_UUID_U128);
        assert_eq!(to_u128(super::REMOTE_UUID), super::REMOTE_UUID_U128);
        // Same service as the C3 beacon, different characteristic ids.
        assert!(str_eq(
            gps_proto::packet::SERVICE_UUID,
            "c3a10001-9f6e-4b2c-8f5a-2e32c3b1e5d0"
        ));
    }
}
