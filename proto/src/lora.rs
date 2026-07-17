//! Payload formats carried over the LoRa mesh (embedded-nano-mesh).
//!
//! Mesh payloads are at most 32 bytes. The first byte is a type tag.
//! `embedded-nano-mesh` pads payloads with 0x00 to full capacity, so
//! binary formats carry their own length or are fixed-size; receivers
//! must not truncate tagged binary payloads at the first null byte.

use gps_proto::packet::{PositionPacket, POSITION_PACKET_LEN};

/// Position broadcast: `[POSITION tag] [PositionPacket 20B]`.
pub const MSG_POSITION: u8 = 0x50;

/// Encoded position message length.
pub const POSITION_MSG_LEN: usize = 1 + POSITION_PACKET_LEN;

/// The 0xF0-0xF6 range is reserved for the legacy OTA-over-mesh protocol
/// of the long-range-radio nodes; treat it as binary too.
pub fn is_binary(data: &[u8]) -> bool {
    matches!(data.first(), Some(&MSG_POSITION) | Some(0xF0..=0xF6))
}

/// Encode a position broadcast.
pub fn encode_position(p: &PositionPacket) -> [u8; POSITION_MSG_LEN] {
    let mut b = [0u8; POSITION_MSG_LEN];
    b[0] = MSG_POSITION;
    b[1..].copy_from_slice(&p.encode());
    b
}

/// Decode a position broadcast. Trailing padding from the mesh layer is
/// tolerated (the packet length is fixed).
pub fn decode_position(data: &[u8]) -> Option<PositionPacket> {
    if data.first() != Some(&MSG_POSITION) {
        return None;
    }
    PositionPacket::decode(&data[1..])
}

#[cfg(test)]
mod tests {
    use super::*;
    use gps_proto::packet::FLAG_FIX;

    #[test]
    fn position_roundtrip_with_mesh_padding() {
        let p = PositionPacket {
            lat_e7: 481_173_000,
            lon_e7: -1_226_760_000,
            alt_dm: 1234,
            speed_cms: 0,
            course_cdeg: 0,
            flags: FLAG_FIX,
            sats: 7,
            tod_ms: 1000,
        };
        let enc = encode_position(&p);
        assert_eq!(enc.len(), 21);
        assert!(is_binary(&enc));
        // Simulate nano-mesh padding to 32 bytes with zeros.
        let mut padded = [0u8; 32];
        padded[..enc.len()].copy_from_slice(&enc);
        assert_eq!(decode_position(&padded), Some(p));
        assert_eq!(decode_position(b"hello"), None);
        assert!(!is_binary(b"hello"));
    }
}
