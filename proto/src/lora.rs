//! The LoRa over-air frame and the payload formats it carries.
//!
//! Every transmission is a broadcast: one 3-byte header followed by an
//! application payload. There is no addressing beyond the originator and
//! no routing state - a node either repeats a frame or it does not, which
//! is all a leaf/repeater topology needs.
//!
//! ```text
//! [0] src        originating node address, 1-255
//! [1] id         originator's sequence number, wraps at 256
//! [2] hops_left  remaining retransmissions; 0 = nobody repeats this
//! [3..] payload  1..=PAYLOAD_MAX application bytes
//! ```
//!
//! `(src, id)` identifies a frame for as long as it is in flight, which is
//! what lets a receiver drop duplicates and a repeater avoid looping.
//!
//! There is no checksum here: the SX126x transmits LoRa packets with its
//! hardware CRC enabled and the driver discards frames that fail it, so a
//! software check would only repeat work already done in the radio.
//!
//! Payloads are sent at their true length - nothing is padded - so shrinking
//! a payload format shortens the air time it costs.

use gps_proto::packet::{PositionPacket, POSITION_PACKET_LEN};

/// Frame header length: `[src, id, hops_left]`.
pub const HEADER_LEN: usize = 3;

/// Largest application payload carried in one frame. Matches the payload
/// space the ESP link reserves for a forwarded frame.
pub const PAYLOAD_MAX: usize = 32;

/// Largest encoded frame.
pub const FRAME_MAX: usize = HEADER_LEN + PAYLOAD_MAX;

/// A decoded over-air frame borrowing its payload from the receive buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    /// Address of the node that originated the frame (never a repeater's).
    pub src: u8,
    /// Originator's sequence number.
    pub id: u8,
    /// Retransmissions still permitted. A repeater forwards only when this
    /// is non-zero, and decrements it on the way out.
    pub hops_left: u8,
    /// Application payload.
    pub payload: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Encoded length of this frame.
    pub fn encoded_len(&self) -> usize {
        HEADER_LEN + self.payload.len()
    }

    /// Write the frame into `out`, returning the number of bytes written.
    ///
    /// Returns `None` if the payload is empty, longer than [`PAYLOAD_MAX`],
    /// or `out` is too small.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let n = self.encoded_len();
        if self.payload.is_empty() || self.payload.len() > PAYLOAD_MAX || out.len() < n {
            return None;
        }
        out[0] = self.src;
        out[1] = self.id;
        out[2] = self.hops_left;
        out[HEADER_LEN..n].copy_from_slice(self.payload);
        Some(n)
    }

    /// Decode a received packet.
    ///
    /// Returns `None` for a runt, an empty payload, or a source address of
    /// 0 - the last of which is not a legal address and so marks the packet
    /// as something other than one of ours.
    pub fn decode(bytes: &'a [u8]) -> Option<Frame<'a>> {
        if bytes.len() <= HEADER_LEN || bytes[0] == 0 {
            return None;
        }
        Some(Frame {
            src: bytes[0],
            id: bytes[1],
            hops_left: bytes[2],
            payload: &bytes[HEADER_LEN..],
        })
    }
}

/// Position broadcast: `[POSITION tag] [PositionPacket 20B]`.
pub const MSG_POSITION: u8 = 0x50;

/// Encoded position message length.
pub const POSITION_MSG_LEN: usize = 1 + POSITION_PACKET_LEN;

/// Encode a position broadcast.
pub fn encode_position(p: &PositionPacket) -> [u8; POSITION_MSG_LEN] {
    let mut b = [0u8; POSITION_MSG_LEN];
    b[0] = MSG_POSITION;
    b[1..].copy_from_slice(&p.encode());
    b
}

/// Decode a position broadcast, or `None` if the payload is something else.
pub fn decode_position(data: &[u8]) -> Option<PositionPacket> {
    if data.first() != Some(&MSG_POSITION) || data.len() < POSITION_MSG_LEN {
        return None;
    }
    PositionPacket::decode(&data[1..POSITION_MSG_LEN])
}

#[cfg(test)]
mod tests {
    use super::*;
    use gps_proto::packet::FLAG_FIX;

    fn sample() -> PositionPacket {
        PositionPacket {
            lat_e7: 481_173_000,
            lon_e7: -1_226_760_000,
            alt_dm: 1234,
            speed_cms: 0,
            course_cdeg: 0,
            flags: FLAG_FIX,
            sats: 7,
            tod_ms: 1000,
        }
    }

    #[test]
    fn position_roundtrip() {
        let p = sample();
        let enc = encode_position(&p);
        assert_eq!(enc.len(), 21);
        assert_eq!(decode_position(&enc), Some(p));
        assert_eq!(decode_position(b"hello"), None);
        // A tagged but truncated payload is rejected rather than decoded
        // from whatever follows it in the buffer.
        assert_eq!(decode_position(&enc[..10]), None);
    }

    #[test]
    fn frame_roundtrip() {
        let payload = encode_position(&sample());
        let frame = Frame {
            src: 3,
            id: 42,
            hops_left: 1,
            payload: &payload,
        };
        let mut buf = [0u8; FRAME_MAX];
        let n = frame.encode(&mut buf).unwrap();
        // 3 header + 21 payload; no padding to a fixed size.
        assert_eq!(n, 24);
        assert_eq!(Frame::decode(&buf[..n]), Some(frame));
    }

    #[test]
    fn frame_rejects_malformed() {
        let mut buf = [0u8; FRAME_MAX];
        // Header only, no payload.
        assert_eq!(Frame::decode(&[1, 2, 3]), None);
        assert_eq!(Frame::decode(&[1, 2]), None);
        assert_eq!(Frame::decode(&[]), None);
        // Source address 0 is not assignable.
        assert_eq!(Frame::decode(&[0, 1, 1, 0x50]), None);
        // Empty and oversized payloads do not encode.
        let empty = Frame { src: 1, id: 0, hops_left: 0, payload: &[] };
        assert_eq!(empty.encode(&mut buf), None);
        let big = [0u8; PAYLOAD_MAX + 1];
        let over = Frame { src: 1, id: 0, hops_left: 0, payload: &big };
        assert_eq!(over.encode(&mut buf), None);
        // Exactly full fits.
        let full = Frame { src: 1, id: 0, hops_left: 0, payload: &big[..PAYLOAD_MAX] };
        assert_eq!(full.encode(&mut buf), Some(FRAME_MAX));
    }

    #[test]
    fn hops_survive_a_repeat() {
        let payload = encode_position(&sample());
        let mut buf = [0u8; FRAME_MAX];
        let n = Frame { src: 7, id: 9, hops_left: 2, payload: &payload }
            .encode(&mut buf)
            .unwrap();

        // What a repeater does: decode, decrement, re-encode. The origin
        // address and id must survive so the next hop still dedups on them.
        let recv = Frame::decode(&buf[..n]).unwrap();
        let mut out = [0u8; FRAME_MAX];
        let n2 = Frame { hops_left: recv.hops_left - 1, ..recv }
            .encode(&mut out)
            .unwrap();
        let hop2 = Frame::decode(&out[..n2]).unwrap();
        assert_eq!((hop2.src, hop2.id, hop2.hops_left), (7, 9, 1));
        assert_eq!(decode_position(hop2.payload), Some(sample()));
    }
}
