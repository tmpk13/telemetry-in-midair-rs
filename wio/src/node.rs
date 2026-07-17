//! Safe wrapper around [`embedded_nano_mesh::Node`].

use core::num::NonZeroU8;

use embedded_nano_mesh::{
    ExactAddressType, LifeTimeType, Node, NodeConfig, PacketDataBytes, SendError,
};

use crate::io::LoraIo;
use crate::radio::PacketRadio;
use embedded_io::Write as _;

/// A received mesh message.
#[derive(Debug)]
pub struct MeshMessage {
    /// Source node address (0 = unknown / broadcast origin).
    pub source: u8,
    /// Payload bytes.
    pub data: PacketDataBytes,
}

/// A mesh network node backed by `embedded-nano-mesh`.
pub struct MeshNode {
    node: Node,
}

impl MeshNode {
    /// Create a new mesh node.
    ///
    /// `address` must be 1-255 (0 is reserved for broadcast).
    /// `listen_period_ms` is how long the node listens before
    /// transmitting queued packets.
    ///
    /// # Panics
    ///
    /// Panics if `address` is 0.
    pub fn new(address: u8, listen_period_ms: u32) -> Self {
        let device_address: ExactAddressType =
            NonZeroU8::new(address).expect("mesh address must be 1-255");
        Self {
            node: Node::new(NodeConfig {
                device_address,
                listen_period: listen_period_ms,
            }),
        }
    }

    /// Drive the mesh protocol.
    ///
    /// Must be called frequently in the main loop. Handles receiving,
    /// forwarding, and transmitting queued packets.
    pub fn update<R: PacketRadio>(&mut self, io: &mut LoraIo<R>, current_time_ms: u32) {
        // NodeUpdateError only reports queue-full conditions, which are
        // non-fatal - just means some packets were dropped.
        let _ = self.node.update(io, current_time_ms);
        // embedded-nano-mesh only calls write() on the IO, never flush().
        // flush() is what triggers actual radio transmission, so we must
        // call it here after every update to drain the tx buffer.
        let _ = io.flush();
    }

    /// Queue a message for a specific destination node.
    pub fn send(&mut self, data: &[u8], dest: u8, lifetime: LifeTimeType) -> Result<(), SendError> {
        let dest_addr = NonZeroU8::new(dest).expect("destination address must be 1-255");
        let mut pkt_data = PacketDataBytes::new();
        pkt_data
            .extend_from_slice(data)
            .map_err(|_| SendError::SendingQueueIsFull)?;
        self.node.send_to_exact(pkt_data, dest_addr, lifetime, true)
    }

    /// Queue a broadcast message to all reachable nodes.
    pub fn broadcast(&mut self, data: &[u8], lifetime: LifeTimeType) -> Result<(), SendError> {
        let mut pkt_data = PacketDataBytes::new();
        pkt_data
            .extend_from_slice(data)
            .map_err(|_| SendError::SendingQueueIsFull)?;
        self.node.broadcast(pkt_data, lifetime)
    }

    /// Check for a received message addressed to this node.
    pub fn receive(&mut self) -> Option<MeshMessage> {
        self.node.receive().map(|mut pkt| {
            // Packet::new() pads data to full capacity with null bytes and
            // the library does not expose the real data_length field. Text
            // payloads are truncated at the first null; tagged binary
            // payloads (positions, legacy OTA) carry their own length and
            // must pass through untouched.
            if !midair_proto::lora::is_binary(&pkt.data)
                && let Some(end) = pkt.data.iter().position(|&b| b == 0) {
                    pkt.data.truncate(end);
                }
            MeshMessage {
                source: pkt.source_device_identifier,
                data: pkt.data,
            }
        })
    }
}
