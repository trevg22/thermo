//! Inter-MCU link protocol between the thermostat control MCU and the UI MCU.
//!
//! Frames are binary: each message is [`serde`] serialized with
//! [`postcard`], then COBS encoded ([`postcard::to_slice_cobs`]) which
//! appends a `0x00` frame sentinel. The same `k` code runs on both peers, so
//! the Rust GUI and the firmware share the exact wire format:
//!
//! - UI -> control: [`Message::SetTemp`], [`Message::SystemMode`],
//!   [`Message::Ping`].
//! - control -> UI: [`Message::Status`] (`1 Hz` heartbeat, and immediately on
//!   every state transition), [`Message::Pong`], [`Message::Nack`].
//!
//! Receive side is incremental: feed bytes into a [`CobsAccumulator`] as they
//! arrive (partial frames across polls are buffered) and handle the
//! [`FeedResult`]. COBS is self-synchronizing, so after a corrupted or dropped
//! frame the receiver re-syncs at the next `0x00` boundary.

use postcard::accumulator::CobsAccumulator;
use serde::{Deserialize, Serialize};

use crate::core::{OpState, SystemMode};

/// Re-exported so peers only need to depend on `thermo` to build/send frames.
pub use postcard;

/// Size of an encoded frame. Used for both the transmit buffer and the receive
/// accumulator. A `Status` frame is roughly 16-18 bytes, with headroom for
/// future fields.
pub const COBS_BUF: usize = 64;

/// Incremental COBS receive decoder shared by both peers.
pub type CobsDecoder = CobsAccumulator<COBS_BUF>;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Status {
    pub system_mode: SystemMode,
    pub op_state: OpState,
    pub set_temp: f32,
    pub ambient: f32,
    pub fan: bool,
    pub uptime: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Message {
    /// UI -> control: liveness check.
    Ping,

    /// UI -> control: set the setpoint in degrees.
    SetTemp(f32),

    /// UI -> control: set the operating mode.
    SystemMode(SystemMode),

    /// control -> UI: periodic status (1 Hz heartbeat and on transitions).
    Status(Status),

    /// control -> UI: reply to `Ping`.
    Pong,

    /// control -> UI: a frame was received but failed to decode.
    Nack,
}

/// COBS-encodes `msg` into `buf`, returning the filled slice including the
/// trailing `0x00` frame sentinel that terminates it on the wire.
pub fn send_frame<'b>(
    buf: &'b mut [u8],
    msg: &Message,
) -> Result<&'b mut [u8], postcard::Error> {
    postcard::to_slice_cobs(msg, buf)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec;
    use std::vec::Vec;

    use super::*;
    use postcard::accumulator::FeedResult;

    fn feed_all<T: for<'de> serde::Deserialize<'de>>(
        acc: &mut CobsAccumulator<COBS_BUF>,
        input: &[u8],
    ) -> Vec<T> {
        let mut out = Vec::new();
        let mut window = input;
        while !window.is_empty() {
            match acc.feed::<T>(window) {
                FeedResult::Consumed => break,
                FeedResult::Success { data, remaining } => {
                    out.push(data);
                    window = remaining;
                }
                FeedResult::OverFull(w) | FeedResult::DeserError(w) => window = w,
            }
        }
        out
    }

    fn encode(msg: &Message) -> Vec<u8> {
        let mut buf = [0u8; COBS_BUF * 2];
        send_frame(&mut buf, msg).unwrap().to_vec()
    }

    fn sample_status() -> Status {
        Status {
            system_mode: SystemMode::Auto,
            op_state: OpState::Heat,
            set_temp: 21.0,
            ambient: 19.8,
            fan: true,
            uptime: 42,
        }
    }

    #[test]
    fn roundtrips_all_messages() {
        let messages = [
            Message::Ping,
            Message::SetTemp(20.5),
            Message::SystemMode(SystemMode::Cool),
            Message::Status(sample_status()),
            Message::Pong,
            Message::Nack,
        ];
        for msg in messages {
            let frame = encode(&msg);
            let mut acc: CobsAccumulator<COBS_BUF> = CobsAccumulator::new();
            let decoded = feed_all::<Message>(&mut acc, &frame);
            assert_eq!(decoded, vec![msg]);
        }
    }

    #[test]
    fn decodes_byte_dribbled_frame() {
        let frame = encode(&Message::Status(sample_status()));
        let mut acc: CobsAccumulator<COBS_BUF> = CobsAccumulator::new();
        let mut hits = 0;
        for &b in &frame {
            let payload = [b];
            match acc.feed::<Message>(&payload) {
                FeedResult::Consumed => {}
                FeedResult::Success { data, .. } => {
                    assert_eq!(data, Message::Status(sample_status()));
                    hits += 1;
                }
                _ => panic!("unexpected feed result"),
            }
        }
        assert_eq!(hits, 1);
    }

    #[test]
    fn decodes_back_to_back_frames() {
        let mut wire = encode(&Message::Ping);
        wire.extend(encode(&Message::Status(sample_status())));
        let mut acc: CobsAccumulator<COBS_BUF> = CobsAccumulator::new();
        assert_eq!(
            feed_all::<Message>(&mut acc, &wire),
            vec![Message::Ping, Message::Status(sample_status())]
        );
    }

    #[test]
    fn decodes_after_corrupted_frame() {
        let mut wire = vec![0xDE, 0xAD, 0x00];
        wire.extend(encode(&Message::SetTemp(21.5)));
        let mut acc: CobsAccumulator<COBS_BUF> = CobsAccumulator::new();
        assert_eq!(
            feed_all::<Message>(&mut acc, &wire),
            vec![Message::SetTemp(21.5)]
        );
    }

    #[test]
    fn survives_garbage_burst() {
        let mut wire = encode(&Message::Ping);
        wire.extend([0xAA; COBS_BUF * 3]);
        let mut acc: CobsAccumulator<COBS_BUF> = CobsAccumulator::new();
        assert_eq!(feed_all::<Message>(&mut acc, &wire), vec![Message::Ping]);
    }

    #[test]
    fn rejects_undersized_encode_buffer() {
        let mut buf = [0u8; 4];
        assert!(send_frame(&mut buf, &Message::Status(sample_status())).is_err());
    }
}