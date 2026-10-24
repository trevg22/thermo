//! Transports for carrying [`Message`] frames to and from the thermostat
//! controller. The controller talks the [`thermo::comm`] postcard/COBS wire;
//! [`MockLink`] runs the real state machine in-process so the UI can be
//! developed and tested without hardware.

use std::error::Error;
use std::fmt;
use std::time::Duration;

use serialport::SerialPort;
use thermo::comm::{self, CobsDecoder, Message, Status};
use thermo::core::Thermostat;
use thermo::comm::postcard::accumulator::FeedResult;

#[derive(Debug)]
pub struct LinkError(String);

impl fmt::Display for LinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Error for LinkError {}

impl From<serialport::Error> for LinkError {
    fn from(e: serialport::Error) -> Self {
        LinkError(e.to_string())
    }
}

/// A thermostat transport. Polled from the UI thread; partial frames are
/// buffered inside each transport so callers just drain decoded messages.
pub trait Link {
    fn name(&self) -> &'static str;
    fn send(&mut self, msg: &Message);
    fn poll(&mut self, out: &mut Vec<Message>);
}

/// Real UART link over a host serial port (e.g. a USB-UART bridge to the
/// ESP32-C6 control board running the same [`thermo::comm`] protocol).
pub struct SerialLink {
    port: Box<dyn SerialPort>,
    decoder: CobsDecoder,
}

impl SerialLink {
    pub fn open(path: &str) -> Result<Self, LinkError> {
        let port = serialport::new(path, 115_200)
            .timeout(Duration::ZERO)
            .open()?;
        Ok(Self {
            port,
            decoder: CobsDecoder::new(),
        })
    }
}

impl Link for SerialLink {
    fn name(&self) -> &'static str {
        "serial"
    }

    fn send(&mut self, msg: &Message) {
        let mut buf = [0u8; comm::COBS_BUF * 2];
        match comm::send_frame(&mut buf, msg) {
            Ok(frame) => {
                let _ = self.port.write_all(frame);
                let _ = self.port.flush();
            }
            Err(_) => eprintln!("[link] frame did not fit"),
        }
    }

    fn poll(&mut self, out: &mut Vec<Message>) {
        let mut chunk = [0u8; 64];
        loop {
            match self.port.read(&mut chunk) {
                Ok(0) => break,
                Ok(n) => {
                    let mut window = &chunk[..n];
                    while !window.is_empty() {
                        match self.decoder.feed::<Message>(window) {
                            FeedResult::Consumed => break,
                            FeedResult::Success { data, remaining } => {
                                out.push(data);
                                window = remaining;
                            }
                            FeedResult::OverFull(w) | FeedResult::DeserError(w) => {
                                window = w;
                            }
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }
}

/// In-process thermostat that emulates the controller. Runs the real state
/// machine with an ambient temperature that drifts over time, mirroring the
/// firmware's simulated sensor.
pub struct MockLink {
    therm: Thermostat,
    ambient: f32,
    ticks: u64,
}

impl MockLink {
    pub fn new() -> Self {
        Self {
            therm: Thermostat::new(),
            ambient: 20.0,
            ticks: 0,
        }
    }

    fn status(&self) -> Status {
        Status {
            system_mode: self.therm.system_mode(),
            op_state: self.therm.op_state(),
            set_temp: self.therm.setpoint(),
            ambient: self.ambient,
            fan: self.therm.fan(),
            uptime: self.ticks / 10,
        }
    }
}

impl Link for MockLink {
    fn name(&self) -> &'static str {
        "mock"
    }

    fn send(&mut self, msg: &Message) {
        match msg {
            Message::SetTemp(v) => self.therm.set_temp(v.clamp(5.0, 35.0)),
            Message::SystemMode(m) => self.therm.set_system_mode(*m),
            Message::Ping => {}
            _ => {}
        }
    }

    fn poll(&mut self, out: &mut Vec<Message>) {
        self.ticks += 1;
        if self.ticks == 1 || self.ticks.is_multiple_of(20) {
            if self.ticks.is_multiple_of(20) {
                self.ambient =
                    20.0 + if (self.ticks / 20).is_multiple_of(2) { -8.0 } else { 6.0 };
            }
            self.therm.set_ambient(self.ambient);
            self.therm.update(self.ticks);
            out.push(Message::Status(self.status()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_echoes_commands() {
        let mut link = MockLink::new();
        link.send(&Message::SetTemp(24.5));
        link.send(&Message::SystemMode(thermo::core::SystemMode::Cool));

        let mut msgs = Vec::new();
        for _ in 0..20 {
            msgs.clear();
            link.poll(&mut msgs);
        }
        let status = match msgs.pop() {
            Some(Message::Status(s)) => s,
            other => panic!("expected status, got {other:?}"),
        };
        assert_eq!(status.set_temp, 24.5);
        assert_eq!(status.system_mode, thermo::core::SystemMode::Cool);
        assert!(status.fan);
    }

    #[test]
    fn mock_clamps_setpoint() {
        let mut link = MockLink::new();
        link.send(&Message::SetTemp(99.0));
        let mut msgs = Vec::new();
        for _ in 0..20 {
            link.poll(&mut msgs);
        }
        let status = match msgs.last().unwrap() {
            Message::Status(s) => s,
            _ => unreachable!(),
        };
        assert_eq!(status.set_temp, 35.0);
    }
}