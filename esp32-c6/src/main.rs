#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    time::Instant,
    uart::{Config, Uart},
};
use esp_println::println;
use thermo::comm::{self, postcard::accumulator::FeedResult, CobsDecoder, Message, Status};
use thermo::core::{Thermostat, OpState};

fn ambient(idle: u64) -> f32 {
    21.0 + (idle % 20) as f32 - 10.0
}

fn apply_outputs(
    heat: &mut Output<'_>,
    cool: &mut Output<'_>,
    fan: &mut Output<'_>,
    therm: &Thermostat,
) {
    heat.set_level(Level::Low);
    cool.set_level(Level::Low);
    if therm.fan() {
        fan.set_level(Level::High);
    } else {
        fan.set_level(Level::Low);
    }
    match therm.op_state() {
        OpState::Heat => heat.set_level(Level::High),
        OpState::Cool => cool.set_level(Level::High),
        OpState::Idle => {}
    }
}

fn send_frame(uart: &mut Uart<Blocking>, msg: &Message) {
    let mut buf = [0u8; comm::COBS_BUF];
    match comm::send_frame(&mut buf, msg) {
        Ok(frame) => {
            let _ = uart.write(frame);
            let _ = uart.flush();
        }
        Err(_) => println!("frame did not fit"),
    }
}

fn handle_message(uart: &mut Uart<Blocking>, therm: &mut Thermostat, msg: Message) {
    match msg {
        Message::Ping => send_frame(uart, &Message::Pong),
        Message::SetTemp(value) => therm.set_temp(value.clamp(5.0, 35.0)),
        Message::SystemMode(mode) => therm.set_system_mode(mode),
        Message::Status(_) | Message::Pong | Message::Nack => {}
    }
}

fn process_rx(uart: &mut Uart<Blocking>, therm: &mut Thermostat) {
    let mut cobs = CobsDecoder::new();
    let mut chunk = [0u8; 16];

    while uart.read_ready() {
        if let Ok(n) = uart.read(&mut chunk) {
            let mut window = &chunk[..n];
            while !window.is_empty() {
                match cobs.feed::<Message>(window) {
                    FeedResult::Consumed => break,
                    FeedResult::Success { data, remaining } => {
                        handle_message(uart, therm, data);
                        window = remaining;
                    }
                    FeedResult::OverFull(w) | FeedResult::DeserError(w) => {
                        send_frame(uart, &Message::Nack);
                        window = w;
                    }
                }
            }
        }
    }
}

fn send_status(
    uart: &mut Uart<Blocking>,
    therm: &Thermostat,
    ambient: f32,
    uptime: u64,
    log: bool,
) {
    let status = Status {
        system_mode: therm.system_mode(),
        op_state: therm.op_state(),
        set_temp: therm.setpoint(),
        ambient,
        fan: therm.fan(),
        uptime,
    };
    if log {
        println!("{status:?}");
    }
    send_frame(uart, &Message::Status(status));
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    let mut uart = Uart::new(peripherals.UART1, Config::default())
        .unwrap()
        .with_tx(peripherals.GPIO18)
        .with_rx(peripherals.GPIO19);

    let mut heat = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let mut cool = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let mut fan = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());

    let mut therm = Thermostat::new();

    println!("thermo-esp32-c6 started");

    loop {
        process_rx(&mut uart, &mut therm);

        let uptime = Instant::now().duration_since_epoch().as_secs();
        let ambient = ambient(uptime);
        therm.set_ambient(ambient);

        let changed = therm.update(uptime);

        apply_outputs(&mut heat, &mut cool, &mut fan, &therm);

        send_status(&mut uart, &therm, ambient, uptime, changed || uptime == 0);

        delay.delay_millis(1_000);
    }
}