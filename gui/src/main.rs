//! Thermostat GUI. Talks to the controller (ESP32-C6 firmware) over a serial
//! port using the shared `thermo::comm` protocol, or runs against an
//! in-process simulator with `--sim` so the UI can be developed without
//! hardware.

slint::include_modules!();

mod link;

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::{Duration, Instant};

use slint::{SharedString, Timer, TimerMode};
use thermo::comm::Message;
use thermo::core::{OpState, SystemMode};

use link::Link;

fn mode_str(m: SystemMode) -> &'static str {
    match m {
        SystemMode::Auto => "auto",
        SystemMode::Heat => "heat",
        SystemMode::Cool => "cool",
        SystemMode::Off => "off",
    }
}

fn c_to_f(c: f32) -> f32 {
    c * 9.0 / 5.0 + 32.0
}

fn f_to_c(f: f32) -> f32 {
    (f - 32.0) * 5.0 / 9.0
}

fn op_str(o: OpState) -> &'static str {
    match o {
        OpState::Idle => "idle",
        OpState::Heat => "heat",
        OpState::Cool => "cool",
    }
}

fn fmt_uptime(secs: u64) -> SharedString {
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    if h > 0 {
        format!("UP {h}h {m:02}m").into()
    } else if m > 0 {
        format!("UP {m}m {s:02}s").into()
    } else {
        format!("UP {s}s").into()
    }
}

fn main() {
    let mut sim = false;
    let mut port = "/dev/ttyUSB0".to_string();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--sim" => sim = true,
            "--port" => port = args.next().unwrap_or_else(|| port.clone()),
            _ => {
                eprintln!("usage: thermo-gui [--sim] [--port /dev/ttyUSB0]");
                std::process::exit(2);
            }
        }
    }

    let transport: Rc<RefCell<Box<dyn Link>>> = if sim {
        Rc::new(RefCell::new(Box::new(link::MockLink::new())))
    } else {
        match link::SerialLink::open(&port) {
            Ok(l) => Rc::new(RefCell::new(Box::new(l))),
            Err(e) => {
                eprintln!("failed to open {port}: {e}");
                std::process::exit(1);
            }
        }
    };

    eprintln!("[gui] using {} link", transport.borrow().name());

    let ui = MainWindow::new().expect("failed to create window");
    ui.set_link_state(SharedString::from("searching"));
    ui.set_mode(SharedString::from("off"));
    ui.set_op_state(SharedString::from("idle"));
    ui.set_set_temp(70.0);
    ui.set_ambient(68.0);
    ui.set_uptime_text(SharedString::from("UP 0s"));

    let last_rx = Rc::new(Cell::new(None));

    // -- user interactions ---------------------------------------------------
    ui.on_temp_changed({
        let transport = Rc::clone(&transport);
        move |v: f32| transport.borrow_mut().send(&Message::SetTemp(f_to_c(v)))
    });

    ui.on_mode_selected({
        let transport = Rc::clone(&transport);
        let ui_weak = ui.as_weak();
        move |m: SharedString| {
            let msg = match m.as_str() {
                "auto" => Message::SystemMode(SystemMode::Auto),
                "heat" => Message::SystemMode(SystemMode::Heat),
                "cool" => Message::SystemMode(SystemMode::Cool),
                "off" => Message::SystemMode(SystemMode::Off),
                _ => return,
            };
            if let Some(ui) = ui_weak.upgrade() {
                ui.set_mode(SharedString::from(m.as_str()));
            }
            transport.borrow_mut().send(&msg);
        }
    });

    // -- poll: drain received messages, apply Status, track staleness --------
    let poll = Timer::default();
    poll.start(TimerMode::Repeated, Duration::from_millis(100), {
        let ui_weak = ui.as_weak();
        let transport = Rc::clone(&transport);
        let last_rx = Rc::clone(&last_rx);
        move || {
            let Some(ui) = ui_weak.upgrade() else { return };
            let mut msgs = Vec::new();
            transport.borrow_mut().poll(&mut msgs);

            let mut received = false;
            for msg in msgs {
                if let Message::Status(s) = msg {
                    received = true;
                    ui.set_link_state(SharedString::from("online"));
                    ui.set_mode(SharedString::from(mode_str(s.system_mode)));
                    ui.set_op_state(SharedString::from(op_str(s.op_state)));
                    ui.set_set_temp(c_to_f(s.set_temp));
                    ui.set_ambient(c_to_f(s.ambient));
                    ui.set_fan_on(s.fan);
                    ui.set_uptime_text(fmt_uptime(s.uptime));
                }
            }
            let now = Instant::now();
            if received {
                last_rx.set(Some(now));
            }
            let stale = match last_rx.get() {
                Some(t) => now.duration_since(t) > Duration::from_secs(3),
                None => true,
            };
            if stale {
                ui.set_link_state(SharedString::from("offline"));
            }
        }
    });

    // -- ping: keep the link warm and detect a live controller ---------------
    let ping = Timer::default();
    ping.start(TimerMode::Repeated, Duration::from_secs(2), {
        let transport = Rc::clone(&transport);
        move || transport.borrow_mut().send(&Message::Ping)
    });

    ui.run().expect("event loop error");
}