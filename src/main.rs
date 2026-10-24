use std::time::{SystemTime, UNIX_EPOCH};

use thermo::core::Thermostat;

fn main() {
    let mut therm = Thermostat::new();
    therm.set_temp(21.0);

    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
    if therm.update(now) {
        println!("transitioned to {:?}", therm.op_state());
    }
}