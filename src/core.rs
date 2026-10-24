const COMPRESSOR_MIN_ON: u64 = 3 * 60;
const COMPRESSOR_MIN_OFF: u64 = 5 * 60;

const HEAT_MIN_ON: u64 = 3 * 60;
const HEAT_MIN_OFF: u64 = 2 * 60;
const TEMP_SET_MARGIN: f32 = 1.0;

/// Default setpoint in degrees Celsius applied on a fresh controller.
const DEFAULT_SET_TEMP: f32 = 21.0;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SystemMode {
    #[default]
    Auto,
    Off,
    Heat,
    Cool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OpState {
    Idle,
    Heat,
    Cool,
}

pub struct Thermostat {
    outs: Outputs,
    ins: Inputs,
    set_temp: f32,
    system_mode: SystemMode,
    last_transition: Option<u64>,
}

pub struct Outputs {
    mode: OpState,
    fan: bool,
}

pub struct Inputs {
    temp: f32,
}

impl Thermostat {
    pub fn new() -> Self {
        let outputs = Outputs {
            mode: OpState::Idle,
            fan: false,
        };
        let inputs = Inputs { temp: 0.0 };
        Self {
            outs: outputs,
            ins: inputs,
            set_temp: DEFAULT_SET_TEMP,
            system_mode: SystemMode::Auto,
            last_transition: None,
        }
    }
}

impl Default for Thermostat {
    fn default() -> Self {
        Self::new()
    }
}

impl Thermostat {
    pub fn set_temp(&mut self, t: f32) {
        self.set_temp = t;
    }

    pub fn setpoint(&self) -> f32 {
        self.set_temp
    }

    pub fn set_ambient(&mut self, t: f32) {
        self.ins.temp = t;
    }

    pub fn set_system_mode(&mut self, mode: SystemMode) {
        self.system_mode = mode;
    }

    pub fn system_mode(&self) -> SystemMode {
        self.system_mode
    }

    pub fn op_state(&self) -> OpState {
        self.outs.mode
    }

    pub fn fan(&self) -> bool {
        self.outs.fan
    }

    fn min_on(mode: OpState) -> u64 {
        match mode {
            OpState::Cool => COMPRESSOR_MIN_ON,
            OpState::Heat => HEAT_MIN_ON,
            OpState::Idle => 0,
        }
    }

    fn min_off(mode: OpState) -> u64 {
        match mode {
            OpState::Cool => COMPRESSOR_MIN_OFF,
            OpState::Heat => HEAT_MIN_OFF,
            OpState::Idle => 0,
        }
    }

    fn min_run_elapsed(&self, now: u64) -> bool {
        match self.last_transition {
            Some(t) => now.saturating_sub(t) >= Self::min_on(self.outs.mode),
            None => true,
        }
    }

    fn can_start(&self, mode: OpState, now: u64) -> bool {
        match self.last_transition {
            Some(t) => now.saturating_sub(t) >= Self::min_off(mode),
            None => true,
        }
    }

    fn demand(&self) -> OpState {
        match self.system_mode {
            SystemMode::Off => OpState::Idle,
            SystemMode::Heat => {
                if self.set_temp - self.ins.temp > TEMP_SET_MARGIN {
                    OpState::Heat
                } else {
                    OpState::Idle
                }
            }
            SystemMode::Cool => {
                if self.ins.temp - self.set_temp > TEMP_SET_MARGIN {
                    OpState::Cool
                } else {
                    OpState::Idle
                }
            }
            SystemMode::Auto => {
                let diff = self.set_temp - self.ins.temp;

                if diff.abs() <= TEMP_SET_MARGIN {
                    OpState::Idle
                } else if diff.is_sign_positive() {
                    OpState::Heat
                } else {
                    OpState::Cool
                }
            }
        }
    }

    pub fn update(&mut self, now: u64) -> bool {
        let changed = match self.outs.mode {
            OpState::Idle => {
                let next = self.demand();

                if next != OpState::Idle && self.can_start(next, now) {
                    self.outs.mode = next;
                    self.last_transition = Some(now);
                    true
                } else {
                    false
                }
            }

            OpState::Heat => {
                if self.demand() != OpState::Heat && self.min_run_elapsed(now) {
                    self.outs.mode = OpState::Idle;
                    self.last_transition = Some(now);
                    true
                } else {
                    false
                }
            }

            OpState::Cool => {
                if self.demand() != OpState::Cool && self.min_run_elapsed(now) {
                    self.outs.mode = OpState::Idle;
                    self.last_transition = Some(now);
                    true
                } else {
                    false
                }
            }
        };

        self.outs.fan = self.outs.mode != OpState::Idle;
        changed
    }
}