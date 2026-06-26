use crate::rc::RcData;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

/// Minimum number of ticks (1ms each) to hold arming stick position
const ARM_HOLD_TICKS: u64 = 1000;
/// Number of failsafe ticks before auto-disarming (0.5 seconds at 1kHz)
const FAILSAFE_DISARM_TICKS: u64 = 500;

pub static ARMED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[derive(Copy, Clone, PartialEq)]
pub enum ArmingState {
    Disarmed,
    Armed,
}

pub struct Arming {
    state: ArmingState,
    failsafe_ticks: u64,
    arm_request_ticks: u64,
    disarm_request_ticks: u64,
}

impl Arming {
    pub const fn new() -> Self {
        Self {
            state: ArmingState::Disarmed,
            failsafe_ticks: 0,
            arm_request_ticks: 0,
            disarm_request_ticks: 0,
        }
    }

    fn throttle_low(throttle: f32) -> bool {
        throttle < 0.1
    }

    fn arm_switch_high(rc_data: &RcData) -> bool {
        rc_data.arm_switch() > 0.5
    }

    fn arm_switch_low(rc_data: &RcData) -> bool {
        rc_data.arm_switch() < 0.5
    }

    pub fn update(&mut self, rc_data: &RcData, rc_valid: bool) -> ArmingState {
        if !rc_valid {
            if self.state == ArmingState::Armed {
                self.failsafe_ticks += 1;
                if self.failsafe_ticks >= FAILSAFE_DISARM_TICKS {
                    log::warn!("Auto-disarm: failsafe timeout");
                    self.state = ArmingState::Disarmed;
                }
            }
            self.arm_request_ticks = 0;
            self.disarm_request_ticks = 0;
            return self.state;
        }

        self.failsafe_ticks = 0;

        match self.state {
            ArmingState::Disarmed => self.try_arm(rc_data),
            ArmingState::Armed => self.try_disarm(rc_data),
        }
        self.state
    }

    fn try_arm(&mut self, rc_data: &RcData) {
        if Self::throttle_low(rc_data.throttle()) && Self::arm_switch_high(rc_data) {
            self.arm_request_ticks += 1;
            if self.arm_request_ticks >= ARM_HOLD_TICKS {
                log::info!("Armed (switch command)");
                self.state = ArmingState::Armed;
                self.arm_request_ticks = 0;
                ARMED.signal(());
            }
        } else {
            self.arm_request_ticks = 0;
        }
    }

    fn try_disarm(&mut self, rc_data: &RcData) {
        if Self::arm_switch_low(rc_data) {
            self.disarm_request_ticks += 1;
            if self.disarm_request_ticks >= ARM_HOLD_TICKS {
                log::info!("Disarmed (switch command)");
                self.state = ArmingState::Disarmed;
                self.disarm_request_ticks = 0;
            }
        } else {
            self.disarm_request_ticks = 0;
        }
    }

    pub fn state(&self) -> ArmingState {
        self.state
    }
}
