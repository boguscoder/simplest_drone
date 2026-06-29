use crate::imu::ImuData;
use crate::pid::{self, Pid};
use crate::rc::RcData;
use crate::telemetry::Category;

const MAX_POWER: f32 = 0.6;
const THROTTLE_MIN: f32 = 48.0;
const THROTTLE_MAX: f32 = 2047.0;
const SLOPE: f32 = THROTTLE_MAX - THROTTLE_MIN;

const ROLL_RATE: f32 = 200.0 * core::f32::consts::PI / 180.0;
const PITCH_RATE: f32 = ROLL_RATE;
const YAW_RATE: f32 = 100.0 * core::f32::consts::PI / 180.0;

const ROLL_MIX_GAIN: f32 = 0.5;
const PITCH_MIX_GAIN: f32 = 0.5;
const YAW_MIX_GAIN: f32 = 0.4;

pub fn pid_to_throttle(rc: f32) -> u16 {
    (THROTTLE_MIN + SLOPE * rc) as u16
}

fn inputs_to_throttle(throttle: f32, pid_roll: f32, pid_pitch: f32, pid_yaw: f32) -> [u16; 4] {
    tele!(Category::Pid, throttle, pid_roll, pid_pitch, pid_yaw);

    let mixed_vals = [
        throttle - pid_pitch + pid_roll - pid_yaw,
        throttle + pid_pitch - pid_roll - pid_yaw,
        throttle - pid_pitch - pid_roll + pid_yaw,
        throttle + pid_pitch + pid_roll + pid_yaw,
    ];

    tele!(
        Category::Mix,
        mixed_vals[0],
        mixed_vals[1],
        mixed_vals[2],
        mixed_vals[3]
    );

    let throttle_vals = [
        pid_to_throttle(mixed_vals[0] * MAX_POWER),
        pid_to_throttle(mixed_vals[1] * MAX_POWER),
        pid_to_throttle(mixed_vals[2] * MAX_POWER),
        pid_to_throttle(mixed_vals[3] * MAX_POWER),
    ];

    tele!(
        Category::Dshot,
        throttle_vals[0],
        throttle_vals[1],
        throttle_vals[2],
        throttle_vals[3]
    );

    throttle_vals
}

pub struct MotorInput {
    pid_roll: Pid,
    pid_pitch: Pid,
    pid_yaw: Pid,
}

impl MotorInput {
    pub fn new(cycle_time: f32) -> MotorInput {
        let pid_limits = Some(pid::Limits {
            min: -1.0,
            max: 1.0,
        });
        let d_filter_cutoff_hz = Some(50.0);
        MotorInput {
            pid_roll: Pid::new(0.8, 0.0, 0.05, cycle_time, pid_limits, d_filter_cutoff_hz),
            pid_pitch: Pid::new(0.8, 0.0, 0.05, cycle_time, pid_limits, d_filter_cutoff_hz),
            pid_yaw: Pid::new(0.8, 0.0, 0.05, cycle_time, pid_limits, d_filter_cutoff_hz),
        }
    }

    pub fn update(&mut self, rc_data: &RcData, imu: &ImuData) -> [u16; 4] {
        self.pid_roll.set_kp(rc_data.gain());
        self.pid_pitch.set_kp(rc_data.gain());

        let pid_roll = self
            .pid_roll
            .update(rc_data.roll() * ROLL_RATE, -imu.att[0])
            * ROLL_MIX_GAIN;
        let pid_pitch = self
            .pid_pitch
            .update(rc_data.pitch() * PITCH_RATE, imu.att[1])
            * PITCH_MIX_GAIN;
        let pid_yaw = self.pid_yaw.update(rc_data.yaw() * YAW_RATE, imu.yaw_rate) * YAW_MIX_GAIN;

        inputs_to_throttle(rc_data.throttle(), pid_roll, pid_pitch, pid_yaw)
    }
}
