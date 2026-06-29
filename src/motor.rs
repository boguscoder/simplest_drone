use crate::imu::ImuData;
use crate::pid::{self, Pid};
use crate::rc::RcData;
use crate::telemetry::Category;

const MAX_POWER: f32 = 0.6;
const THROTTLE_MIN: f32 = 48.0;
const THROTTLE_MAX: f32 = 2047.0;
const SLOPE: f32 = THROTTLE_MAX - THROTTLE_MIN;

const YAW_RATE: f32 = 100.0 * core::f32::consts::PI / 180.0;

const MAX_LEAN_ANGLE: f32 = 45.0 * core::f32::consts::PI / 180.0;
const ANGLE_P_GAIN: f32 = 4.5;

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
            pid_roll: Pid::new(0.4, 0.0, 0.05, cycle_time, pid_limits, d_filter_cutoff_hz),
            pid_pitch: Pid::new(0.4, 0.0, 0.05, cycle_time, pid_limits, d_filter_cutoff_hz),
            pid_yaw: Pid::new(0.4, 0.0, 0.05, cycle_time, pid_limits, d_filter_cutoff_hz),
        }
    }

    pub fn update(&mut self, rc_data: &RcData, imu: &ImuData) -> [u16; 4] {
        self.pid_roll.set_kp(rc_data.gain());
        self.pid_pitch.set_kp(rc_data.gain());

        let allow_i_term = rc_data.throttle() > 0.1;

        if !allow_i_term {
            self.pid_roll.reset_i();
            self.pid_pitch.reset_i();
            self.pid_yaw.reset_i();
        }

        let target_angle_roll = rc_data.roll() * MAX_LEAN_ANGLE;
        let angle_error_roll = target_angle_roll - imu.att[0];
        let target_rate_roll = angle_error_roll * ANGLE_P_GAIN;
        let pid_roll = self.pid_roll.update(target_rate_roll, imu.gyro_rates[0]);

        let target_angle_pitch = rc_data.pitch() * MAX_LEAN_ANGLE;
        let angle_error_pitch = target_angle_pitch - imu.att[1];
        let target_rate_pitch = angle_error_pitch * ANGLE_P_GAIN;
        let pid_pitch = self.pid_pitch.update(target_rate_pitch, imu.gyro_rates[1]);

        let pid_yaw = self
            .pid_yaw
            .update(rc_data.yaw() * YAW_RATE, imu.gyro_rates[2]);

        inputs_to_throttle(rc_data.throttle(), pid_roll, pid_pitch, pid_yaw)
    }
}
