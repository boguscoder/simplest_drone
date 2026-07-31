use crate::alt_hold::{ALT_HOLD_OFF_SIGNAL, ALT_HOLD_ON_SIGNAL};
use crate::consts::{
    ALT_HOLD_SMOOTHING_TICKS, ALT_HOLD_THROTTLE_MAX, ALT_HOLD_THROTTLE_MIN, ALT_KD_MIN, ALT_KI_MIN,
    ALT_KP_MIN, ALT_PID_LIMIT_MAX, ALT_PID_LIMIT_MIN, ANGLE_P_GAIN, D_FILTER_CUTOFF_HZ,
    I_TERM_THROTTLE_LIMIT, KD_MIN, KI_MIN, KP_MIN, MAX_LEAN_ANGLE, MAX_POWER, PID_LIMIT_MAX,
    PID_LIMIT_MIN, PID_YAW_KP, SLOPE, THROTTLE_MIN, YAW_RATE,
};
use crate::imu::ImuData;
use crate::pid::{self, Pid};
use crate::rc::RcData;
use crate::telemetry::Category;

pub fn pid_to_throttle(rc: f32) -> u16 {
    let clamped_rc = rc.clamp(0.0, MAX_POWER);
    (THROTTLE_MIN + SLOPE * clamped_rc) as u16
}

fn inputs_to_throttle(
    throttle: f32,
    pid_roll: f32,
    pid_pitch: f32,
    pid_yaw: f32,
    is_armed: bool,
) -> [u16; 4] {
    let mixed_vals = [
        throttle - pid_pitch - pid_roll + pid_yaw,
        throttle + pid_pitch + pid_roll + pid_yaw,
        throttle - pid_pitch + pid_roll - pid_yaw,
        throttle + pid_pitch - pid_roll - pid_yaw,
    ];

    tele!(
        Category::Mix,
        mixed_vals[0],
        mixed_vals[1],
        mixed_vals[2],
        mixed_vals[3]
    );

    let throttle_vals = if is_armed {
        [
            pid_to_throttle(mixed_vals[0]),
            pid_to_throttle(mixed_vals[1]),
            pid_to_throttle(mixed_vals[2]),
            pid_to_throttle(mixed_vals[3]),
        ]
    } else {
        [0u16; 4]
    };

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
    pid_alt: Pid,
    target_alt: f32,
    hover_throttle: f32,
    alt_hold_entry_ticks: u16,
}

impl MotorInput {
    pub fn new(cycle_time: f32) -> MotorInput {
        let pid_limits = Some(pid::Limits {
            min: PID_LIMIT_MIN,
            max: PID_LIMIT_MAX,
        });
        let alt_pid_limits = Some(pid::Limits {
            min: ALT_PID_LIMIT_MIN,
            max: ALT_PID_LIMIT_MAX,
        });
        let d_filter_cutoff_hz = Some(D_FILTER_CUTOFF_HZ);

        MotorInput {
            pid_roll: Pid::new(
                KP_MIN,
                KI_MIN,
                KD_MIN,
                cycle_time,
                pid_limits,
                d_filter_cutoff_hz,
            ),
            pid_pitch: Pid::new(
                KP_MIN,
                KI_MIN,
                KD_MIN,
                cycle_time,
                pid_limits,
                d_filter_cutoff_hz,
            ),
            pid_yaw: Pid::new(
                PID_YAW_KP,
                KI_MIN,
                KD_MIN,
                cycle_time,
                pid_limits,
                d_filter_cutoff_hz,
            ),
            pid_alt: Pid::new(
                ALT_KP_MIN,
                ALT_KI_MIN,
                ALT_KD_MIN,
                cycle_time,
                alt_pid_limits,
                None,
            ),
            target_alt: 0.0,
            hover_throttle: 0.0,
            alt_hold_entry_ticks: 0,
        }
    }

    pub fn update(
        &mut self,
        rc_data: &RcData,
        imu: &ImuData,
        att: &[f32; 3],
        alt: f32,
        is_armed: bool,
        alt_hold: bool,
    ) -> [u16; 4] {
        let kp = rc_data.kp_gain();
        self.pid_roll.kp = kp;
        self.pid_pitch.kp = kp;

        let ki = rc_data.ki_gain();
        self.pid_roll.ki = ki;
        self.pid_pitch.ki = ki;

        let allow_i_term = rc_data.throttle() > I_TERM_THROTTLE_LIMIT;

        if !allow_i_term || !is_armed {
            self.pid_roll.prev_i = 0.0;
            self.pid_pitch.prev_i = 0.0;
            self.pid_yaw.prev_i = 0.0;

            if !alt_hold {
                self.pid_alt.prev_i = 0.0;
            }
        }

        if ALT_HOLD_ON_SIGNAL.try_take().is_some() {
            self.target_alt = alt;
            self.hover_throttle = rc_data
                .throttle()
                .clamp(ALT_HOLD_THROTTLE_MIN, ALT_HOLD_THROTTLE_MAX);
            self.pid_alt.prev_i = 0.0;
            self.alt_hold_entry_ticks = ALT_HOLD_SMOOTHING_TICKS;
            log::info!(
                "AltHold locked: {:.2}m | Hover throttle: {:.2}",
                self.target_alt,
                self.hover_throttle
            );
        }

        if ALT_HOLD_OFF_SIGNAL.try_take().is_some() {
            self.pid_alt.prev_i = 0.0;
        }

        let mut pid_alt = 0.0;
        let throttle = if alt_hold {
            let alt_error = self.target_alt - alt;
            pid_alt = self.pid_alt.update(alt_error, alt);

            let alt_correction = if self.alt_hold_entry_ticks > 0 {
                self.alt_hold_entry_ticks -= 1;
                let scale =
                    1.0 - (self.alt_hold_entry_ticks as f32 / ALT_HOLD_SMOOTHING_TICKS as f32);
                pid_alt * scale
            } else {
                pid_alt
            };
            pid_alt = alt_correction;

            (self.hover_throttle + alt_correction).clamp(0.0, MAX_POWER)
        } else {
            rc_data.throttle()
        };

        let target_angle_roll = -rc_data.roll() * MAX_LEAN_ANGLE;
        let angle_error_roll = target_angle_roll - att[0];
        let target_rate_roll = angle_error_roll * ANGLE_P_GAIN;
        let pid_roll = self.pid_roll.update(target_rate_roll, imu.gyro[0]);

        let target_angle_pitch = rc_data.pitch() * MAX_LEAN_ANGLE;
        let angle_error_pitch = target_angle_pitch - att[1];
        let target_rate_pitch = angle_error_pitch * ANGLE_P_GAIN;
        let pid_pitch = self.pid_pitch.update(target_rate_pitch, imu.gyro[1]);

        let pid_yaw = self.pid_yaw.update(rc_data.yaw() * YAW_RATE, -imu.gyro[2]);

        tele!(
            Category::Pid,
            pid_roll,
            pid_pitch,
            pid_yaw,
            pid_alt,
            self.pid_roll.prev_i,
            self.pid_pitch.prev_i,
            self.pid_yaw.prev_i,
            self.pid_alt.prev_i,
        );

        inputs_to_throttle(throttle, pid_roll, pid_pitch, pid_yaw, is_armed)
    }
}
