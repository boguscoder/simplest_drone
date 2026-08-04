use crate::alt_hold::{ALT_HOLD_OFF_SIGNAL, ALT_HOLD_ON_SIGNAL};
use crate::consts::{
    ALT_HOLD_THROTTLE_MAX, ALT_HOLD_THROTTLE_MIN, ALT_KD_MIN, ALT_KI_FIXED, ALT_KP_MIN,
    ANGLE_P_GAIN, D_FILTER_CUTOFF_HZ, I_TERM_THROTTLE_LIMIT, KD_FIXED, KI_FIXED, KP_FIXED,
    MAX_LEAN_ANGLE, MAX_POWER, PID_LIMIT_MAX, PID_LIMIT_MIN, SLOPE, THROTTLE_MIN, YAW_KD_FIXED,
    YAW_KP_FIXED, YAW_RATE,
};
use crate::{
    imu::ImuData,
    pid::{self, Pid},
    rc::RcData,
};
use drone_consts::telemetry::Category;

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
}

impl MotorInput {
    pub fn new(cycle_time: f32) -> MotorInput {
        let pid_limits = Some(pid::Limits {
            min: PID_LIMIT_MIN,
            max: PID_LIMIT_MAX,
        });
        let d_filter_cutoff_hz = Some(D_FILTER_CUTOFF_HZ);

        MotorInput {
            pid_roll: Pid::new(
                KP_FIXED,
                KI_FIXED,
                KD_FIXED,
                cycle_time,
                pid_limits,
                d_filter_cutoff_hz,
            ),
            pid_pitch: Pid::new(
                KP_FIXED,
                KI_FIXED,
                KD_FIXED,
                cycle_time,
                pid_limits,
                d_filter_cutoff_hz,
            ),
            pid_yaw: Pid::new(
                YAW_KP_FIXED,
                KI_FIXED,
                YAW_KD_FIXED,
                cycle_time,
                pid_limits,
                d_filter_cutoff_hz,
            ),
            pid_alt: Pid::new(
                ALT_KP_MIN,
                ALT_KI_FIXED,
                ALT_KD_MIN,
                cycle_time,
                pid_limits,
                None,
            ),
            target_alt: 0.0,
            hover_throttle: 0.0,
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
        self.pid_alt.kp = rc_data.kp_gain();
        self.pid_alt.kd = rc_data.kd_gain();

        let allow_i_term = rc_data.throttle() > I_TERM_THROTTLE_LIMIT;

        if !allow_i_term || !is_armed {
            self.pid_roll.i = 0.0;
            self.pid_pitch.i = 0.0;
            self.pid_yaw.i = 0.0;

            if !alt_hold {
                self.pid_alt.i = 0.0;
            }
        }

        if ALT_HOLD_ON_SIGNAL.try_take().is_some() {
            self.target_alt = alt;
            self.hover_throttle = rc_data
                .throttle()
                .clamp(ALT_HOLD_THROTTLE_MIN, ALT_HOLD_THROTTLE_MAX);
            self.pid_alt.i = 0.0;
            log::info!(
                "AltHold locked: {:.2}m | Hover throttle: {:.2}",
                self.target_alt,
                self.hover_throttle
            );
        }

        if ALT_HOLD_OFF_SIGNAL.try_take().is_some() {
            self.pid_alt.i = 0.0;
        }

        let mut pid_alt = 0.0;
        let throttle = if alt_hold {
            let alt_error = self.target_alt - alt;
            pid_alt = self.pid_alt.update(alt_error, alt);
            (self.hover_throttle + pid_alt).clamp(0.0, MAX_POWER)
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
            self.pid_roll.i,
            self.pid_pitch.i,
            self.pid_yaw.i,
            self.pid_alt.i,
        );

        inputs_to_throttle(throttle, pid_roll, pid_pitch, pid_yaw, is_armed)
    }
}
