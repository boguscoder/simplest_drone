#[derive(Copy, Clone)]
pub struct Limits {
    pub min: f32,
    pub max: f32,
}

struct LowPassFilterState {
    alpha: f32,
    prev_d: f32,
}

pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    prev_error: f32,
    prev_measured: f32,
    cycle_time: f32,
    prev_i: f32,
    limit_i: f32,
    limit_pid: Option<Limits>,
    d_lowpass_filter: Option<LowPassFilterState>,
}

impl Pid {
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        cycle_time: f32,
        limit_pid: Option<Limits>,
        d_filter_cutoff_hz: Option<f32>,
    ) -> Pid {
        let d_lowpass_filter: Option<LowPassFilterState> =
            d_filter_cutoff_hz.map(|freq| LowPassFilterState {
                alpha: {
                    let rc_constant = 1.0 / (2.0 * core::f32::consts::PI * freq);
                    cycle_time / (rc_constant + cycle_time)
                },
                prev_d: 0.0,
            });
        Pid {
            kp,
            ki,
            kd,
            prev_error: 0.0,
            prev_measured: 0.0,
            cycle_time,
            prev_i: 0.0,
            limit_i: 10.0,
            limit_pid,
            d_lowpass_filter,
        }
    }

    pub fn update(&mut self, desired_rate: f32, measured_rate: f32) -> f32 {
        let error_rate = desired_rate - measured_rate;
        // P term
        let p = error_rate * self.kp;
        // I term
        let mut i = self.prev_i + (error_rate * self.ki * self.cycle_time);
        i = f32::max(f32::min(i, self.limit_i), -self.limit_i);
        // D term
        let mut d = -self.kd * (measured_rate - self.prev_measured) / self.cycle_time;
        if let Some(low_pass) = &mut self.d_lowpass_filter {
            low_pass.prev_d = low_pass.prev_d + low_pass.alpha * (d - low_pass.prev_d);
            d = low_pass.prev_d;
        }

        // state store
        self.prev_measured = measured_rate;
        self.prev_error = error_rate;
        self.prev_i = i;

        let mut pid = p + i + d;
        if let Some(limits) = self.limit_pid {
            pid = pid.clamp(limits.min, limits.max);
        }
        pid
    }
}
