#[derive(Copy, Clone)]
pub struct Limits {
    pub min: f32,
    pub max: f32,
}

struct LowPassFilterState {
    alpha: f32,
    prev: f32,
}

pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub i: f32,
    error: f32,
    measured_rate: f32,
    cycle_time: f32,
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
                prev: 0.0,
            });
        Pid {
            kp,
            ki,
            kd,
            i: 0.0,
            error: 0.0,
            measured_rate: 0.0,
            cycle_time,
            limit_i: 0.5,
            limit_pid,
            d_lowpass_filter,
        }
    }

    pub fn update(&mut self, desired_rate: f32, measured_rate: f32) -> f32 {
        let error_rate = desired_rate - measured_rate;
        // P term
        let p = error_rate * self.kp;
        // I term
        let mut i = self.i + (error_rate * self.ki * self.cycle_time);
        i = i.clamp(-self.limit_i, self.limit_i);
        // D term
        let mut d = -self.kd * (measured_rate - self.measured_rate) / self.cycle_time;
        if let Some(low_pass) = &mut self.d_lowpass_filter {
            low_pass.prev = low_pass.prev + low_pass.alpha * (d - low_pass.prev);
            d = low_pass.prev;
        }

        // state store
        self.measured_rate = measured_rate;
        self.error = error_rate;
        self.i = i;

        let pid = p + i + d;

        if let Some(limits) = self.limit_pid {
            pid.clamp(limits.min, limits.max)
        } else {
            pid
        }
    }
}
