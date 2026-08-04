#[derive(Copy, Clone)]
pub struct Limits {
    pub min: f32,
    pub max: f32,
}

struct LowPassFilter {
    alpha: f32,
    prev1: f32,
    prev2: f32,
}

impl LowPassFilter {
    fn new(freq: f32, cycle_time: f32) -> LowPassFilter {
        LowPassFilter {
            alpha: {
                let rc_constant = 1.0 / (2.0 * core::f32::consts::PI * freq);
                cycle_time / (rc_constant + cycle_time)
            },
            prev1: 0.0,
            prev2: 0.0,
        }
    }
    fn filter(&mut self, input: f32) -> f32 {
        self.prev1 = self.prev1 + self.alpha * (input - self.prev1);
        self.prev2 = self.prev2 + self.alpha * (self.prev1 - self.prev2);
        self.prev2
    }
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
    rate_lp: Option<LowPassFilter>,
    d_lp: Option<LowPassFilter>,
}

impl Pid {
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        cycle_time: f32,
        limit_pid: Option<Limits>,
        rate_filter_cutoff_hz: Option<f32>,
        d_filter_cutoff_hz: Option<f32>,
    ) -> Pid {
        let rate_lp = rate_filter_cutoff_hz.map(|freq| LowPassFilter::new(freq, cycle_time));
        let d_lp: Option<LowPassFilter> =
            d_filter_cutoff_hz.map(|freq| LowPassFilter::new(freq, cycle_time));

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
            rate_lp,
            d_lp,
        }
    }

    pub fn update(&mut self, desired_rate: f32, mut measured_rate: f32) -> f32 {
        if let Some(filter) = &mut self.rate_lp {
            measured_rate = filter.filter(measured_rate);
        }

        let error_rate = desired_rate - measured_rate;
        // P term
        let p = error_rate * self.kp;
        // I term
        let mut i = self.i + (error_rate * self.ki * self.cycle_time);
        i = i.clamp(-self.limit_i, self.limit_i);
        // D term
        let mut d = -self.kd * (measured_rate - self.measured_rate) / self.cycle_time;

        if let Some(filter) = &mut self.d_lp {
            d = filter.filter(d);
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
