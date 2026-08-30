use signal_filters::{Pt2Filterf32, SignalFilter};

#[derive(Copy, Clone)]
pub struct Limits {
    pub min: f32,
    pub max: f32,
}

type LowPassFilter = Pt2Filterf32;

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
        let rate_lp = rate_filter_cutoff_hz.map(|freq| {
            let mut lp = LowPassFilter::new();
            lp.set_cutoff_frequency(freq, cycle_time);
            lp
        });
        let d_lp: Option<LowPassFilter> = d_filter_cutoff_hz.map(|freq| {
            let mut lp = LowPassFilter::new();
            lp.set_cutoff_frequency(freq, cycle_time);
            lp
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
            rate_lp,
            d_lp,
        }
    }

    pub fn update(&mut self, desired_rate: f32, mut measured_rate: f32) -> f32 {
        if let Some(filter) = &mut self.rate_lp {
            measured_rate = filter.update(measured_rate);
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
            d = filter.update(d);
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
