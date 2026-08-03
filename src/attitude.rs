use crate::consts::{AHRS_BETA, CYCLE_TIME};
use ahrs::{Ahrs, Madgwick};
use nalgebra::{UnitQuaternion, Vector3};

pub struct Attitude {
    ahrs: Madgwick<f32>,
}

impl Attitude {
    pub fn new() -> Attitude {
        Attitude {
            ahrs: Madgwick::new(CYCLE_TIME, AHRS_BETA),
        }
    }

    pub fn update(
        &mut self,
        gyr: &Vector3<f32>,
        acc: &Vector3<f32>,
        mag: &Vector3<f32>,
        dt: f32,
    ) -> Option<UnitQuaternion<f32>> {
        *self.ahrs.sample_period_mut() = dt;
        let update_result = if mag != &Vector3::<f32>::zeros() && dt != 0.0 {
            self.ahrs.update(gyr, acc, mag)
        } else {
            self.ahrs.update_imu(gyr, acc)
        };

        match update_result {
            Ok(quat) => Some(*quat),
            Err(e) => {
                log::error!("ahrs error: {:?}", e);
                None
            }
        }
    }
}
