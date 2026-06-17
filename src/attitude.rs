use crate::imu::IMU_TICK;
use crate::telemetry::Category;
use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;

pub struct Attitude {
    ahrs: Madgwick<f32>,
}

impl Attitude {
    pub fn new() -> Attitude {
        Attitude {
            ahrs: Madgwick::new(1.0 / IMU_TICK as f32, 0.05),
        }
    }

    pub fn update(
        &mut self,
        gyr: &Vector3<f32>,
        acc: &Vector3<f32>,
        mag: &Vector3<f32>,
    ) -> Option<[f32; 3]> {
        let update_result = if mag != &Vector3::<f32>::zeros() {
            self.ahrs.update(gyr, acc, mag)
        } else {
            self.ahrs.update_imu(gyr, acc)
        };

        match update_result {
            Ok(quat) => {
                let att: [f32; 3] = quat.euler_angles().into();

                tele!(Category::Attitude, att[0], att[1], att[2]);

                Some(att)
            }
            Err(e) => {
                log::error!("ahrs error: {:?}", e);
                None
            }
        }
    }
}
