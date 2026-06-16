use crate::imu::{IMU_TICK, ImuType};
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

    pub fn update(&mut self, raw_imu: &ImuType) -> Option<[f32; 3]> {
        let gyr = Vector3::from(raw_imu.gyr);
        let acc = Vector3::from(raw_imu.acc);

        let update_result = match raw_imu.mag {
            Some(raw_mag) => self.ahrs.update(&gyr, &acc, &Vector3::from(raw_mag)),
            None => self.ahrs.update_imu(&gyr, &acc),
        };

        match update_result {
            Ok(quat) => {
                let att: [f32; 3] = quat.euler_angles().into();

                tele!(
                    Category::Attitude,
                    att[0].to_degrees(),
                    att[1].to_degrees(),
                    att[2].to_degrees()
                );

                Some(att)
            }
            Err(e) => {
                log::error!("ahrs error: {:?}", e);
                None
            }
        }
    }
}
