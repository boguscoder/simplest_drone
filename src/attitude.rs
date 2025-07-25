use crate::imu;
use crate::telemetry::Category;
use ahrs::{Ahrs, Madgwick};
use icm20948_async::Data6Dof;
use nalgebra::Vector3;

pub struct Attitude {
    ahrs: Madgwick<f32>,
}

impl Attitude {
    pub fn new() -> Attitude {
        Attitude {
            ahrs: Madgwick::new(1.0 / imu::IMU_TICK as f32, 0.05),
        }
    }

    pub fn update(&mut self, raw_imu: &Data6Dof<f32>) -> Option<[f32; 3]> {
        let gyr = Vector3::from(raw_imu.gyr);
        let acc = Vector3::from(raw_imu.acc);

        if let Ok(quat) = self.ahrs.update_imu(&gyr, &acc) {
            let att: [f32; 3] = quat.euler_angles().into();

            tele!(
                Category::Attitude,
                "{},{},{}",
                att[0].to_degrees(),
                att[1].to_degrees(),
                att[2].to_degrees()
            );

            return Some(att);
        } else {
            log::error!("ahrs error");
        }
        None
    }
}
