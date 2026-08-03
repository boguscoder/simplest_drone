use crate::{consts::CYCLE_TIME, imu::ImuData};
use nalgebra::UnitQuaternion;

const GRAVITY: f32 = 9.81;
const K_ALT: f32 = 0.02; // Pulls altitude toward baro
const K_VEL: f32 = 0.005; // Fixes velocity drift using baro error

pub struct AltitudeEstimator {
    estimated_alt: f32,
    velocity_z: f32,
}

impl AltitudeEstimator {
    pub fn new() -> AltitudeEstimator {
        AltitudeEstimator {
            estimated_alt: 0.0,
            velocity_z: 0.0,
        }
    }

    pub fn update(&mut self, quat: &UnitQuaternion<f32>, imu: &ImuData, baro_alt: f32) -> f32 {
        let w = quat[0];
        let x = quat[1];
        let y = quat[2];
        let z = quat[3];

        // Rotate the raw body acceleration into the earth frame's Z-axis
        let mut accel_z_earth = 2.0 * (x * z - w * y) * imu.acc[0]
            + 2.0 * (y * z + w * x) * imu.acc[1]
            + (w * w - x * x - y * y + z * z) * imu.acc[2];

        accel_z_earth -= GRAVITY;

        if accel_z_earth.abs() < 0.05 {
            accel_z_earth = 0.0;
        }

        // PREDICT (Fast, but drifts)
        self.velocity_z += accel_z_earth * CYCLE_TIME;
        self.estimated_alt += self.velocity_z * CYCLE_TIME;

        // CORRECT (Slow, anchors to reality)
        let alt_error = baro_alt - self.estimated_alt;

        self.estimated_alt += alt_error * K_ALT;
        self.velocity_z += alt_error * K_VEL;

        self.estimated_alt
    }
}
