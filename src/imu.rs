use crate::setup;
use crate::telemetry::Category;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use nalgebra::Vector3;

const CALIBRATION_TICKS: usize = 2000;

pub const IMU_TICK: u64 = 1000;
#[derive(Clone)]
pub struct ImuType {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub mag: Option<Vector3<f32>>,
}

pub static IMU_DATA: Watch<CriticalSectionRawMutex, ImuType, 1> = Watch::new();

#[embassy_executor::task]
pub async fn imu_task(mut imu: setup::ImuReader) -> ! {
    let mut loop_ticker = Ticker::every(Duration::from_hz(IMU_TICK));
    let imu_sender = IMU_DATA.sender();
    let mut calibration_ticks: usize = 0;
    let mut total_ticks: usize = 0;
    let mut gyr_bias: Vector3<f32> = Vector3::default();

    loop {
        let Ok(imudata) = imu.read_6dof().await else {
            log::error!("Failed to read IMU");
            continue;
        };

        if calibration_ticks == 0 {
            log::info!("Calibration...");
            calibration_ticks += 1;
        } else if calibration_ticks < CALIBRATION_TICKS {
            gyr_bias += Vector3::from(imudata.gyr);
            calibration_ticks += 1;
        } else if calibration_ticks == CALIBRATION_TICKS {
            gyr_bias /= CALIBRATION_TICKS as f32;
            log::info!(
                "Calibrated after {} ticks, gyro bias {:?}",
                calibration_ticks,
                gyr_bias,
            );
            calibration_ticks += 1;
        } else {
            let mag = if total_ticks.is_multiple_of(10) {
                imu.read_mag().await.ok().map(Vector3::from)
            } else {
                None
            };

            let imu_fixed = ImuType {
                gyr: Vector3::from(imudata.gyr) - gyr_bias,
                acc: Vector3::from(imudata.acc),
                mag,
            };

            #[rustfmt::skip]
            tele!(Category::Imu,
                imu_fixed.gyr[0], imu_fixed.gyr[1], imu_fixed.gyr[2],
                imu_fixed.acc[0], imu_fixed.acc[1], imu_fixed.acc[2]);

            imu_sender.send(imu_fixed);
        }

        total_ticks += 1;
        loop_ticker.next().await;
    }
}
