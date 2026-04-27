use crate::setup;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use icm20948_async::Data6Dof;
use nalgebra::Vector3;

const CALIBRATION_TICKS: usize = 2000;

pub const IMU_TICK: u64 = 1000;
pub static IMU_DATA: Watch<CriticalSectionRawMutex, Data6Dof<f32>, 1> = Watch::new();

#[embassy_executor::task]
pub async fn imu_task(mut imu: setup::ImuReader) -> ! {
    let mut loop_ticker = Ticker::every(Duration::from_hz(IMU_TICK));
    let imu_sender = IMU_DATA.sender();
    let mut ticks: usize = 0;
    let mut gyr_bias: Vector3<f32> = Vector3::default();
    let mut acc_bias: Vector3<f32> = Vector3::default();

    loop {
        let Ok(imudata) = imu.read_6dof().await else {
            log::error!("Failed to read IMU");
            continue;
        };

        if ticks == 0 {
            log::info!("Calibration...");
            ticks += 1;
        } else if ticks < CALIBRATION_TICKS {
            ticks += 1;
            gyr_bias += Vector3::from(imudata.gyr);
            acc_bias += Vector3::from(imudata.acc);
        } else if ticks == CALIBRATION_TICKS {
            gyr_bias /= CALIBRATION_TICKS as f32;
            acc_bias /= CALIBRATION_TICKS as f32;
            acc_bias.z -= 9.81;
            log::info!(
                "Calibrated after {} ticks, gyro bias {:?}, acc bias {:?}",
                ticks,
                gyr_bias,
                acc_bias
            );
            ticks += 1;
        } else {
            imu_sender.send(Data6Dof::<f32> {
                gyr: (Vector3::from(imudata.gyr) - gyr_bias).into(),
                acc: (Vector3::from(imudata.acc) - acc_bias).into(),
                ..imudata
            })
        }

        loop_ticker.next().await;
    }
}
