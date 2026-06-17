use crate::{attitude::Attitude, setup, telemetry::Category};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Ticker};
use nalgebra::Vector3;

const CALIBRATION_TICKS: usize = 2000;

pub const IMU_TICK: u64 = 1000;

pub static ATT_DATA: Watch<CriticalSectionRawMutex, [f32; 3], 1> = Watch::new();

#[embassy_executor::task]
pub async fn imu_task(mut imu: setup::ImuReader) -> ! {
    let mut loop_ticker = Ticker::every(Duration::from_hz(IMU_TICK));
    let mut calibration_ticks: usize = 0;
    let mut total_ticks: usize = 0;
    let mut gyr_bias: Vector3<f32> = Vector3::zeros();

    let att_sender = ATT_DATA.sender();
    let mut att_transformer = Attitude::new();

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
                imu.read_mag()
                    .await
                    .ok()
                    .map(Vector3::from)
                    .unwrap_or_else(Vector3::zeros)
            } else {
                Vector3::<f32>::zeros()
            };

            let corrected_gyr = Vector3::from(imudata.gyr) - gyr_bias;
            let acc = Vector3::from(imudata.acc);

            #[rustfmt::skip]
            tele!(Category::Imu,
                corrected_gyr[0], corrected_gyr[1], corrected_gyr[2],
                acc[0], acc[1], acc[2]);

            if let Some(att) = att_transformer.update(&corrected_gyr, &acc, &mag) {
                att_sender.send(att)
            }
            total_ticks += 1;
        }

        loop_ticker.next().await;
    }
}
