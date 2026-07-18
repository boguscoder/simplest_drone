use crate::{arming::DISARMED, attitude::Attitude, setup, telemetry::Category};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::Vector3;

const CALIBRATION_TICKS: usize = 2000;

pub const IMU_TICK: u64 = 1000;

const ACC_OFFSET: Vector3<f32> = Vector3::new(
    -0.100000, // X
    -0.246035, // Y
    0.152372,  // Z
);

const ACC_SCALE: Vector3<f32> = Vector3::new(
    0.993833, // X
    0.998219, // Y
    0.990074, // Z
);

#[derive(Clone)]
pub struct ImuData {
    pub att: [f32; 3],
    pub gyro_rates: Vector3<f32>,
}

pub static IMU_DATA: Watch<CriticalSectionRawMutex, ImuData, 1> = Watch::new();

#[embassy_executor::task]
pub async fn imu_task(mut imu: setup::ImuReader) -> ! {
    Timer::after_secs(3).await;

    let mut loop_ticker = Ticker::every(Duration::from_hz(IMU_TICK));
    let mut calibration_ticks: usize = 0;
    let mut total_ticks: usize = 0;
    let mut gyr_bias: Vector3<f32> = Vector3::zeros();

    let imu_sender = IMU_DATA.sender();
    let mut att_transformer = Attitude::new();
    let mut last_time = Instant::now();

    loop {
        let Ok(imudata) = imu.read_6dof().await else {
            log::error!("Failed to read IMU");
            continue;
        };

        let now = Instant::now();
        let elapsed = now.duration_since(last_time);
        last_time = now;
        let dt = elapsed.as_micros() as f32 / 1_000_000.0;

        if DISARMED.try_take().is_some() {
            log::info!("Calibration reset requested");
            calibration_ticks = 0;
            gyr_bias = Vector3::zeros();
            imu_sender.clear();
        }

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
            let corrected_acc = (Vector3::from(imudata.acc) - ACC_OFFSET).component_mul(&ACC_SCALE);

            #[rustfmt::skip]
            tele!(Category::Imu,
                corrected_gyr[0], corrected_gyr[1], corrected_gyr[2],
                corrected_acc[0], corrected_acc[1], corrected_acc[2]);

            if let Some(att) = att_transformer.update(&corrected_gyr, &corrected_acc, &mag, dt) {
                imu_sender.send(ImuData {
                    att,
                    gyro_rates: corrected_gyr,
                })
            }
            total_ticks += 1;
        }

        loop_ticker.next().await;
    }
}
