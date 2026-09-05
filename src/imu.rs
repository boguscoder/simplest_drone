use crate::consts::*;
use crate::setup;
use drone_consts::telemetry::*;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::Vector3;
use signal_filters::{Pt2Filterf32, SignalFilter};

#[derive(Clone)]
pub struct ImuData {
    pub gyro: Vector3<f32>,
    pub acc: Vector3<f32>,
    pub mag: Vector3<f32>,
    pub dt: f32,
}

pub static IMU_DATA: Watch<CriticalSectionRawMutex, ImuData, 1> = Watch::new();

#[embassy_executor::task]
pub async fn imu_task(mut imu: setup::ImuReader) -> ! {
    Timer::after_secs(3).await;

    let mut loop_ticker = Ticker::every(Duration::from_hz(TICK_HZ));
    let mut calibration_ticks: usize = 0;
    let mut total_ticks: usize = 0;
    let mut gyr_bias: Vector3<f32> = Vector3::zeros();
    let mut gyro_fx = Pt2Filterf32::new();
    let mut gyro_fy = Pt2Filterf32::new();
    let mut gyro_fz = Pt2Filterf32::new();
    gyro_fx.set_cutoff_frequency(RATE_FILTER_CUTOFF_HZ, CYCLE_TIME);
    gyro_fy.set_cutoff_frequency(RATE_FILTER_CUTOFF_HZ, CYCLE_TIME);
    gyro_fz.set_cutoff_frequency(RATE_FILTER_CUTOFF_HZ, CYCLE_TIME);

    let imu_sender = IMU_DATA.sender();
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

        let gyr = Vector3::new(imudata.gyr[0], -imudata.gyr[1], -imudata.gyr[2]);
        let acc = Vector3::new(imudata.acc[0], -imudata.acc[1], -imudata.acc[2]);

        if calibration_ticks == 0 {
            log::info!("Calibration...");
            calibration_ticks += 1;
        } else if calibration_ticks < CALIBRATION_TICKS {
            gyr_bias += gyr;
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

            let corrected_gyr = gyr - gyr_bias;
            let corrected_acc = (acc - ACC_OFFSET).component_mul(&ACC_SCALE);
            let filtered_gyr = Vector3::new(
                gyro_fx.update(corrected_gyr[0]),
                gyro_fy.update(corrected_gyr[1]),
                gyro_fz.update(corrected_gyr[2]),
            );

            #[rustfmt::skip]
            tele!(Mode::Imu,
                filtered_gyr[0], filtered_gyr[1], filtered_gyr[2],
                corrected_acc[0], corrected_acc[1], corrected_acc[2]);

            imu_sender.send(ImuData {
                gyro: filtered_gyr,
                acc: corrected_acc,
                mag,
                dt,
            });
            total_ticks += 1;
        }

        loop_ticker.next().await;
    }
}
