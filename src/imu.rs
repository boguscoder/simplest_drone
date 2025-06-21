use crate::setup;
use crate::telemetry::Category;
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

    loop {
        let Ok(imudata) = imu.read_9dof().await else {
            log::error!("Failed to read IMU");
            continue;
        };

        #[rustfmt::skip]
        crate::tele!(Category::Imu, "{},{},{},{},{},{},{},{},{}",
            imudata.gyr[0], imudata.gyr[1], imudata.gyr[2],
            imudata.acc[0], imudata.acc[1], imudata.acc[2],
            imudata.mag[0], imudata.mag[1], imudata.mag[2]);

        if ticks == 0 {
            log::info!("Calibration...");
            ticks += 1;
        } else if ticks < CALIBRATION_TICKS {
            ticks += 1;
            gyr_bias += Vector3::from(imudata.gyr);
        } else if ticks == CALIBRATION_TICKS {
            gyr_bias /= CALIBRATION_TICKS as f32;
            log::info!("Calibrated after {} ticks, gyro bias {:?}", ticks, gyr_bias,);
            ticks += 1;
        } else {
            imu_sender.send(Data6Dof::<f32> {
                gyr: (Vector3::from(imudata.gyr) - gyr_bias).into(),
                acc: imudata.acc,
                tmp: imudata.tmp,
            })
        }

        loop_ticker.next().await;
    }
}
