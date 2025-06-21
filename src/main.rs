#![no_std]
#![no_main]

mod attitude;
mod imu;
mod log;
mod motor;
mod pid;
mod rc;
mod setup;
mod telemetry;
mod usb;

use dshot_pio::DshotPioTrait;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use panic_probe as _;

const TICK_HZ: u64 = 1000;

#[cfg(feature = "logging")]
const LOG_DIVISIOR: u64 = 3;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut dshot = setup::connect(spawner).await;

    let mut loop_ticker = Ticker::every(Duration::from_hz(TICK_HZ));
    let mut motor = motor::MotorInput::new(1.0 / TICK_HZ as f32);
    let mut rc_reader = rc::RC_DATA.receiver().unwrap();
    let mut imu_reader = imu::IMU_DATA.receiver().unwrap();

    // PID loop //
    loop {
        if let Some(imudata) = imu_reader.try_get() {
            if let Some(rc_data) = rc_reader.try_get() {
                let throttle = motor.update(&rc_data, &imudata);
                dshot.throttle_clamp(throttle);
            } else {
                dshot.throttle_minimum();
            }
        }

        // Delay until next loop
        loop_ticker.next().await;
    }
}
