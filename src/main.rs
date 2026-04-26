#![no_std]
#![no_main]

#[macro_use]
mod telemetry;

mod arming;
mod attitude;
mod device;
mod imu;
mod log;
mod motor;
mod pid;
mod rc;
mod setup;
mod usb;

use arming::{Arming, ArmingState};
use dshot_pio::DshotPioTrait;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use panic_probe as _;
use rc::RcData;

const TICK_HZ: u64 = 1000;

#[cfg(feature = "logging")]
const LOG_DIVISIOR: u64 = 4;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut dshot = setup::connect(spawner).await;

    let mut loop_ticker = Ticker::every(Duration::from_hz(TICK_HZ));
    let mut motor = motor::MotorInput::new(1.0 / TICK_HZ as f32);
    let mut arming = Arming::new();

    let mut rc_reader = rc::RC_DATA.receiver().unwrap();
    let mut imu_reader = imu::IMU_DATA.receiver().unwrap();
    let zero_rc = RcData::from_channels([0; 16]);

    loop {
        let imu_data = imu_reader.try_get();
        let rc_data = rc_reader.try_get();
        let rc_valid = rc_data.is_some();

        if let Some(ref rc) = rc_data {
            arming.update(rc, rc_valid);
        } else {
            arming.update(&zero_rc, rc_valid);
        }

        if arming.state() == ArmingState::Armed {
            if let Some(imudata) = imu_data {
                if let Some(rc) = rc_data {
                    let throttle = motor.update(&rc, &imudata);
                    dshot.throttle_clamp(throttle);
                } else {
                    dshot.throttle_minimum();
                }
            }
        } else {
            dshot.throttle_clamp(motor::throttle_disarm());
        }

        loop_ticker.next().await;
    }
}
