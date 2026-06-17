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

#[cfg(feature = "logging")]
mod usb;

use arming::{Arming, ArmingState};
use embassy_dshot::DshotPioTrait;
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
    let mut att_reader = imu::ATT_DATA.receiver().unwrap();

    const ZERO_RC: RcData = RcData::from_channels([0; 16]);

    loop {
        let att = att_reader.try_get();
        let rc = rc_reader.try_get();

        let rc_ref = rc.as_ref().unwrap_or(&ZERO_RC);
        arming.update(rc_ref, rc.is_some());

        let throttle = if let (Some(att), Some(rc)) = (att, rc) {
            Some(motor.update(&rc, &att))
        } else {
            None
        };

        match (throttle, arming.state()) {
            (Some(t), ArmingState::Armed) => dshot.throttle_clamp(t).unwrap_or_default(),
            _ => dshot.throttle_idle(),
        }

        loop_ticker.next().await;
    }
}
