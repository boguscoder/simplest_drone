#![no_std]
#![no_main]

#[macro_use]
mod telemetry;

mod alt_estimator;
mod alt_hold;
mod arming;
mod attitude;
mod baro;
mod consts;
mod device;
mod imu;
mod logs;
mod motor;
mod pid;
mod rc;
mod setup;
mod switch;

#[cfg(feature = "logging")]
mod usb;

use alt_estimator::AltitudeEstimator;
use alt_hold::AltHold;
use arming::Arming;
use attitude::Attitude;
use consts::{CYCLE_TIME, TICK_HZ};
use drone_consts::telemetry::Category;
use embassy_dshot::{Command, DshotPioTrait};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use panic_probe as _;
use rc::RcData;
use switch::{Switch, SwitchState};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut dshot = setup::connect(spawner).await;

    let mut loop_ticker = Ticker::every(Duration::from_hz(TICK_HZ));
    let mut motor = motor::MotorInput::new(CYCLE_TIME);
    let mut arming = Switch::<Arming>::new();
    let mut alt_hold = Switch::<AltHold>::new();
    let mut rc_reader = rc::RC_DATA.receiver().unwrap();
    let mut imu_reader = imu::IMU_DATA.receiver().unwrap();
    let mut alt_reader = baro::ALT_DATA.receiver().unwrap();
    let mut att_transformer = Attitude::new();
    let mut alt_estimator = AltitudeEstimator::new();

    const ZERO_RC: RcData = RcData::from_channels([0; 16]);

    loop {
        let imu = imu_reader.try_get();
        let rc = rc_reader.try_get();
        let baro_alt = alt_reader.try_get();

        let rc_ref = rc.as_ref().unwrap_or(&ZERO_RC);
        arming.update(rc_ref, rc.is_some());
        alt_hold.update(rc_ref, arming.state() == SwitchState::Active);

        let throttle = if let (Some(imu), Some(rc), Some(baro_alt)) = (imu, rc, baro_alt) {
            att_transformer
                .update(&imu.gyro, &imu.acc, &imu.mag, imu.dt)
                .map(|quat| {
                    let alt = alt_estimator.update(&quat, &imu, baro_alt);
                    let att: [f32; 3] = quat.euler_angles().into();
                    tele!(Category::Attitude, att[0], att[1], att[2], alt);

                    motor.update(
                        &rc,
                        &imu,
                        &att,
                        alt,
                        arming.state() == SwitchState::Active,
                        alt_hold.state() == SwitchState::Active,
                    )
                })
        } else {
            None
        };

        match (throttle, arming.state()) {
            (Some(t), SwitchState::Active) => dshot.throttle_clamp(t).unwrap_or_default(),
            _ => dshot.send_command(Command::MotorStop),
        }

        loop_ticker.next().await;
    }
}
