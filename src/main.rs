#![no_std]
#![no_main]

#[macro_use]
mod telemetry;

mod alt_estimator;
mod alt_hold;
mod arming;
mod attitude;
mod baro;
mod cmd;
mod consts;
mod device;
mod imu;
mod logs;
mod motor;
mod pid;
mod rc;
mod setup;
mod switch;
mod usb;

#[cfg(feature = "telemetry")]
mod blackbox;

#[cfg(feature = "telemetry")]
use blackbox::BlackBoxSwitch;

use alt_estimator::AltitudeEstimator;
use alt_hold::AltHold;
use arming::Arming;
use attitude::Attitude;
use consts::*;
use drone_consts::telemetry::*;
use embassy_dshot::{Command, DshotPioTrait};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
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

    #[cfg(feature = "telemetry")]
    let mut bbox = Switch::<BlackBoxSwitch>::new();

    const ZERO_RC: RcData = RcData::from_channels([0; 16]);

    let mut last_imu: Option<Instant> = None;
    let mut last_baro: Option<Instant> = None;

    loop {
        let now = Instant::now();
        let rc = rc_reader.try_get();

        let imu = match imu_reader.try_changed() {
            Some(fresh) => {
                last_imu = Some(now);
                Some(fresh)
            }
            None => imu_reader.try_get(),
        };
        let imu_ok =
            last_imu.is_none_or(|t| now.duration_since(t) <= Duration::from_millis(IMU_STALE_MS));
        if !imu_ok {
            rl_log!(TICK_HZ, "IMU stale, failsafe disarm");
        }

        let baro_alt = match alt_reader.try_changed() {
            Some(fresh) => {
                last_baro = Some(now);
                Some(fresh)
            }
            None => alt_reader.try_get(),
        };
        let baro_ok =
            last_baro.is_none_or(|t| now.duration_since(t) <= Duration::from_millis(BARO_STALE_MS));
        if !baro_ok {
            rl_log!(TICK_HZ, "Baro stale, alt-hold off");
        }

        let rc_ref = rc.as_ref().unwrap_or(&ZERO_RC);
        arming.update(rc_ref, rc.is_some() && imu_ok);
        alt_hold.update(rc_ref, arming.state() == SwitchState::Active && baro_ok);

        #[cfg(feature = "telemetry")]
        bbox.update(rc_ref, ());

        let throttle = if let (Some(imu), Some(rc), Some(baro_alt)) = (imu, rc, baro_alt) {
            att_transformer
                .update(&imu.gyro, &imu.acc, &imu.mag, imu.dt)
                .map(|quat| {
                    let alt = alt_estimator.update(&quat, &imu, baro_alt, imu.dt);
                    let att: [f32; 3] = quat.euler_angles().into();
                    tele!(Mode::Attitude, att[0], att[1], att[2], alt);

                    motor.update(&rc, &imu, &att, alt, arming.state() == SwitchState::Active)
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
