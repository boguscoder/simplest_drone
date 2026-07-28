use crate::{consts::BARO_HZ, setup, telemetry::Category};
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn baro_task(mut baro: setup::BaroReader) -> ! {
    let mut loop_ticker = Ticker::every(Duration::from_hz(BARO_HZ));

    loop {
        let Ok(data) = baro.sensor_data().await else {
            log::error!("Failed to read Barometer pressure");
            continue;
        };

        tele!(
            1,
            Category::Baro,
            data.pressure as f32 / 1000.0,
            data.temperature as f32
        );

        loop_ticker.next().await;
    }
}
