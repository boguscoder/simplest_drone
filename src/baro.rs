use crate::{consts::BARO_HZ, setup, telemetry::Category};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Ticker};

pub static ALT_DATA: Watch<CriticalSectionRawMutex, f32, 1> = Watch::new();

fn precise_altitude(pressure_hpa: f32, temp_celsius: f32, sea_level_pressure: f32) -> f32 {
    if pressure_hpa <= 0.0 {
        return 0.0;
    }
    // Convert ambient temperature to Kelvin
    let temp_kelvin = temp_celsius + 273.15;
    // Hypsometric constants for the troposphere
    let lapse_rate = 0.0065; // Temperature drop per meter (K/m)
    let exponent = 0.190284; // (R * L) / (g * M)
    let pressure_ratio = sea_level_pressure / pressure_hpa;
    let power_result = libm::powf(pressure_ratio, exponent);
    (temp_kelvin / lapse_rate) * (power_result - 1.0)
}

#[embassy_executor::task]
pub async fn baro_task(mut baro: setup::BaroReader) -> ! {
    let mut loop_ticker = Ticker::every(Duration::from_hz(BARO_HZ));
    let alt_sender = ALT_DATA.sender();

    loop {
        let Ok(data) = baro.sensor_data().await else {
            log::error!("Failed to read Barometer pressure");
            continue;
        };

        let alt = precise_altitude(
            data.pressure as f32 / 100.0,
            data.temperature as f32,
            1013.25,
        );

        tele!(1, Category::Baro, data.temperature as f32, alt);
        alt_sender.send(alt);
        loop_ticker.next().await;
    }
}
