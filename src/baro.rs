use crate::{consts::BARO_HZ, setup};
use drone_consts::telemetry::Category;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Ticker};

pub static ALT_DATA: Watch<CriticalSectionRawMutex, f32, 1> = Watch::new();

#[embassy_executor::task]
pub async fn baro_task(mut baro: setup::BaroReader) -> ! {
    let mut loop_ticker = Ticker::every(Duration::from_hz(BARO_HZ));
    let alt_sender = ALT_DATA.sender();

    let mut ground_pa: Option<f32> = None;
    let mut tick_count: usize = 0;
    let mut pa_accumulator = 0.0f32;

    const SKIP_TICKS: usize = 25;
    const CALIB_TICKS: usize = SKIP_TICKS;

    loop {
        let Ok(data) = baro.sensor_data().await else {
            continue;
        };

        let current_pa = data.pressure as f32;
        tick_count += 1;

        if tick_count <= SKIP_TICKS {
            loop_ticker.next().await;
            continue;
        }

        let relative_alt = match ground_pa {
            Some(base) => (base - current_pa) * 0.0843,
            None => {
                pa_accumulator += current_pa;

                if tick_count >= (SKIP_TICKS + CALIB_TICKS) {
                    let avg_base = pa_accumulator / (CALIB_TICKS as f32);
                    ground_pa = Some(avg_base);
                    log::info!(
                        "Baro calibrated after {} ticks. Base: {:.2} Pa",
                        tick_count,
                        avg_base
                    );
                }
                0.0
            }
        };

        tele!(Category::Baro, relative_alt);
        alt_sender.send(relative_alt);
        loop_ticker.next().await;
    }
}
