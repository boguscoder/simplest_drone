use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::Timer;
use smart_leds::RGB8;

use crate::setup::NeoWs;

#[derive(Clone, Copy, PartialEq)]
pub enum SysState {
    Placement,
    Calibration,
    Ready,
    Armed,
    AltHold,
    Fail,
}

pub static LED_STATE: Watch<CriticalSectionRawMutex, SysState, 1> = Watch::new();

#[embassy_executor::task]
pub async fn led_task(mut ws: NeoWs) -> ! {
    let mut sys_rx = LED_STATE.receiver().unwrap();
    let mut on = false;

    loop {
        on = !on;
        let base = match sys_rx.try_get().unwrap_or(SysState::Placement) {
            SysState::Placement => RGB8::new(0, 0, 255),
            SysState::Calibration => RGB8::new(255, 140, 0),
            SysState::Ready => RGB8::new(40, 40, 40),
            SysState::Armed => RGB8::new(0, 255, 0),
            SysState::AltHold => RGB8::new(0, 255, 255),
            SysState::Fail => RGB8::new(255, 0, 0),
        };

        ws.write(&[if on { base } else { RGB8::new(0, 0, 0) }]).await;
        Timer::after_millis(250).await;
    }
}
