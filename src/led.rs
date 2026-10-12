use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use smart_leds::RGB8;

use crate::setup::NeoWs;

#[derive(Clone, Copy, PartialEq)]
pub enum SysState {
    Placement,
    Calibration,
    Ready,
}

pub static LED_STATE: Watch<CriticalSectionRawMutex, SysState, 1> = Watch::new();

#[embassy_executor::task]
pub async fn led_task(mut ws: NeoWs) -> ! {
    let mut sys_rx = LED_STATE.receiver().unwrap();

    loop {
        let color = match sys_rx.changed().await {
            SysState::Placement => RGB8::new(0, 0, 40),
            SysState::Calibration => RGB8::new(40, 20, 0),
            SysState::Ready => RGB8::new(0, 40, 0),
        };

        ws.write(&[color]).await;
    }
}
