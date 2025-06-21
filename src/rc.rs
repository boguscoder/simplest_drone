use crate::setup;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker, with_timeout};

const RC_MIN: u16 = 240;
const RC_MAX: u16 = 1807;
const RC_FAILSAFE: u16 = 1500;

pub static RC_DATA: Watch<CriticalSectionRawMutex, RcData, 1> = Watch::new();

#[derive(Clone)]
pub struct RcData([u16; 16]);

impl RcData {
    pub fn from_channels(channels: [u16; 16]) -> RcData {
        RcData(channels)
    }

    pub fn roll(&self) -> f32 {
        Self::normalize(self.0[0], RC_MIN, RC_MAX, -1.0, 1.0)
    }

    pub fn pitch(&self) -> f32 {
        Self::normalize(self.0[1], RC_MIN, RC_MAX, -1.0, 1.0)
    }

    pub fn throttle(&self) -> f32 {
        Self::normalize(self.0[2], RC_MIN, RC_MAX, 0.0, 1.0)
    }

    pub fn yaw(&self) -> f32 {
        Self::normalize(self.0[3], RC_MIN, RC_MAX, -1.0, 1.0)
    }

    fn swd(&self) -> u16 {
        self.0[6]
    }

    fn normalize(
        val: u16,
        original_min: u16,
        original_max: u16,
        new_min: f32,
        new_max: f32,
    ) -> f32 {
        new_min
            + ((new_max - new_min)
                * ((val as f32 - original_min as f32) / (original_max - original_min) as f32))
    }
}

impl core::fmt::Debug for RcData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RcData")
            .field("throttle", &self.throttle())
            .field("roll", &self.roll())
            .field("pitch", &self.pitch())
            .field("yaw", &self.yaw())
            .finish()
    }
}

#[embassy_executor::task]
pub async fn rc_task(mut uart: setup::UartReader) -> ! {
    let rc_timeout = Duration::from_millis(100);
    let mut read_buffer = [0u8; 25];
    let mut sbusparser = sbus_parser::receiver::Receiver::new();
    let rc_sender = RC_DATA.sender();
    let mut loop_ticker = Ticker::every(Duration::from_hz(100));

    loop {
        let read_result = with_timeout(rc_timeout, uart.read(&mut read_buffer)).await;
        match read_result {
            Ok(Ok(_)) => {
                if let Some(packet) = sbusparser.receive(&read_buffer) {
                    log::trace!("rc {:?}", packet.channels);

                    let rc_data = RcData::from_channels(packet.channels);
                    if rc_data.swd() >= RC_FAILSAFE {
                        log::error!("Failsafe");
                    } else {
                        rc_sender.send(rc_data);
                        continue;
                    }
                }
            }
            Ok(Err(_e)) => {
                log::error!("Serial read err {_e:?}");
            }
            Err(_) => {
                log::error!("Serial timeout");
            }
        }
        rc_sender.clear();
        loop_ticker.next().await;
    }
}
