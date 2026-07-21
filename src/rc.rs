use crate::{setup, telemetry::Category};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, with_timeout};
use crate::consts::{RC_MIN, RC_MAX, KP_MIN, KP_MAX, KI_MIN, KI_MAX};

pub static RC_DATA: Watch<CriticalSectionRawMutex, RcData, 1> = Watch::new();

#[derive(Clone)]
pub struct RcData([u16; 16]);

impl RcData {
    pub const fn from_channels(channels: [u16; 16]) -> RcData {
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

    pub fn kp_gain(&self) -> f32 {
        Self::normalize(self.0[4], RC_MIN, RC_MAX, KP_MIN, KP_MAX)
    }

    pub fn ki_gain(&self) -> f32 {
        Self::normalize(self.0[5], RC_MIN, RC_MAX, KI_MIN, KI_MAX)
    }

    pub fn arm_switch(&self) -> f32 {
        Self::normalize(self.0[6], RC_MIN, RC_MAX, 0.0, 1.0)
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

#[embassy_executor::task]
pub async fn rc_task(mut uart: setup::UartReader) -> ! {
    let rc_timeout = Duration::from_millis(100);
    let mut read_buffer = [0u8; 25];
    let mut sbusparser = sbus::SBusPacketParser::new();
    let rc_sender = RC_DATA.sender();

    loop {
        let read_result = with_timeout(rc_timeout, uart.read(&mut read_buffer)).await;
        match read_result {
            Ok(Ok(())) => {
                sbusparser.push_bytes(&read_buffer);
                if let Some(packet) = sbusparser.try_parse() {
                    match packet.failsafe {
                        false => {
                            let rc_data = RcData::from_channels(packet.channels);

                            #[rustfmt::skip]
                            tele!(
                                1, Category::Rc,
                                rc_data.0[0], rc_data.0[1], rc_data.0[2],
                                rc_data.0[3], rc_data.0[4], rc_data.0[5], 
                                rc_data.0[6]);

                            rc_sender.send(rc_data);
                            continue;
                        }
                        true => log::error!("Failsafe"),
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
    }
}
