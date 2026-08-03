use crate::consts::{KI_MAX, KI_MIN, KP_MAX, KP_MIN, RC_MAX, RC_MIN};
use crate::setup;
use drone_consts::telemetry::Category;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, with_timeout};

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

    pub fn altitude_switch(&self) -> f32 {
        Self::normalize(self.0[7], RC_MIN, RC_MAX, 0.0, 1.0)
    }

    pub fn unused(&self) -> f32 {
        Self::normalize(self.0[8], RC_MIN, RC_MAX, -1.0, 1.0)
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RcError {
    None,
    Failsafe,
    ReadError,
    Timeout,
}

fn change_state(state: &mut RcError, new_state: RcError) {
    if *state != new_state {
        log::info!("RC state changed from {:?} to {:?}", state, new_state);
        *state = new_state;
    }
}

#[embassy_executor::task]
pub async fn rc_task(mut uart: setup::UartReader) -> ! {
    let rc_timeout = Duration::from_millis(100);
    let mut read_buffer = [0u8; 25];
    let mut sbusparser = sbus::SBusPacketParser::new();
    let rc_sender = RC_DATA.sender();
    let mut state = RcError::None;

    loop {
        let read_result = with_timeout(rc_timeout, uart.read(&mut read_buffer)).await;
        match read_result {
            Ok(Ok(())) => {
                sbusparser.push_bytes(&read_buffer);
                if let Some(packet) = sbusparser.try_parse() {
                    match packet.failsafe {
                        false => {
                            change_state(&mut state, RcError::None);
                            let rc_data = RcData::from_channels(packet.channels);

                            #[rustfmt::skip]
                            tele!(
                                Category::Rc,
                                rc_data.roll(), rc_data.pitch(), rc_data.throttle(),
                                rc_data.yaw(), rc_data.kp_gain(), rc_data.ki_gain(),
                                rc_data.arm_switch(), rc_data.altitude_switch(), rc_data.unused());

                            rc_sender.send(rc_data);
                            continue;
                        }
                        true => change_state(&mut state, RcError::Failsafe),
                    }
                }
            }
            Ok(Err(_e)) => {
                change_state(&mut state, RcError::ReadError);
            }
            Err(_) => {
                change_state(&mut state, RcError::Timeout);
            }
        }
        rc_sender.clear();
    }
}
