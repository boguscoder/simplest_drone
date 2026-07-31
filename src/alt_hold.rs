use crate::{consts::ALT_HOLD_THROTTLE_MIN, rc::RcData, switch::SwitchingPolicy};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

pub static ALT_HOLD_ON_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static ALT_HOLD_OFF_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct AltHold;

impl SwitchingPolicy for AltHold {
    type SafetyContext = bool; // armed

    const NAME: &'static str = "ALT_HOLD";
    const ON_TICKS: u64 = 10;
    const OFF_TICKS: u64 = 10;

    const ON_SIGNAL: Option<&'static Signal<CriticalSectionRawMutex, ()>> =
        Some(&ALT_HOLD_ON_SIGNAL);
    const OFF_SIGNAL: Option<&'static Signal<CriticalSectionRawMutex, ()>> =
        Some(&ALT_HOLD_OFF_SIGNAL);

    #[inline(always)]
    fn want_on(rc: &RcData) -> bool {
        rc.altitude_switch() > 0.5 && rc.throttle() > ALT_HOLD_THROTTLE_MIN
    }

    #[inline(always)]
    fn want_off(rc: &RcData) -> bool {
        rc.altitude_switch() < 0.5
    }

    #[inline(always)]
    fn force_off(rc: &RcData, armed: bool) -> bool {
        !armed || rc.throttle() < 0.05
    }
}
