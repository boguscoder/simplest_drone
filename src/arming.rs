use crate::rc::RcData;
use crate::{
    consts::{ARM_HOLD_TICKS, DISARM_HOLD_TICKS},
    switch::SwitchingPolicy,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

pub static DISARMED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct Arming;

impl SwitchingPolicy for Arming {
    type SafetyContext = bool; // rc_valid

    const NAME: &'static str = "ARMING";
    const ON_TICKS: u64 = ARM_HOLD_TICKS;
    const OFF_TICKS: u64 = DISARM_HOLD_TICKS;

    const OFF_SIGNAL: Option<&'static Signal<CriticalSectionRawMutex, ()>> = Some(&DISARMED);

    #[inline(always)]
    fn want_on(rc: &RcData) -> bool {
        rc.throttle() < 0.1 && rc.arm_switch() > 0.5
    }

    #[inline(always)]
    fn want_off(rc: &RcData) -> bool {
        rc.arm_switch() < 0.5
    }

    #[inline(always)]
    fn force_off(_: &RcData, rc_valid: bool) -> bool {
        !rc_valid // Safety trip: lost RC signal
    }
}
