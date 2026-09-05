use crate::rc::RcData;
use crate::{
    consts::{ARM_HOLD_TICKS, DISARM_HOLD_TICKS},
    switch::{SwitchWatch, SwitchingPolicy},
};

static ARMING_STATE: SwitchWatch = SwitchWatch::new();

pub struct Arming;

impl SwitchingPolicy for Arming {
    type SafetyContext = bool; // rc_valid

    const NAME: &'static str = "ARMING";
    const ON_TICKS: u64 = ARM_HOLD_TICKS;
    const OFF_TICKS: u64 = DISARM_HOLD_TICKS;

    const STATE: &'static SwitchWatch = &ARMING_STATE;

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
