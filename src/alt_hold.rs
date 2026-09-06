use crate::{
    consts::*,
    rc::RcData,
    switch::{SwitchWatch, SwitchingPolicy},
};

static ALT_HOLD_STATE: SwitchWatch = SwitchWatch::new();

pub struct AltHold;

impl SwitchingPolicy for AltHold {
    type SafetyContext = bool; // armed

    const NAME: &'static str = "ALT_HOLD";

    const STATE: &'static SwitchWatch = &ALT_HOLD_STATE;

    #[inline(always)]
    fn want_on(rc: &RcData) -> bool {
        rc.altitude_switch() > 0.5
            && rc.throttle() > ALT_HOLD_THROTTLE_MIN
            && rc.throttle() < ALT_HOLD_THROTTLE_MAX
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
