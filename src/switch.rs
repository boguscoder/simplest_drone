use crate::rc::RcData;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

pub trait SwitchingPolicy {
    type SafetyContext;

    fn want_on(rc: &RcData) -> bool;
    fn want_off(rc: &RcData) -> bool;
    fn force_off(rc: &RcData, ctx: Self::SafetyContext) -> bool;

    const ON_TICKS: u64;
    const OFF_TICKS: u64;

    const NAME: &'static str;

    const ON_SIGNAL: Option<&'static Signal<CriticalSectionRawMutex, ()>> = None;
    const OFF_SIGNAL: Option<&'static Signal<CriticalSectionRawMutex, ()>> = None;
}
use core::marker::PhantomData;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SwitchState {
    Inactive,
    Active,
}

pub struct Switch<P: SwitchingPolicy> {
    state: SwitchState,
    ticks: u64,
    _policy: PhantomData<P>,
}

impl<P: SwitchingPolicy> Switch<P> {
    pub const fn new() -> Self {
        Self {
            state: SwitchState::Inactive,
            ticks: 0,
            _policy: PhantomData,
        }
    }

    pub fn update(&mut self, rc: &RcData, ctx: P::SafetyContext) -> SwitchState {
        if P::force_off(rc, ctx) {
            self.ticks = 0;
            if self.state == SwitchState::Active {
                self.transition_to(SwitchState::Inactive, true);
            }
            return self.state;
        }

        let target_condition = match self.state {
            SwitchState::Inactive => P::want_on(rc),
            SwitchState::Active => P::want_off(rc),
        };

        if target_condition {
            self.ticks += 1;
            let threshold = match self.state {
                SwitchState::Inactive => P::ON_TICKS,
                SwitchState::Active => P::OFF_TICKS,
            };

            if self.ticks >= threshold {
                let next_state = match self.state {
                    SwitchState::Inactive => SwitchState::Active,
                    SwitchState::Active => SwitchState::Inactive,
                };
                self.transition_to(next_state, false);
            }
        } else {
            self.ticks = 0;
        }

        self.state
    }

    fn transition_to(&mut self, next_state: SwitchState, forced: bool) {
        self.state = next_state;
        self.ticks = 0;

        match next_state {
            SwitchState::Active => {
                log::info!("[MODE] {} ENABLED", P::NAME);
                if let Some(signal) = P::ON_SIGNAL {
                    signal.signal(());
                }
            }
            SwitchState::Inactive => {
                if forced {
                    log::warn!("[MODE] {} FORCE DISENGAGED (Failsafe)", P::NAME);
                } else {
                    log::info!("[MODE] {} DISABLED", P::NAME);
                }
                if let Some(signal) = P::OFF_SIGNAL {
                    signal.signal(());
                }
            }
        }
    }

    #[inline(always)]
    pub fn state(&self) -> SwitchState {
        self.state
    }
}
