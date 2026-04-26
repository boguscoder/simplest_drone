use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Category {
    None = 0,
    Imu,
    Attitude,
    Pid,
    Mix,
    Dshot,
}

#[cfg(feature = "telemetry")]
pub static TELE_CATEGORY: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);

#[macro_export]
macro_rules! tele {
    ($cat:expr, $($arg:tt)*) => {
        #[cfg(feature = "telemetry")]
        {
            let current = $crate::telemetry::Category::try_from(
                $crate::telemetry::TELE_CATEGORY.load(portable_atomic::Ordering::Relaxed)
            ).unwrap_or($crate::telemetry::Category::None);
            if current == $cat {
                $crate::rl_log!($crate::LOG_DIVISIOR, $($arg)*);
            }
        }
        #[cfg(not(feature = "telemetry"))]
        {
            let _ = $cat;
        }
    }
}
