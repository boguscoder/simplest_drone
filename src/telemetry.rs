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
pub static mut TELE_CATEGORY: Category = Category::None;

#[macro_export]
macro_rules! tele {
    ($cat:expr, $($arg:tt)*) => {
        #[cfg(feature = "telemetry")]
        {
            let log_tele = unsafe {$crate::telemetry::TELE_CATEGORY} == $cat;
            if log_tele {
                $crate::rl_log!($crate::LOG_DIVISIOR, $($arg)*);
            }
        }
        #[cfg(not(feature = "telemetry"))]
        {
            let _ = $cat;
        }
    }
}
