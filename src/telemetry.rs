use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Category {
    None = 0,
    Imu,
    Rc,
    Attitude,
    Pid,
    Mix,
    Dshot,
}

#[cfg(feature = "telemetry")]
pub static TELE_CATEGORY: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);

#[cfg(feature = "telemetry")]
pub const TELE_MAX_VALUES: usize = 8;
#[cfg(feature = "telemetry")]
pub const TELE_FRAME_SIZE: usize = 2 + TELE_MAX_VALUES * 4;

#[cfg(feature = "telemetry")]
pub type TeleChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    [u8; TELE_FRAME_SIZE],
    32,
>;

#[cfg(feature = "telemetry")]
pub static TELE_CHANNEL: TeleChannel = TeleChannel::new();

#[macro_export]
macro_rules! tele {
    ($cat:path, $($v:expr),+ $(,)?) => {
        $crate::tele_impl!($crate::LOG_DIVISIOR, $cat, $($v),+)
    };
    ($div:expr, $cat:path, $($v:expr),+ $(,)?) => {
        $crate::tele_impl!($div, $cat, $($v),+)
    };
}

#[macro_export]
macro_rules! tele_impl {
    ($div:expr, $cat:path, $($v:expr),+ $(,)?) => {
        #[cfg(feature = "telemetry")]
        {
            static __CALL_COUNTER: portable_atomic::AtomicUsize = portable_atomic::AtomicUsize::new(0);
            if __CALL_COUNTER.fetch_add(1, portable_atomic::Ordering::Relaxed) % $div as usize == 0 {
                let current = $crate::telemetry::Category::try_from(
                    $crate::telemetry::TELE_CATEGORY.load(portable_atomic::Ordering::Relaxed)
                ).unwrap_or($crate::telemetry::Category::None);
                if current == $cat {
                    let values = [$($v as f32),+];
                    let n = values.len().min($crate::telemetry::TELE_MAX_VALUES);
                    let mut frame = [0u8; $crate::telemetry::TELE_FRAME_SIZE];
                    frame[0] = 0xAA;
                    frame[1] = n as u8;
                    for (i, v) in values.iter().take(n).enumerate() {
                        frame[2 + i * 4..6 + i * 4].copy_from_slice(&v.to_le_bytes());
                    }
                    let _ = $crate::telemetry::TELE_CHANNEL.try_send(frame);
                }
            }
        }
        #[cfg(not(feature = "telemetry"))]
        {
            let _ = $div;
            let _ = $cat;
            $(let _ = $v;)+
        }
    };
}
