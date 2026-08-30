#[cfg(feature = "telemetry")]
pub static TELE_MODE: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);

#[cfg(feature = "telemetry")]
pub type TeleChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    [u8; crate::consts::TELE_FRAME_SIZE],
    32,
>;

#[cfg(feature = "telemetry")]
pub static USB_CHANNEL: TeleChannel = TeleChannel::new();
#[cfg(feature = "telemetry")]
pub static BBOX_CHANNEL: TeleChannel = TeleChannel::new();

#[macro_export]
macro_rules! tele {
    ($cat:path, $($v:expr),+ $(,)?) => {
        #[cfg(feature = "telemetry")]
        {
            use portable_atomic::Ordering;
            let current = Mode::try_from(
                $crate::telemetry::TELE_MODE.load(Ordering::Relaxed)
            ).unwrap_or(Mode::None);
            if current == $cat {
                let values = [$($v as f32),+];
                let n = values.len().min($crate::consts::TELE_MAX_VALUES);
                let mut frame = [0u8; $crate::consts::TELE_FRAME_SIZE];
                frame[0] = 0xAA;
                frame[1] = n as u8;
                for (i, v) in values.iter().take(n).enumerate() {
                    frame[2 + i * 4..6 + i * 4].copy_from_slice(&v.to_le_bytes());
                }
                let _ = $crate::telemetry::USB_CHANNEL.try_send(frame);
                let _ = $crate::telemetry::BBOX_CHANNEL.try_send(frame);
            }
        }
        #[cfg(not(feature = "telemetry"))]
        {
            let _ = $cat;
            $(let _ = $v;)+
        }
    };
}
