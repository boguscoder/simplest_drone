#![cfg(feature = "telemetry")]

use crate::{blackbox, telemetry::TELE_MODE};
use drone_consts::telemetry::*;
use portable_atomic::Ordering;

pub fn process_payload(data: &[u8]) {
    if data.is_empty() {
        return;
    }

    if let Ok(cmd) = Command::try_from(data[0]) {
        match cmd {
            Command::SetTelemetryMode(category) => {
                TELE_MODE.store(category as u8, Ordering::Relaxed);
            }
            Command::DumpFlash => {
                blackbox::DUMP_SIGNAL.signal(());
            }
        }
    }
}
