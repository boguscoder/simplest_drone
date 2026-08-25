#![cfg(feature = "telemetry")]

use crate::{blackbox, telemetry::TELE_CATEGORY};
use drone_consts::telemetry::*;
use portable_atomic::Ordering;

pub fn process_payload(data: &[u8]) {
    if data.is_empty() {
        return;
    }

    if let Ok(category) = Category::try_from(data[0]) {
        if category == Category::Dump {
            blackbox::DUMP_SIGNAL.signal(());
        }
        TELE_CATEGORY.store(category as u8, Ordering::Relaxed);
    }
}
