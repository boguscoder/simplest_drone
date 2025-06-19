#![cfg(feature = "logging")]

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb_logger::ReceiverHandler;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = str::from_utf8(data) {
            let data = data.trim();
            log::info!("Recieved: {:?}", data);
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
pub async fn usb_setup(p: embassy_rp::peripherals::USB) {
    let driver = Driver::new(p, Irqs);
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, Handler);
}
