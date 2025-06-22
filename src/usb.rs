#![cfg(feature = "logging")]

use crate::telemetry;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type AcmClass = CdcAcmClass<'static, Driver<'static, USB>>;
type UsbDevice = embassy_usb::UsbDevice<'static, Driver<'static, USB>>;

fn handle_data(data: &[u8]) {
    #[cfg(feature = "telemetry")]
    {
        match telemetry::Category::try_from(data[0]) {
            Ok(cat) => unsafe {
                telemetry::TELE_CATEGORY = cat;
            },
            Err(_) => unsafe {
                telemetry::TELE_CATEGORY = telemetry::Category::None;
            },
        }
    }
}

async fn usb_log_task(class: AcmClass) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class).await
}

async fn usb_read_task(mut class: AcmClass) {
    let mut buf = [0; 64];
    loop {
        class.wait_connection().await;

        loop {
            match class.read_packet(&mut buf).await {
                Ok(count) => {
                    if count > 0 {
                        handle_data(&buf[..count]);
                    }
                }
                Err(e) => {
                    log::error!("App Class RX Error: {:?}", e);
                }
            }
        }
    }
}

async fn usb_run_task(mut dev: UsbDevice) {
    dev.run().await;
}

#[embassy_executor::task]
pub async fn usb_setup(p: embassy_rp::peripherals::USB) {
    let driver = Driver::new(p, Irqs);
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial console");
    config.serial_number = Some("0xC0DECAFE");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    let logger_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let app_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let usb = builder.build();

    join(
        usb_run_task(usb),
        join(usb_log_task(logger_class), usb_read_task(app_class)),
    )
    .await;
}
