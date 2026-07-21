#![cfg(feature = "logging")]

use crate::consts::{USB_PID, USB_VID};
use crate::telemetry::Category;
use core::convert::TryFrom;
use embassy_futures::join::{join, join3};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;
type UsbDevice = embassy_usb::UsbDevice<'static, UsbDriver>;

fn handle_data(data: &[u8]) {
    #[cfg(feature = "telemetry")]
    {
        match Category::try_from(data[0]) {
            Ok(cat) => {
                crate::telemetry::TELE_CATEGORY
                    .store(cat as u8, portable_atomic::Ordering::Relaxed);
            }
            Err(_) => {
                crate::telemetry::TELE_CATEGORY
                    .store(Category::None as u8, portable_atomic::Ordering::Relaxed);
            }
        }
    }
}

async fn usb_log_task(class: CdcAcmClass<'static, UsbDriver>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class).await
}

async fn usb_telemetry_task(mut sender: Sender<'static, UsbDriver>) {
    #[cfg(feature = "telemetry")]
    {
        let receiver = crate::telemetry::TELE_CHANNEL.receiver();
        loop {
            sender.wait_connection().await;
            loop {
                let frame = receiver.receive().await;
                let len = frame[1] as usize;
                let frame_len = 2 + len * 4;
                if sender.write_packet(&frame[..frame_len]).await.is_err() {
                    break;
                }
            }
        }
    }
    #[cfg(not(feature = "telemetry"))]
    loop {
        embassy_time::Timer::after_secs(3600).await;
    }
}

async fn usb_read_task(mut receiver: Receiver<'static, UsbDriver>) {
    let mut buf = [0; 64];
    loop {
        receiver.wait_connection().await;

        while let Ok(count) = receiver.read_packet(&mut buf).await {
            if count > 0 {
                handle_data(&buf[..count]);
            }
        }
    }
}

async fn usb_run_task(mut dev: UsbDevice) {
    dev.run().await;
}

#[embassy_executor::task]
pub async fn usb_setup(p: embassy_rp::Peri<'static, embassy_rp::peripherals::USB>) {
    let driver = Driver::new(p, Irqs);
    let mut config = Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("Embassy");
    config.product = Some("Drone Console");
    config.serial_number = Some("0xBABECAFE");
    config.max_power = 250;
    config.max_packet_size_0 = 64;

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [],
            CONTROL_BUF.init([0; 64]),
        )
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
    let (app_sender, app_receiver) = app_class.split();

    let app_task = join(usb_read_task(app_receiver), usb_telemetry_task(app_sender));
    join3(usb_run_task(usb), usb_log_task(logger_class), app_task).await;
}
