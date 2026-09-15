#[cfg(feature = "telemetry")]
use crate::{
    cmd,
    telemetry::USB_CHANNEL,
};
use crate::consts::{USB_PID, USB_VID};

#[cfg(feature = "telemetry")]
use embassy_futures::join::{join, join3};
use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    rom_data::reset_to_usb_boot,
    usb::{Driver, InterruptHandler},
};
use embassy_usb::{
    Builder, Config, Handler,
    control::{OutResponse, Recipient, Request, RequestType},
    types::{InterfaceNumber, StringIndex},
};
#[cfg(feature = "telemetry")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;
type UsbDevice = embassy_usb::UsbDevice<'static, UsbDriver>;

struct PicobootResetHandler {
    iface: InterfaceNumber,
    desc: StringIndex,
}

impl PicobootResetHandler {
    fn new(builder: &mut embassy_usb::Builder<'static, UsbDriver>) -> Self {
        let mut func = builder.function(0xFF, 0x00, 0x01);
        let mut iface = func.interface();
        let iface_num = iface.interface_number();
        let desc = iface.string();
        iface.alt_setting(0xFF, 0x00, 0x01, Some(desc));
        Self {
            iface: iface_num,
            desc,
        }
    }
}

impl Handler for PicobootResetHandler {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        (index == self.desc).then_some("rp2xxx-reset")
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.index == u8::from(self.iface) as u16
            && req.request == 0x01
        {
            reset_to_usb_boot(0, 0);
            Some(OutResponse::Accepted)
        } else {
            None
        }
    }
}

#[cfg(feature = "telemetry")]
async fn usb_log_task(class: CdcAcmClass<'static, UsbDriver>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class).await
}

#[cfg(feature = "telemetry")]
async fn usb_telemetry_task(mut sender: Sender<'static, UsbDriver>) {
    let receiver = USB_CHANNEL.receiver();
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

#[cfg(feature = "telemetry")]
async fn usb_read_task(mut receiver: Receiver<'static, UsbDriver>) {
    let mut buf = [0; 64];
    loop {
        receiver.wait_connection().await;

        while let Ok(count) = receiver.read_packet(&mut buf).await {
            if count > 0 {
                cmd::process_payload(&buf[..count]);
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

    #[cfg(feature = "telemetry")]
    let logger_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    #[cfg(feature = "telemetry")]
    let app_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    {
        static RESET_HANDLER: StaticCell<PicobootResetHandler> = StaticCell::new();
        let reset_handler = RESET_HANDLER.init(PicobootResetHandler::new(&mut builder));
        builder.handler(reset_handler);
    }

    let usb = builder.build();

    #[cfg(feature = "telemetry")]
    {
        let (app_sender, app_receiver) = app_class.split();

        let app_task = join(
            usb_read_task(app_receiver),
            usb_telemetry_task(app_sender),
        );
        join3(usb_run_task(usb), usb_log_task(logger_class), app_task).await;
    }
    #[cfg(not(feature = "telemetry"))]
    {
        usb_run_task(usb).await;
    }
}
