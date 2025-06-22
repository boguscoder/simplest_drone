use dshot_pio::DshotPioTrait;
use dshot_pio::dshot_embassy_rp::DshotPio;
use embassy_executor::Spawner;
use embassy_rp::i2c::{self, InterruptHandler as I2CHandler};
use embassy_rp::pio::InterruptHandler as PioHandler;
use embassy_rp::uart::{self, DataBits, InterruptHandler as UartHandler, Parity, StopBits, UartRx};
use embassy_rp::{bind_interrupts, peripherals};
use embassy_time::{Delay, Timer};
use icm20948_async::{
    AccDlp, AccRange, AccUnit, BusI2c, GyrDlp, GyrRange, GyrUnit, Icm20948, IcmBuilder,
};

use crate::imu;
use crate::rc;

#[cfg(feature = "logging")]
use crate::usb;

bind_interrupts!(struct Irqs {
    UART1_IRQ => UartHandler<peripherals::UART1>;
    PIO0_IRQ_0 => PioHandler<peripherals::PIO0>;
    I2C0_IRQ => I2CHandler<peripherals::I2C0>;
});

pub type ImuReader =
    Icm20948<BusI2c<i2c::I2c<'static, peripherals::I2C0, i2c::Async>>, icm20948_async::MagEnabled>;
pub type UartReader = UartRx<'static, peripherals::UART1, uart::Async>;

pub async fn connect(spawner: Spawner) -> impl DshotPioTrait<4> {
    let p = embassy_rp::init(Default::default());

    #[cfg(feature = "logging")]
    {
        let (usb_dev, log_class, app_class) = usb::usb_setup(p.USB);
        spawner.must_spawn(usb::usb_run_task(usb_dev));
        spawner.must_spawn(usb::usb_log_task(log_class));
        spawner.must_spawn(usb::usb_read_task(app_class));
    }
    // RC via SBUS setup //
    log::info!("// RC via SBUS setup //");

    let uart = p.UART1;
    let rx = p.PIN_5;
    let dma = p.DMA_CH1;

    let mut sbus_uart_config = uart::Config::default();
    sbus_uart_config.baudrate = 100_000;
    sbus_uart_config.data_bits = DataBits::DataBits8;
    sbus_uart_config.stop_bits = StopBits::STOP2;
    sbus_uart_config.parity = Parity::ParityEven;
    sbus_uart_config.invert_rx = true;

    let uart_rx = embassy_rp::uart::UartRx::new(uart, rx, Irqs, dma, sbus_uart_config);
    spawner.must_spawn(rc::rc_task(uart_rx));

    // IMU via UART setup //
    log::info!("// IMU via UART setup //");

    let sda = p.PIN_0;
    let scl = p.PIN_1;

    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c::Config::default());

    let imu_result = IcmBuilder::new_i2c(i2c, Delay)
        .gyr_range(GyrRange::Dps2000)
        .gyr_unit(GyrUnit::Rps)
        .gyr_dlp(GyrDlp::Hz196)
        .acc_range(AccRange::Gs8)
        .acc_unit(AccUnit::Mpss)
        .acc_dlp(AccDlp::Hz246)
        .set_address(0x69)
        .initialize_9dof()
        .await;

    let Ok(imu) = imu_result else {
        panic!("Failed to initialize IMU")
    };
    spawner.must_spawn(imu::imu_task(imu));

    // Motors via DSHOT setup //
    log::info!("// Motors via DSHOT setup //");

    let mut dshot = DshotPio::<4, _>::new(
        p.PIO0,
        Irqs,
        //        // My Solder:)  // Canonical 'X' // Place
        p.PIN_10, // M4           // M1            // Front Right
        p.PIN_20, // M1           // M2            // Back Left
        p.PIN_21, // M2           // M3            // Front Left
        p.PIN_11, // M3           // M4            // Back Right
        (52, 0),  // clock divider
    );

    for _ in 0..20 {
        dshot.throttle_minimum();
        Timer::after_millis(50).await;
    }
    dshot
}
