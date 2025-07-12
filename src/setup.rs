use dshot_pio::DshotPioTrait;
use dshot_pio::dshot_embassy_rp::DshotPio;
use embassy_executor::Spawner;
use embassy_rp::i2c;
use embassy_rp::uart::{self, DataBits, Parity, StopBits, UartRx};
use embassy_time::{Delay, Timer};
use icm20948_async::{
    AccDlp, AccRange, AccUnit, BusI2c, GyrDlp, GyrRange, GyrUnit, Icm20948, IcmBuilder,
};

use crate::imu;
use crate::rc;

#[cfg(feature = "logging")]
use crate::usb;

pub type ImuReader = Icm20948<
    BusI2c<i2c::I2c<'static, crate::device::I2cPeripheral, i2c::Async>>,
    icm20948_async::MagDisabled,
>;
pub type UartReader = UartRx<'static, crate::device::SbusUartPeripheral, uart::Async>;

pub async fn connect(spawner: Spawner) -> impl DshotPioTrait<4> {
    let device = crate::device::Device::new(embassy_rp::init(Default::default()));

    #[cfg(feature = "logging")]
    spawner.must_spawn(usb::usb_setup(device.usb));

    // RC via SBUS setup //
    log::info!("// RC via SBUS setup //");

    let mut sbus_uart_config = uart::Config::default();
    sbus_uart_config.baudrate = 100_000;
    sbus_uart_config.data_bits = DataBits::DataBits8;
    sbus_uart_config.stop_bits = StopBits::STOP2;
    sbus_uart_config.parity = Parity::ParityEven;
    sbus_uart_config.invert_rx = true;

    let uart_rx = embassy_rp::uart::UartRx::new(
        device.rc.uart,
        device.rc.rx,
        crate::device::Irqs,
        device.rc.dma,
        sbus_uart_config,
    );
    spawner.must_spawn(rc::rc_task(uart_rx));

    // IMU via UART setup //
    log::info!("// IMU via UART setup //");

    let i2c = i2c::I2c::new_async(
        device.imu.i2c,
        device.imu.scl,
        device.imu.sda,
        crate::device::Irqs,
        i2c::Config::default(),
    );

    let imu_result = IcmBuilder::new_i2c(i2c, Delay)
        .gyr_range(GyrRange::Dps2000)
        .gyr_unit(GyrUnit::Rps)
        .gyr_dlp(GyrDlp::Hz196)
        .acc_range(AccRange::Gs8)
        .acc_unit(AccUnit::Mpss)
        .acc_dlp(AccDlp::Hz246)
        .set_address(0x69)
        .initialize_6dof()
        .await;

    let Ok(imu) = imu_result else {
        panic!("Failed to initialize IMU")
    };
    spawner.must_spawn(imu::imu_task(imu));

    // Motors via DSHOT setup //
    log::info!("// Motors via DSHOT setup //");

    let mut dshot = DshotPio::<4, _>::new(
        device.motors.pio,
        crate::device::Irqs,
        //                // My Solder:)  // Canonical 'X' // Place
        device.motors.m1, // M4           // M1            // Front Right
        device.motors.m2, // M1           // M2            // Back Left
        device.motors.m3, // M2           // M3            // Front Left
        device.motors.m4, // M3           // M4            // Back Right
        (52, 0),          // clock divider
    );

    for _ in 0..20 {
        dshot.throttle_minimum();
        Timer::after_millis(50).await;
    }
    dshot
}
