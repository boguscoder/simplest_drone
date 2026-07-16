use crate::{imu, rc};
use embassy_dshot::{DshotPioTrait, DshotSpeed, rp::DshotPio};
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    clocks::{ClockConfig, CoreVoltage},
    config::Config,
    i2c,
    multicore::Stack,
    uart::{self, DataBits, Parity, StopBits, UartRx},
};
use embassy_time::Delay;
use icm20948_async::{
    AccDlp, AccRange, AccUnit, BusI2c, GyrDlp, GyrRange, GyrUnit, Icm20948, IcmBuilder,
};
use static_cell::StaticCell;

#[cfg(feature = "logging")]
use crate::usb;

pub type ImuReader = Icm20948<
    BusI2c<i2c::I2c<'static, crate::device::I2cPeripheral, i2c::Async>>,
    icm20948_async::MagEnabled,
>;
pub type UartReader = UartRx<'static, uart::Async>;

pub async fn connect(spawner: Spawner) -> impl DshotPioTrait<4> {
    let mut clock_cfg = ClockConfig::system_freq(200_000_000).unwrap();
    clock_cfg.core_voltage = CoreVoltage::V1_15;
    let mut config = Config::default();
    config.clocks = clock_cfg;
    let peripherals = embassy_rp::init(config);
    let device = crate::device::Device::new(peripherals);

    #[cfg(feature = "logging")]
    spawner.spawn(usb::usb_setup(device.usb).unwrap());

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
    spawner.spawn(rc::rc_task(uart_rx).unwrap());

    // IMU via UART setup //
    log::info!("// IMU via UART setup //");

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = 400_000;

    let i2c = i2c::I2c::new_async(
        device.imu.i2c,
        device.imu.scl,
        device.imu.sda,
        crate::device::Irqs,
        i2c_config,
    );

    let imu_result = IcmBuilder::new_i2c(i2c, Delay)
        .gyr_range(GyrRange::Dps2000)
        .gyr_unit(GyrUnit::Rps)
        .gyr_dlp(GyrDlp::Hz51)
        .acc_range(AccRange::Gs8)
        .acc_unit(AccUnit::Mpss)
        .acc_dlp(AccDlp::Hz50)
        .set_address(0x69)
        .initialize_9dof()
        .await;

    let Ok(imu) = imu_result else {
        panic!("Failed to initialize IMU")
    };

    static CORE_EXECUTOR: StaticCell<Executor> = StaticCell::new();
    static CORE_STACK: StaticCell<Stack<2048>> = StaticCell::new();

    embassy_rp::multicore::spawn_core1(device.core1, CORE_STACK.init(Stack::new()), move || {
        let executor = CORE_EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(imu::imu_task(imu).unwrap());
        })
    });

    // Motors via DSHOT setup //
    log::info!("// Motors via DSHOT setup //");

    DshotPio::<4, _>::new(
        device.motors.pio,
        crate::device::Irqs,
        //                // My ECS    // 'X' in PX4   // Place
        device.motors.m1, // M4        // M1           // Front Right
        device.motors.m2, // M1        // M2           // Back Left
        device.motors.m3, // M2        // M3           // Front Left
        device.motors.m4, // M3        // M4           // Back Right
        DshotSpeed::DShot600,
    )
}
