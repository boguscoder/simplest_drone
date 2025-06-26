use embassy_rp::i2c::InterruptHandler as I2CHandler;
use embassy_rp::pio::InterruptHandler as PioHandler;
use embassy_rp::uart::InterruptHandler as UartHandler;
use embassy_rp::{bind_interrupts, peripherals};

#[cfg(feature = "feather")]
pub mod device_impl {
    pub type SbusUartPeripheral = super::peripherals::UART1;
    pub type SbusUartPin = super::peripherals::PIN_9;
    pub type SbusDmaChannel = super::peripherals::DMA_CH1;

    pub type I2cPeripheral = super::peripherals::I2C1;
    pub type I2cSdaPin = super::peripherals::PIN_2;
    pub type I2cSclPin = super::peripherals::PIN_3;

    pub type DshotPioPeripheral = super::peripherals::PIO0;
    pub type DshotPioM1Pin = super::peripherals::PIN_10;
    pub type DshotPioM2Pin = super::peripherals::PIN_13;
    pub type DshotPioM3Pin = super::peripherals::PIN_12;
    pub type DshotPioM4Pin = super::peripherals::PIN_11;

    super::bind_interrupts!(pub struct Irqs {
        UART1_IRQ => super::UartHandler<SbusUartPeripheral>;
        PIO0_IRQ_0 => super::PioHandler<DshotPioPeripheral>;
        I2C1_IRQ => super::I2CHandler<I2cPeripheral>;
    });
}

#[cfg(not(feature = "feather"))]
mod device_impl {
    pub type SbusUartPeripheral = super::peripherals::UART1;
    pub type SbusUartPin = super::peripherals::PIN_5;
    pub type SbusDmaChannel = super::peripherals::DMA_CH1;

    pub type I2cPeripheral = super::peripherals::I2C0;
    pub type I2cSdaPin = super::peripherals::PIN_0;
    pub type I2cSclPin = super::peripherals::PIN_1;

    pub type DshotPioPeripheral = super::peripherals::PIO0;
    pub type DshotPioM1Pin = super::peripherals::PIN_10;
    pub type DshotPioM2Pin = super::peripherals::PIN_20;
    pub type DshotPioM3Pin = super::peripherals::PIN_21;
    pub type DshotPioM4Pin = super::peripherals::PIN_11;

    super::bind_interrupts!(pub struct Irqs {
        UART1_IRQ => super::UartHandler<SbusUartPeripheral>;
        PIO0_IRQ_0 => super::PioHandler<DshotPioPeripheral>;
        I2C0_IRQ => super::I2CHandler<I2cPeripheral>;
    });
}

pub use device_impl::*;

pub struct Sbus {
    pub uart: SbusUartPeripheral,
    pub rx: SbusUartPin,
    pub dma: SbusDmaChannel,
}

pub struct I2c {
    pub i2c: I2cPeripheral,
    pub sda: I2cSdaPin,
    pub scl: I2cSclPin,
}

pub struct Dshot {
    pub pio: DshotPioPeripheral,
    pub m1: DshotPioM1Pin,
    pub m2: DshotPioM2Pin,
    pub m3: DshotPioM3Pin,
    pub m4: DshotPioM4Pin,
}

pub struct Device {
    pub rc: Sbus,
    pub imu: I2c,
    pub motors: Dshot,
    #[cfg(feature = "logging")]
    pub usb: embassy_rp::peripherals::USB,
}

impl Device {
    #[cfg(feature = "feather")]
    pub fn new(p: embassy_rp::Peripherals) -> Device {
        Device {
            rc: Sbus {
                uart: p.UART1,
                rx: p.PIN_9,
                dma: p.DMA_CH1,
            },
            imu: I2c {
                i2c: p.I2C1,
                sda: p.PIN_2,
                scl: p.PIN_3,
            },
            motors: Dshot {
                pio: p.PIO0,
                m1: p.PIN_10,
                m2: p.PIN_13,
                m3: p.PIN_12,
                m4: p.PIN_11,
            },

            #[cfg(feature = "logging")]
            usb: p.USB,
        }
    }
    #[cfg(not(feature = "feather"))]
    pub fn new(p: embassy_rp::Peripherals) -> Device {
        Device {
            rc: Sbus {
                uart: p.UART1,
                rx: p.PIN_5,
                dma: p.DMA_CH1,
            },
            imu: I2c {
                i2c: p.I2C0,
                sda: p.PIN_0,
                scl: p.PIN_1,
            },
            motors: Dshot {
                pio: p.PIO0,
                m1: p.PIN_10,
                m2: p.PIN_20,
                m3: p.PIN_21,
                m4: p.PIN_11,
            },

            #[cfg(feature = "logging")]
            usb: p.USB,
        }
    }
}
