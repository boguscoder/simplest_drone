use nalgebra::Vector3;

// --- System & Hardware ---
pub const TICK_HZ: u64 = 2000;
pub const CYCLE_TIME: f32 = 1.0 / TICK_HZ as f32;
pub const BARO_HZ: u64 = 50;
pub const SBUS_BAUD: u32 = 100_000;
pub const I2C_FREQ: u32 = 1_000_000;
pub const IMU_I2C_ADDR: u8 = 0x69;
#[cfg(feature = "telemetry")]
pub const IMU_HZ: u64 = TICK_HZ / 2;
#[cfg(not(feature = "telemetry"))]
pub const IMU_HZ: u64 = TICK_HZ;
pub const IMU_CYCLE_TIME: f32 = 1.0 / IMU_HZ as f32;

// --- Telemetry ---
#[cfg(feature = "telemetry")]
pub mod tele_consts {
    pub const USB_VID: u16 = 0xc0de;
    pub const USB_PID: u16 = 0xbabe;
    pub const TELE_MAX_VALUES: usize = 9;
    pub const TELE_FRAME_SIZE: usize = 2 + TELE_MAX_VALUES * 4;
    pub const FLASH_TELE_SIZE: usize = 1024 * 1024;
    pub const FLASH_TOTAL_SIZE: usize = 2048 * 1024;
    pub const BBOX_BUFFER_SIZE: usize = 475_000;
    pub const BBOX_TELE_DIVISOR: usize = 4;
}

#[cfg(feature = "telemetry")]
pub use tele_consts::*;

// --- RC & Input ---
pub const RC_MIN: u16 = 240;
pub const RC_MAX: u16 = 1807;
pub const ARM_HOLD_TICKS: u64 = 1000;
pub const DISARM_HOLD_TICKS: u64 = 100;
pub const IMU_STALE_MS: u64 = 50;
pub const BARO_STALE_MS: u64 = 200;
pub const SWITCH_FOLLOWERS: usize = 2;

// --- Tuning ---
pub const MAX_POWER: f32 = 0.45;
const THROTTLE_HEADROOM: f32 = 0.05;
pub const THROTTLE_STICK_MAX: f32 = MAX_POWER - THROTTLE_HEADROOM;
pub const THROTTLE_MIN: f32 = 48.0;
const THROTTLE_MAX: f32 = 2047.0;
pub const SLOPE: f32 = THROTTLE_MAX - THROTTLE_MIN;
pub const YAW_RATE: f32 = 200.0 * core::f32::consts::PI / 180.0;
pub const MAX_LEAN_ANGLE: f32 = 45.0 * core::f32::consts::PI / 180.0;
pub const ANGLE_P_GAIN: f32 = 5.0;
pub const RATE_FILTER_CUTOFF_HZ: f32 = 100.0;
pub const D_FILTER_CUTOFF_HZ: f32 = 25.0;
pub const I_TERM_THROTTLE_LIMIT: f32 = 0.1;
pub const AHRS_BETA: f32 = 0.05;

pub const KP_FIXED: f32 = 0.065;

pub const KI_FIXED: f32 = 0.12;

pub const KD_FIXED: f32 = 0.0003;

pub const YAW_KP_FIXED: f32 = 0.08;
pub const YAW_KI_FIXED: f32 = KI_FIXED;
pub const YAW_KD_FIXED: f32 = 0.0;

pub const PID_LIMIT_MIN: f32 = -0.4;
pub const PID_LIMIT_MAX: f32 = 0.4;

pub const ALT_KP_MIN: f32 = 0.0;
pub const ALT_KP_MAX: f32 = 0.5;
pub const ALT_KI_FIXED: f32 = 0.05;
pub const ALT_KD_MIN: f32 = 0.0;
pub const ALT_KD_MAX: f32 = 0.05;
pub const ALT_HOLD_THROTTLE_MIN: f32 = 0.15;

// --- IMU ---
pub const CALIBRATION_TICKS: usize = 2000;
pub const ACC_OFFSET: Vector3<f32> = Vector3::new(-0.09, 0.03, 0.0);
pub const ACC_SCALE: Vector3<f32> = Vector3::new(0.993833, 0.998219, 0.990074);
