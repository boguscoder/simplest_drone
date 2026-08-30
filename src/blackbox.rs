use crate::{
    arming::{ARMED, DISARMED},
    consts::{
        BBOX_BUFFER_SIZE, BBOX_TELE_DIVISOR, FLASH_TELE_SIZE, FLASH_TOTAL_SIZE, TELE_FRAME_SIZE,
    },
    device::FlashDmaChannel,
    device::Irqs,
    rc::RcData,
    switch::SwitchingPolicy,
    telemetry::{BBOX_CHANNEL, TELE_MODE, USB_CHANNEL},
};
use drone_consts::telemetry::*;
use embassy_futures::select::{Either3, select3};
use embassy_rp::{
    Peri,
    flash::{Async, ERASE_SIZE, Flash, PAGE_SIZE},
    peripherals::FLASH,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_storage_async::nor_flash::NorFlash;
use portable_atomic::Ordering;

pub static FLUSH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static DUMP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct BlackBoxSwitch;

impl SwitchingPolicy for BlackBoxSwitch {
    type SafetyContext = ();

    const NAME: &'static str = "BBOX_FLUSH";

    const ON_SIGNAL: Option<&'static Signal<CriticalSectionRawMutex, ()>> = Some(&FLUSH_SIGNAL);

    #[inline(always)]
    fn want_on(rc: &RcData) -> bool {
        rc.bbox_flush() > 0.5
    }

    #[inline(always)]
    fn want_off(rc: &RcData) -> bool {
        !BlackBoxSwitch::want_on(rc)
    }

    #[inline(always)]
    fn force_off(_rc: &RcData, _noop: ()) -> bool {
        false
    }
}

static mut RAM_BUFFER: [u8; BBOX_BUFFER_SIZE] = [0; BBOX_BUFFER_SIZE];

unsafe extern "C" {
    static __end_block_addr: u8;
}

pub struct FlashLogger<'d> {
    flash: Flash<'d, FLASH, Async, FLASH_TOTAL_SIZE>,
    flash_offset: usize,
    ram_buffer: &'static mut [u8],
    ram_cursor: usize,
}

impl<'d> FlashLogger<'d> {
    pub fn new(flash: Peri<'d, FLASH>, dma: Peri<'d, FlashDmaChannel>) -> Self {
        let flash_base = 0x10000000;
        let binary_end = unsafe { &__end_block_addr as *const u8 as usize };
        let flash_tele_start = ((binary_end - flash_base) + (ERASE_SIZE - 1)) & !(ERASE_SIZE - 1);
        let flash = Flash::new(flash, dma, Irqs);
        Self {
            flash,
            flash_offset: flash_tele_start,
            ram_buffer: unsafe { &mut *core::ptr::addr_of_mut!(RAM_BUFFER) },
            ram_cursor: 0,
        }
    }

    pub fn clear_cache(&mut self) {
        self.ram_cursor = 0;
    }

    pub fn cache_frame(&mut self, frame: &[u8]) -> bool {
        let end = self.ram_cursor + frame.len();
        if end > self.ram_buffer.len() {
            return false;
        }

        self.ram_buffer[self.ram_cursor..end].copy_from_slice(frame);
        self.ram_cursor = end;
        true
    }

    pub async fn commit_to_flash(&mut self) {
        if self.ram_cursor == 0 {
            log::warn!("Nothing to flush to flash");
            return;
        }

        let bytes_to_write = self.ram_cursor;
        let data_flash_start = self.flash_offset + ERASE_SIZE;
        let total_sectors_needed =
            ERASE_SIZE + ((bytes_to_write + (ERASE_SIZE - 1)) & !(ERASE_SIZE - 1));
        let erase_end = self.flash_offset + total_sectors_needed;

        log::info!(
            "Committing {} bytes to flash. Erasing 0x{:X} to 0x{:X}",
            bytes_to_write,
            self.flash_offset,
            erase_end
        );

        if let Err(e) = self
            .flash
            .erase(self.flash_offset as u32, erase_end as u32)
            .await
        {
            log::error!("Flash erase failed: {:?}", e);
            return;
        }

        let chunks = self.ram_buffer[..bytes_to_write].chunks(PAGE_SIZE);
        let addresses = (data_flash_start..).step_by(PAGE_SIZE);

        for (chunk, write_addr) in chunks.zip(addresses) {
            let mut page_buf = [0xFF; PAGE_SIZE];
            page_buf[..chunk.len()].copy_from_slice(chunk);

            if let Err(e) = self.flash.write(write_addr as u32, &page_buf).await {
                log::error!("Flash write failed at offset 0x{:X}: {:?}", write_addr, e);
                break;
            }
        }

        let mut header_buf = [0u8; PAGE_SIZE];
        header_buf[0..4].copy_from_slice(b"BBOX");
        header_buf[4..8].copy_from_slice(&(bytes_to_write as u32).to_le_bytes());

        if let Err(e) = self
            .flash
            .write(self.flash_offset as u32, &header_buf)
            .await
        {
            log::error!("Failed to write flash header: {:?}", e);
            return;
        }

        log::info!("Flash commit complete! Saved {} bytes.", bytes_to_write);
    }

    pub async fn dump_to_channel(&mut self) {
        let mut read_buf = [0u8; PAGE_SIZE];

        if let Err(e) = self
            .flash
            .read(self.flash_offset as u32, &mut read_buf)
            .await
        {
            log::error!("Failed to read flash header: {:?}", e);
            return;
        }

        if &read_buf[0..4] != b"BBOX" {
            log::error!("No valid blackbox data found in flash (bad magic header).");
            return;
        }

        let total_bytes = u32::from_le_bytes(read_buf[4..8].try_into().unwrap()) as usize;
        if total_bytes == 0 || total_bytes > FLASH_TELE_SIZE {
            log::error!(
                "Invalid payload size found in flash header: {}",
                total_bytes
            );
            return;
        }

        log::info!(
            "Found valid blackbox log! Streaming {} bytes...",
            total_bytes
        );

        const READ_CHUNK_SIZE: usize = TELE_FRAME_SIZE * 6;
        let data_flash_start = self.flash_offset + ERASE_SIZE;
        let mut bytes_sent = 0usize;

        for offset in (0..total_bytes).step_by(READ_CHUNK_SIZE) {
            let current_addr = data_flash_start + offset;
            let chunk_size = (total_bytes - offset).min(READ_CHUNK_SIZE);
            let pad_len = (chunk_size + 3) & !3;

            match self
                .flash
                .read(current_addr as u32, &mut read_buf[..pad_len])
                .await
            {
                Ok(()) => {
                    for frame_slice in read_buf[..chunk_size].chunks_exact(TELE_FRAME_SIZE) {
                        if let Ok(array_chunk) = frame_slice.try_into() {
                            USB_CHANNEL.send(array_chunk).await;
                            bytes_sent += TELE_FRAME_SIZE;
                        }
                    }
                }
                Err(e) => {
                    log::error!("Flash read FAILED at 0x{:X}: {:?}", current_addr, e);
                    break;
                }
            }
        }

        log::info!(
            "Dump complete! Sent {} bytes ({} frames) to USB.",
            bytes_sent,
            bytes_sent / TELE_FRAME_SIZE
        );
    }
}

#[embassy_executor::task]
pub async fn flash_logger_task(mut logger: FlashLogger<'static>) {
    let receiver = BBOX_CHANNEL.receiver();

    loop {
        match select3(ARMED.wait(), DUMP_SIGNAL.wait(), FLUSH_SIGNAL.wait()).await {
            Either3::First(()) => {
                DUMP_SIGNAL.try_take();
                FLUSH_SIGNAL.try_take();
                DISARMED.try_take();

                log::info!("Starting IMU dump to ring buffer in RAM");

                logger.clear_cache();

                if TELE_MODE
                    .compare_exchange(
                        Mode::None as u8,
                        Mode::Imu as u8,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    )
                    .is_err()
                {
                    log::warn!("Telemetry tool seems to be ON, pick None and rearm to record IMU");
                    continue;
                }

                let mut frame_count = 0;

                while !DISARMED.signaled() {
                    let frame = receiver.receive().await;
                    if frame_count % BBOX_TELE_DIVISOR == 0 && !logger.cache_frame(&frame) {
                        log::info!("Stopped IMU dump, disarmed or out of memory");
                        break;
                    }
                    frame_count += 1;
                }

                TELE_MODE.store(Mode::None as u8, Ordering::Release);
            }
            Either3::Second(()) => {
                logger.dump_to_channel().await;
            }
            Either3::Third(()) => {
                logger.commit_to_flash().await;
            }
        }
    }
}
