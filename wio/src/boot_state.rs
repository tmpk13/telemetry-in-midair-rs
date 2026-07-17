//! Read/write helpers for the bootloader state partition.
//!
//! The boot state lives at page 121 (`0x0803_C800`).  The first 4 bytes are a
//! magic word indicating the current state; the next 4 bytes track swap
//! progress (used only by the bootloader itself).

use core::ptr;

use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};
use stm32wlxx_hal::pac;

/// Base address of the boot-state page.
const STATE_ADDR: u32 = 0x0803_C800;

/// Page index for the boot-state page.
const STATE_PAGE_IDX: u8 = 121;

// Magic values (must match the bootloader).
pub const BOOT_OK: u32 = 0x4F4B_4F4B;
pub const SWAP_PENDING: u32 = 0x5357_4150;

/// Read the current boot-state magic word.
pub fn read() -> u32 {
    unsafe { ptr::read_volatile(STATE_ADDR as *const u32) }
}

/// Write a new boot-state magic word.
///
/// Erases the state page first, then programs the 8-byte double-word
/// (state + 0xFFFF_FFFF padding for the progress field).
pub fn write(flash_periph: &mut pac::FLASH, state: u32) {
    let mut flash = Flash::unlock(flash_periph);
    let page = unsafe { Page::from_index_unchecked(STATE_PAGE_IDX) };

    // Erase state page
    unsafe { flash.page_erase(page).ok() };

    // Program 8 bytes: state word + 0xFFFFFFFF (no progress from app side)
    let data: [u8; 8] = {
        let mut buf = [0xFFu8; 8];
        buf[0..4].copy_from_slice(&state.to_le_bytes());
        buf
    };
    let addr = unsafe { AlignedAddr::new_unchecked(STATE_ADDR as usize) };
    unsafe { flash.program_bytes(&data, addr) }.ok();
    // Flash is locked on drop of `flash`.
}

/// Mark the current firmware as successfully booted.
///
/// Call this from the application after confirming the system is healthy
/// (e.g., radio initialised).  If this is never called before a reset,
/// the bootloader will revert to the previous firmware.
pub fn confirm_boot(flash: &mut pac::FLASH) {
    let current = read();
    // Only write if not already BOOT_OK — avoid unnecessary flash wear.
    if current != BOOT_OK {
        write(flash, BOOT_OK);
    }
}

/// Signal the bootloader to swap the DFU image into the ACTIVE slot on
/// next reboot.
pub fn request_swap(flash: &mut pac::FLASH) {
    write(flash, SWAP_PENDING);
}
