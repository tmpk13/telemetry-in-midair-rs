//! Backup copy of `RADIO.CFG` in internal flash.
//!
//! The SD card is the primary store and the one a person can edit, but it is
//! optional hardware: a board with no card, or one whose card is disabled or
//! has failed, would otherwise come back from every power cycle on the
//! firmware defaults - losing its address, which is the one setting that
//! cannot be guessed back.
//!
//! So a config that is applied is written here as well, and at boot the
//! order is SD, then flash, then defaults. The card wins when both exist,
//! because pulling it and editing the file on a computer should do what it
//! looks like it does.
//!
//! # Layout
//!
//! Page 122 (`0x0803_D000`), inside the region past the boot state that the
//! app's linker script already excludes:
//!
//! ```text
//! [0..4]   magic 0x4746_4331
//! [4..8]   length of the config text
//! [8..12]  CRC-32 of the config text
//! [12..16] padding, so the text starts 8-byte aligned
//! [16..]   config text
//! ```
//!
//! The CRC is what makes a half-written page (power lost mid-erase, mid-
//! program) read back as "no config" rather than as garbage that the TOML
//! parser then has to reject.

use midair_proto::link::crc32;
use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};
use stm32wlxx_hal::pac;

/// Page index holding the config, one past the boot state.
const CFG_PAGE_IDX: u8 = 122;

/// Base address of that page.
const CFG_ADDR: u32 = 0x0803_D000;

/// Marks a written config. Changing this invalidates every stored copy,
/// which is the upgrade path if the layout ever changes.
const MAGIC: u32 = 0x4746_4331;

/// Header bytes before the config text.
const HEADER_LEN: usize = 16;

/// Largest config text this page can hold. The transfer limit
/// ([`crate::cfgxfer::CONFIG_MAX`]) is well under it.
pub const MAX_LEN: usize = 2048 - HEADER_LEN;

fn read_u32(offset: usize) -> u32 {
    // The page is plain readable flash; a volatile read keeps the compiler
    // from caching it across the write that may follow.
    unsafe { core::ptr::read_volatile((CFG_ADDR as usize + offset) as *const u32) }
}

fn stored() -> Option<&'static [u8]> {
    if read_u32(0) != MAGIC {
        return None;
    }
    let len = read_u32(4) as usize;
    if len == 0 || len > MAX_LEN {
        return None;
    }
    // SAFETY: within the page, which is mapped flash, and `len` is bounded
    // above by what the page can hold.
    let bytes =
        unsafe { core::slice::from_raw_parts((CFG_ADDR as usize + HEADER_LEN) as *const u8, len) };
    (crc32(bytes) == read_u32(8)).then_some(bytes)
}

/// Copy the stored config into `buf`, returning its length.
///
/// `None` when nothing is stored, the CRC does not match, or `buf` is too
/// small - all of which mean the caller should fall back rather than act on
/// a partial config.
pub fn read(buf: &mut [u8]) -> Option<usize> {
    let bytes = stored()?;
    if bytes.len() > buf.len() {
        return None;
    }
    buf[..bytes.len()].copy_from_slice(bytes);
    Some(bytes.len())
}

/// Whether the stored config is already exactly `bytes`.
pub fn matches(bytes: &[u8]) -> bool {
    stored() == Some(bytes)
}

/// Store `bytes` as the config, returning whether it was written.
///
/// A config identical to what is already there is not rewritten: this page
/// is erased in full for every write, and the endurance budget is better
/// spent on real changes than on re-pushing the same file.
pub fn write(flash_periph: &mut pac::FLASH, bytes: &[u8]) -> bool {
    if bytes.is_empty() || bytes.len() > MAX_LEN {
        return false;
    }
    if matches(bytes) {
        return true;
    }

    let mut header = [0xFFu8; HEADER_LEN];
    header[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    header[4..8].copy_from_slice(&(bytes.len() as u32).to_le_bytes());
    header[8..12].copy_from_slice(&crc32(bytes).to_le_bytes());

    let mut flash = Flash::unlock(flash_periph);
    let page = unsafe { Page::from_index_unchecked(CFG_PAGE_IDX) };
    if unsafe { flash.page_erase(page) }.is_err() {
        return false;
    }
    // Text first, then the header: until the magic and CRC land, a power
    // loss leaves an erased page that reads as "nothing stored" rather than
    // a valid-looking header over half-written text.
    let text_addr = unsafe { AlignedAddr::new_unchecked(CFG_ADDR as usize + HEADER_LEN) };
    if unsafe { flash.program_bytes(bytes, text_addr) }.is_err() {
        return false;
    }
    let head_addr = unsafe { AlignedAddr::new_unchecked(CFG_ADDR as usize) };
    unsafe { flash.program_bytes(&header, head_addr) }.is_ok()
    // Flash is locked on drop of `flash`.
}

/// Erase the stored config, so the next boot falls back to SD or defaults.
pub fn clear(flash_periph: &mut pac::FLASH) -> bool {
    let mut flash = Flash::unlock(flash_periph);
    let page = unsafe { Page::from_index_unchecked(CFG_PAGE_IDX) };
    unsafe { flash.page_erase(page) }.is_ok()
}
