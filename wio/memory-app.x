/* STM32WLE5JCIx — Seeed Wio-E5 (LoRa-E5)
 *
 * Flash layout (256 KB = 128 pages of 2 KB):
 *   Bootloader   pages  0– 7   16 KB  0x0800_0000
 *   ACTIVE (app) pages  8–63  112 KB  0x0800_4000  ← this binary
 *   DFU staging  pages 64–120 114 KB  0x0802_0000
 *   Boot state   page  121      2 KB  0x0803_C800
 *   Radio config page  122      2 KB  0x0803_D000  (see src/cfgstore.rs)
 *   Spare        pages 123–127 10 KB  0x0803_D800
 */
MEMORY
{
    FLASH : ORIGIN = 0x08004000, LENGTH = 112K
    RAM   : ORIGIN = 0x20000000, LENGTH = 64K
}
