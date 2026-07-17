# telemetry-in-midair-rs
[Kicad Board](https://github.com/tmpk13/telemetry-in-midair) https://github.com/tmpk13/telemetry-in-midair

GPS tracker board firmware: a WIO-E5 (STM32WLE5) reads a MAX-M10 GPS,
broadcasts positions over a 915 MHz LoRa mesh and logs to SD, while an
ESP32-C6 serves everything over BLE to the gps-gui-rs app and manages
power. See `PLAN.md` for the intent.

## Layout

| Directory | What | Target |
|-|-|-|
| `proto/` | Shared no_std protocol crate: ESP<->WIO UART link framing, LoRa payloads, BLE extensions, `radio.toml` parser. Host-testable (`cargo test`). | any |
| `wio/` | WIO-E5 application firmware (RTIC). | `thumbv7em-none-eabi` (nightly) |
| `wio/bootloader/` | Two-partition swap bootloader for UART-fed firmware updates. | `thumbv7em-none-eabi` |
| `esp/` | ESP32-C6 firmware (embassy + trouble BLE). | `riscv32imac-unknown-none-elf` (stable) |
| `tools/` | Host uploader (Python/pixi) to flash the WIO through the ESP USB. | host |

Depends on the sibling repo `../gps-proto` for the BLE position protocol
and NMEA parsing (shared with `../esp32c3-gps` and `../gps-gui-rs`).

## Build and flash

```sh
# protocol tests (host)
cd proto && cargo test

# WIO-E5: bootloader once, then the app (SWD via probe-rs)
cd wio && cargo run --release -p bootloader   # no RTT output; Ctrl-C once flashed
cd wio && cargo run --release                 # app, RTT console

# ESP32-C6 (USB Serial/JTAG; console also lives there)
cd esp && cargo run --release

# bench tool: check over UART whether the WIO is alive (powers the rail,
# pulses reset, probes factory AT firmware at 9600 and the link PING at
# 115200; results on the USB console)
cd esp && cargo run --release --bin wio-probe
```

Note the WIO only has power while the ESP drives the LDO enable
(GPIO2) high - flash the ESP first or SWD/UART on the WIO will see a
dead chip.

`FW_VERSION=n` at build time stamps the WIO firmware version reported
over the link (used for update bookkeeping).

## Radio configuration

The WIO loads `RADIO.TOML` from the SD card at boot; the same file can be
pushed over BLE (bulk characteristic) at runtime, which also rewrites the
SD copy. All keys are optional; defaults in parentheses:

```toml
[radio]
frequency_hz = 915000000   # (915 MHz)
spreading_factor = 7       # 5-12 (7)
bandwidth_khz = 125        # 62|125|250|500 (125)
coding_rate = 5            # 4/5..4/8 (5)
power_dbm = 22             # -9..22 (22)

[mesh]
address = 1                # 1-255 (1)
listen_ms = 200            # (200)
lifetime = 2               # broadcast hop count; >=2 repeats (2)

[beacon]
interval_s = 10            # position broadcast period, 0 = off (10)
```

Raise `listen_ms` together with slow presets (SF12 etc.) - the listen
window must exceed one packet's air time.

## BLE

Same service UUID as the ESP32-C3 beacon, so gps-gui-rs discovers it
unchanged (device name `GPS-C6`). On top of the gps-proto position /
config / ack characteristics the C6 adds telemetry (LoRa RSSI/SNR,
counters, SD + fix flags), the last remote node position, a status/log
characteristic (notify + read), and a bulk write characteristic for TOML
config and WIO firmware images.

## Status updates

The WIO-E5 sends human-readable status lines to the ESP over the UART link
(`msg::LOG`) on notable events - boot, GPS fix acquired/lost, soft
sleep/wake, config applied, firmware receive. The ESP prints each to its
USB console (prefixed `wio:`) and notifies it on the status/log
characteristic, so gps-gui-rs (or any BLE client) sees the same live log.
Lines are ASCII, up to `link::LOG_MAX` (64) bytes.

Config command ids (config characteristic, `[id, len, value]`):

| Id | Value | Effect |
|-|-|-|
| `0x01` | u32 ms | position notify interval (gps-proto) |
| `0x10` | u8 0/1 | GPS + LoRa power rail (LDO) off/on |
| `0x11` | u8 0/1 | WIO soft sleep (reset-pulse fallback on wake) |
| `0x12` | u8 0/1 | GPS backup mode (UBX-RXM-PMREQ / EXTINT wake) |
| `0x13` | u32 s | ESP deep-sleep wake-check interval, 0 = off |

With a sleep interval set, the C6 deep-sleeps whenever no central is
connected and wakes every interval to advertise for 15 s (double D2
blink). The power rail and WIO reset pins are pad-held through sleep, so
the WIO keeps logging.

## SD card

Normal FAT16/32 card. `GPSLOG.CSV` gets one line per own/remote fix
(`ms,src,lat_e7,lon_e7,alt_dm,speed_cms,course_cdeg,sats,fix,rssi`);
readable in any spreadsheet. The card is optional and hot-pluggable.

## WIO firmware update

Build the raw image (objcopy of the ELF):

```sh
cd wio && cargo objcopy --release -- -O binary wio-e5-gps.bin
```

Either path streams it over the UART link into the WIO's DFU partition (D2
blinks rapidly); on a verified CRC the WIO reboots and the swap bootloader
installs it power-fail-safely, reverting automatically if the new image
never confirms boot. SWD via the J5 header remains as the recovery path.

**Over BLE:** push the `.bin` through the bulk characteristic (`OP_BEGIN`
kind 2 with size/crc32/version, `OP_DATA` chunks up to 192 bytes,
`OP_END`).

**Over the ESP USB:** the same bulk protocol is exposed on the USB
Serial/JTAG port (framed with the link framing, `link::usb` commands), so a
computer can flash the WIO through the ESP with no BLE. A host uploader
lives in `tools/`:

```sh
cd tools && pixi run fw-upload --file ../wio/wio-e5-gps.bin
```

The ESP console shares the USB port; the uploader's frame parser resyncs
past the console text. Only one transfer (BLE or USB) runs at a time.

## ESP32-C6
`ESP32-C6-MINI-1U-H4`
`4MB Flash`

| Pin | Function |
|-|-|
| I03 | LED D2 |
| IO2 | PWR EN GPS/Radio (AP2112K-3.3) |
| IO4 | RX/GPIO |
| IO5 | TX/GPIO |
| IO6 | WIO-E5 RST |
| RXD0 | WIO-E5 PA2 |
| TXD0 | WIO-E5 PA3 |
| IO12 | USB D- |
| IO13 | USB D+ |

*Boot pad on back*

## WIO-E5

| Pin | Function |
|-|-|
| PB6 (TX) | GPS RX |
| PB7 (RX) | GPS TX |
| PB10 | EXT INT GPS |
| PC1 | I2C SCL (JST SH) |
| PC1 | I2C SDA (JST SH) |
| PB3 | SD SCK |
| PB4 | SD CITO |
| PB5 | SD COTI |
| PA0 | SD CS |
| PA9 | LED D6 |
| PA10 | LED D5 |

*Reset (RST) pad on back*


## Connectors
#### JST SH
*As of Version 1*

**I2C** *(J6)*

| Pin | Function |
|-|-|
| 4 | SCL |
| 3 | SDA |
| 2 | 3V3 |
| 1 | GND |

**SWD** *(J5)*

| Pin | Function |
|-|-|
| 4 | SWDIO |
| 3 | SWDCLK |
| 2 | 3V3 |
| 1 | GND |


Inital WIO wipe:
`openocd -f interface/cmsis-dap.cfg -f target/stm32wlx.cfg -c "init; reset halt; stm32l4x unlock 0; reset halt; exit"`



# Known Issues

Pin 28 on WIO should be NC
SMA footprint

Maybe need a larger capacitor on WIO-E5 input? Check this.