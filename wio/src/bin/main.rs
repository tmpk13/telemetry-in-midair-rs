//! WIO-E5 firmware for the telemetry-in-midair board.
//!
//! - Reads the MAX-M10 GPS on USART1 (PB6 TX / PB7 RX, EXTINT on PB10).
//! - Broadcasts positions over the 915 MHz LoRa mesh (embedded-nano-mesh,
//!   so every node also repeats) and blinks D6 (PA9) on RX, D5 (PA10) on TX.
//! - Logs own and remote positions to a FAT SD card (SPI1 + PA0 CS).
//! - Talks to the ESP32-C6 on USART2 (PA2 TX / PA3 RX): positions and
//!   status out; sleep/config/firmware commands in. Radio-busy flags run
//!   both ways so the two radios can avoid transmitting at once.
//! - Radio parameters come from `RADIO.TOML` on the SD card and/or a
//!   config pushed over the link (which is also saved back to SD).
//! - Accepts firmware images over the link into the DFU partition; the
//!   swap bootloader (see `bootloader/`) installs them on reboot.

#![no_std]
#![no_main]
#![warn(clippy::large_stack_frames)]

use panic_halt as _;

#[macro_use]
extern crate wio_e5_gps;

#[rtic::app(device = stm32wlxx_hal::pac, dispatchers = [DAC])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);

    use midair_proto::link::{self, cmd, msg, Telemetry};
    use midair_proto::lora;
    use midair_proto::radiocfg::{self, RadioConfig};
    use rtt_target::{rprintln, rtt_init, set_print_channel};
    use stm32wlxx_hal::{
        gpio::{PortA, PortB},
        pac::{FLASH, IWDG},
        subghz::SubGhz,
    };
    use wio_e5_gps::cfgxfer::{CfgEvent, CfgTransfer};
    use wio_e5_gps::esplink::EspLink;
    use wio_e5_gps::fwupdate::{FwEvent, FwUpdate};
    use wio_e5_gps::gps::Gps;
    use wio_e5_gps::leds::Leds;
    use wio_e5_gps::platform::{self, SYSCLK_HZ};
    use wio_e5_gps::radio::Sx1262Driver;
    use wio_e5_gps::sdcard::SdCard;
    use wio_e5_gps::sdlog::SdLog;
    use wio_e5_gps::watchdog;
    use wio_e5_gps::{status_println, LoraIo, MeshNode, FIRMWARE_VERSION};

    /// `a` happened at or after deadline `b` in wrapping-u32 time.
    fn due(now: u32, deadline: u32) -> bool {
        now.wrapping_sub(deadline) < 0x8000_0000
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        io: LoraIo<Sx1262Driver>,
        mesh: MeshNode,
        gps: Gps,
        sdlog: SdLog,
        esp: EspLink,
        leds: Leds,
        flash: FLASH,
        iwdg: IWDG,
        fw: FwUpdate,
        cfgxfer: CfgTransfer,
        cfg: RadioConfig,
        cfg_loaded: bool,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, name: "Terminal" }
            }
        };
        set_print_channel(channels.up.0);
        rprintln!("wio-e5-gps v{} starting", FIRMWARE_VERSION);

        // DWT cycle counter drives platform::millis()/random().
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let dp = cx.device;
        let mut rcc = dp.RCC;

        // 16 MHz for SD SPI throughput; before the monotonic starts.
        platform::raise_sysclk(&mut rcc);
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let mut flash = dp.FLASH;

        // Watchdog first: a hang anywhere in init resets us, and the
        // bootloader reverts unconfirmed firmware.
        let iwdg = dp.IWDG;
        watchdog::start(&iwdg, 6_000);
        wio_e5_gps::boot_state::confirm_boot(&mut flash);

        let gpioa = PortA::split(dp.GPIOA, &mut rcc);
        let gpiob = PortB::split(dp.GPIOB, &mut rcc);

        // SD card + FAT (optional; retries in the loop when absent).
        let mut sdlog = cortex_m::interrupt::free(|cs| {
            SdLog::new(SdCard::new(
                dp.SPI1, gpiob.b3, gpiob.b4, gpiob.b5, gpioa.a0, &mut rcc, cs,
            ))
        });
        watchdog::feed(&iwdg);
        sdlog.poll(platform::millis());
        watchdog::feed(&iwdg);

        // Radio config: SD file if present, defaults otherwise.
        let mut cfg_buf = [0u8; wio_e5_gps::sdlog::CONFIG_MAX];
        let (cfg, cfg_loaded) = match sdlog.read_config(&mut cfg_buf) {
            Some(n) => match radiocfg::parse_bytes(&cfg_buf[..n]) {
                Ok(c) => {
                    rprintln!("Config: RADIO.TOML loaded (address {})", c.address);
                    (c, true)
                }
                Err(e) => {
                    rprintln!("Config: RADIO.TOML invalid ({:?}), using defaults", e);
                    (RadioConfig::default(), false)
                }
            },
            None => {
                rprintln!("Config: no SD config, using defaults");
                (RadioConfig::default(), false)
            }
        };

        // SubGHz radio (integrated SX1262).
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let mut radio = Sx1262Driver::new(sg);
        radio.init(&cfg);
        radio.print_diagnostics();
        watchdog::feed(&iwdg);

        // GPS on USART1, ESP link on USART2, activity LEDs.
        let (gps, esp, leds) = cortex_m::interrupt::free(|cs| {
            (
                Gps::new(dp.USART1, gpiob.b6, gpiob.b7, gpiob.b10, &mut rcc, cs),
                EspLink::new(dp.USART2, gpioa.a2, gpioa.a3, &mut rcc, cs),
                Leds::new(gpioa.a9, gpioa.a10, cs),
            )
        });

        let io = LoraIo::new(radio);
        let mesh = MeshNode::new(cfg.address, cfg.listen_ms);
        rprintln!("Mesh node {} ready", cfg.address);

        run::spawn().unwrap();

        (
            Shared {},
            Local {
                io,
                mesh,
                gps,
                sdlog,
                esp,
                leds,
                flash,
                iwdg,
                fw: FwUpdate::new(),
                cfgxfer: CfgTransfer::new(),
                cfg,
                cfg_loaded,
            },
        )
    }

    #[task(local = [io, mesh, gps, sdlog, esp, leds, flash, iwdg, fw, cfgxfer, cfg, cfg_loaded], priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;
        let gps = cx.local.gps;
        let sdlog = cx.local.sdlog;
        let esp = cx.local.esp;
        let leds = cx.local.leds;
        let flash = cx.local.flash;
        let iwdg = cx.local.iwdg;
        let fw = cx.local.fw;
        let cfgxfer = cx.local.cfgxfer;
        let cfg = cx.local.cfg;
        let cfg_loaded = cx.local.cfg_loaded;

        let mut sleeping = false;
        let mut tx_count: u32 = 0;
        let mut rx_count: u32 = 0;
        // Track the GPS fix state so only its transitions are announced.
        let mut had_fix = false;

        // Position report to the ESP at most once a second (the GPS fix
        // rate); the LoRa beacon runs on its own configured interval.
        let mut next_esp_pos: u32 = 0;
        let mut next_beacon: u32 = platform::millis()
            .wrapping_add(cfg.address as u32 * 1_000)
            .wrapping_add(2_000);
        let mut next_status: u32 = platform::millis().wrapping_add(3_000);
        // While set, we flagged our radio busy to the ESP; clear at this time.
        let mut busy_clear_at: Option<u32> = None;

        status_println!(esp, "wio v{} up, node {}", FIRMWARE_VERSION, cfg.address);

        loop {
            let now = platform::millis();
            leds.update(now);

            // ---- ESP link: drain and handle frames -----------------------
            while let Some((cmd_id, _len)) = esp.poll() {
                match cmd_id {
                    cmd::PING => {
                        esp.send_ack(cmd::PING, FIRMWARE_VERSION);
                    }
                    cmd::RADIO_BUSY => {
                        let busy = esp.payload().first() == Some(&1);
                        esp.set_peer_busy(busy, now);
                    }
                    cmd::WIO_SLEEP => {
                        let sleep = esp.payload().first() == Some(&1);
                        if sleep && !sleeping {
                            io.inner().standby();
                            sleeping = true;
                            status_println!(esp, "soft sleep");
                        } else if !sleep && sleeping {
                            io.inner().init(cfg);
                            sleeping = false;
                            status_println!(esp, "woke from soft sleep");
                        }
                        esp.send_ack(cmd::WIO_SLEEP, sleep as u16);
                    }
                    cmd::GPS_SLEEP => {
                        let sleep = esp.payload().first() == Some(&1);
                        if sleep {
                            gps.sleep();
                        } else {
                            gps.wake();
                        }
                        esp.send_ack(cmd::GPS_SLEEP, sleep as u16);
                    }
                    cmd::CFG_BEGIN => match cfgxfer.begin(esp.payload()) {
                        CfgEvent::Ack(seq) => esp.send_ack(cmd::CFG_BEGIN, seq),
                        CfgEvent::Error(e) => esp.send_nak(cmd::CFG_BEGIN, e),
                        CfgEvent::Complete => unreachable!(),
                    },
                    cmd::CFG_DATA => match cfgxfer.data(esp.payload()) {
                        CfgEvent::Ack(seq) => esp.send_ack(cmd::CFG_DATA, seq),
                        CfgEvent::Error(e) => esp.send_nak(cmd::CFG_DATA, e),
                        CfgEvent::Complete => unreachable!(),
                    },
                    cmd::CFG_END => match cfgxfer.end(esp.payload()) {
                        CfgEvent::Complete => match radiocfg::parse_bytes(cfgxfer.bytes()) {
                            Ok(new_cfg) => {
                                let remesh = new_cfg.address != cfg.address
                                    || new_cfg.listen_ms != cfg.listen_ms;
                                *cfg = new_cfg;
                                io.inner().init(cfg);
                                if remesh {
                                    *mesh = MeshNode::new(cfg.address, cfg.listen_ms);
                                }
                                *cfg_loaded = true;
                                // Best effort - the SD card is optional.
                                if !sdlog.write_config(now, cfgxfer.bytes()) {
                                    rprintln!("Config applied; SD save failed");
                                }
                                status_println!(esp, "config applied, node {}", cfg.address);
                                esp.send_ack(cmd::CFG_END, 0);
                            }
                            Err(_) => esp.send_nak(cmd::CFG_END, link::err::BAD_CONFIG),
                        },
                        CfgEvent::Ack(seq) => esp.send_ack(cmd::CFG_END, seq),
                        CfgEvent::Error(e) => esp.send_nak(cmd::CFG_END, e),
                    },
                    cmd::FW_BEGIN => match fw.begin(esp.payload()) {
                        FwEvent::Ack(seq) => {
                            status_println!(esp, "fw update: receiving image");
                            esp.send_ack(cmd::FW_BEGIN, seq);
                        }
                        FwEvent::Error(e) => esp.send_nak(cmd::FW_BEGIN, e),
                        FwEvent::Complete => unreachable!(),
                    },
                    cmd::FW_DATA => {
                        watchdog::feed(iwdg);
                        match fw.data(esp.payload(), flash) {
                            FwEvent::Ack(seq) => esp.send_ack(cmd::FW_DATA, seq),
                            FwEvent::Error(e) => esp.send_nak(cmd::FW_DATA, e),
                            FwEvent::Complete => unreachable!(),
                        }
                    }
                    cmd::FW_END => {
                        watchdog::feed(iwdg);
                        match fw.end(flash) {
                            FwEvent::Complete => {
                                esp.send_ack(cmd::FW_END, 0);
                                rprintln!("Rebooting into bootloader for swap");
                                cortex_m::peripheral::SCB::sys_reset();
                            }
                            FwEvent::Error(e) => esp.send_nak(cmd::FW_END, e),
                            FwEvent::Ack(_) => unreachable!(),
                        }
                    }
                    cmd::FW_ABORT => {
                        fw.abort();
                        esp.send_ack(cmd::FW_ABORT, 0);
                    }
                    _ => {}
                }
            }

            // A firmware transfer owns the loop: skip GPS/SD/mesh work so
            // the link stays responsive and nothing else erases flash.
            if fw.is_active() {
                watchdog::feed(iwdg);
                Mono::delay(1_u32.millis()).await;
                continue;
            }

            if sleeping {
                // Soft sleep: only the ESP link stays alive (for WAKE).
                watchdog::feed(iwdg);
                Mono::delay(50_u32.millis()).await;
                continue;
            }

            // ---- GPS ------------------------------------------------------
            gps.poll();
            let fix = gps.has_fix();
            if fix != had_fix {
                had_fix = fix;
                if fix {
                    status_println!(esp, "gps fix acquired ({} sats)", gps.packet().sats);
                } else {
                    status_println!(esp, "gps fix lost");
                }
            }
            if gps.take_updated() && due(now, next_esp_pos) {
                next_esp_pos = now.wrapping_add(1_000);
                let packet = gps.packet();
                let mut buf = [0u8; 3 + 20];
                buf[0] = 0; // src: local
                buf[1..3].copy_from_slice(&0i16.to_le_bytes());
                buf[3..].copy_from_slice(&packet.encode());
                esp.send(msg::POSITION, &buf);
                if gps.has_fix() {
                    sdlog.log_position(now, 0, 0, &packet);
                }
            }

            // ---- LoRa position beacon --------------------------------------
            if cfg.beacon_interval_s != 0 && due(now, next_beacon) {
                if gps.has_fix() && !esp.peer_busy(now) {
                    let data = lora::encode_position(&gps.packet());
                    // Warn the ESP off the air while the mesh transmits.
                    esp.send(msg::RADIO_BUSY, &[1]);
                    busy_clear_at = Some(now.wrapping_add(2 * cfg.listen_ms + 500));
                    match mesh.broadcast(&data, cfg.lifetime) {
                        Ok(()) => tx_count += 1,
                        Err(e) => debug_println!("Beacon TX failed: {:?}", e),
                    }
                    let jitter = platform::random(0, 2_000) as u32;
                    next_beacon = now
                        .wrapping_add(cfg.beacon_interval_s as u32 * 1_000)
                        .wrapping_add(jitter);
                } else {
                    // No fix or ESP busy: check again shortly.
                    next_beacon = now.wrapping_add(500);
                }
            }
            if let Some(t) = busy_clear_at
                && due(now, t) {
                    esp.send(msg::RADIO_BUSY, &[0]);
                    busy_clear_at = None;
                }

            // ---- Mesh -------------------------------------------------------
            mesh.update(io, platform::millis());
            if let Some(m) = mesh.receive() {
                rx_count += 1;
                let rssi = io.last_rssi();
                if let Some(p) = lora::decode_position(&m.data) {
                    debug_println!("Position from node {} rssi={}", m.source, rssi);
                    let mut buf = [0u8; 3 + 20];
                    buf[0] = m.source;
                    buf[1..3].copy_from_slice(&rssi.to_le_bytes());
                    buf[3..].copy_from_slice(&p.encode());
                    esp.send(msg::POSITION, &buf);
                    sdlog.log_position(now, m.source, rssi, &p);
                } else {
                    // Forward other payloads verbatim (truncated to one frame).
                    let mut buf = [0u8; 3 + 32];
                    let n = m.data.len().min(32);
                    buf[0] = m.source;
                    buf[1..3].copy_from_slice(&rssi.to_le_bytes());
                    buf[3..3 + n].copy_from_slice(&m.data[..n]);
                    esp.send(msg::LORA_RX, &buf[..3 + n]);
                }
            }

            // ---- Periodic status to the ESP ---------------------------------
            if due(now, next_status) {
                next_status = now.wrapping_add(5_000);
                let secs_since_rx = match io.last_rx_ms() {
                    Some(t) => {
                        let s = platform::millis().wrapping_sub(t) / 1000;
                        s.min(0xFFFE) as u16
                    }
                    None => 0xFFFF,
                };
                let mut flags = 0u8;
                if sdlog.ready() {
                    flags |= link::TELEM_FLAG_SD_OK;
                }
                if gps.has_fix() {
                    flags |= link::TELEM_FLAG_GPS_FIX;
                }
                if *cfg_loaded {
                    flags |= link::TELEM_FLAG_CFG_LOADED;
                }
                let telem = Telemetry {
                    last_rssi: io.last_rssi(),
                    last_snr_cb: io.inner().last_snr_cb(),
                    secs_since_rx,
                    rx_count,
                    tx_count,
                    flags,
                    sats: gps.packet().sats,
                };
                esp.send(msg::STATUS, &telem.encode());
            }

            // ---- SD housekeeping --------------------------------------------
            sdlog.poll(now);

            watchdog::feed(iwdg);
            Mono::delay(1_u32.millis()).await;
        }
    }
}
