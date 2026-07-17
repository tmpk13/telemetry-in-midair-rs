//! ESP32-C6 firmware for the telemetry-in-midair board.
//!
//! The C6 is the BLE face of the board: it serves the gps-proto GATT
//! service (so the existing gps-gui-rs app connects unchanged) extended
//! with telemetry, remote-position and bulk-transfer characteristics
//! (midair-proto). Position and status data come from the WIO-E5 over
//! UART0 (GPIO16 TX / GPIO17 RX) and are cached, so a freshly connected
//! central immediately receives the latest fix, LoRa RSSI and timestamps.
//!
//! Controls, all over BLE config writes:
//! - GPS/LoRa power rail (AP2112K LDO enable on GPIO2)
//! - WIO soft sleep / GPS backup mode (forwarded over the link; a stuck
//!   WIO is woken with a reset pulse on GPIO6)
//! - ESP deep sleep with a periodic wake-check: the C6 sleeps whenever no
//!   central is connected, waking every interval to advertise briefly
//! - Radio TOML config and WIO firmware images pushed through the bulk
//!   characteristic and streamed over the UART link
//!
//! LED D2 (GPIO3): double blink on a sleep-interval wake, short burst on
//! config writes from the phone, fast toggling during a firmware upload.
//!
//! Radio coordination: while the WIO flags a LoRa transmission the
//! notifier skips its ticks; while a bulk transfer runs the C6 flags
//! itself busy so the WIO defers its beacon.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::{Cell, RefCell};

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig, RtcPin};
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx, UartTx};
use esp_hal::Async;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use gps_proto::packet::{self, PositionPacket};
use midair_proto::ble;
use midair_proto::link::{self, cmd, msg, FrameBuf, FrameParser, Telemetry};
use trouble_host::prelude::*;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

/// How long to advertise on a sleep-mode wake check before going back to
/// deep sleep.
const WAKE_ADV_WINDOW_S: u64 = 15;
/// Linger after a disconnect (sleep mode active) so the phone can come
/// straight back before the C6 vanishes.
const SLEEP_LINGER_S: u64 = 5;

// ---------------------------------------------------------------------------
// Cached state (UART link task writes, BLE session samples)
// ---------------------------------------------------------------------------

static GPS_STATE: Mutex<CriticalSectionRawMutex, Cell<PositionPacket>> =
    Mutex::new(Cell::new(PositionPacket {
        lat_e7: 0,
        lon_e7: 0,
        alt_dm: 0,
        speed_cms: 0,
        course_cdeg: 0,
        flags: 0,
        sats: 0,
        tod_ms: 0,
    }));

static REMOTE_STATE: Mutex<CriticalSectionRawMutex, Cell<[u8; ble::REMOTE_LEN]>> =
    Mutex::new(Cell::new([0; ble::REMOTE_LEN]));

static TELEM_STATE: Mutex<CriticalSectionRawMutex, Cell<Telemetry>> = Mutex::new(Cell::new(
    Telemetry {
        last_rssi: 0,
        last_snr_cb: 0,
        secs_since_rx: 0xFFFF,
        rx_count: 0,
        tx_count: 0,
        flags: 0,
        sats: 0,
    },
));

/// Position notify interval, set over BLE. Survives disconnects but not
/// resets.
static NOTIFY_INTERVAL_MS: Mutex<CriticalSectionRawMutex, Cell<u32>> =
    Mutex::new(Cell::new(packet::UPDATE_INTERVAL_DEFAULT_MS));

/// `Instant::as_millis` deadline until which the WIO's radio is busy;
/// 0 = not busy. The BLE notifier defers its ticks while this is set.
static WIO_BUSY_UNTIL: Mutex<CriticalSectionRawMutex, Cell<u64>> = Mutex::new(Cell::new(0));

fn wio_busy() -> bool {
    WIO_BUSY_UNTIL.lock(|c| c.get()) > Instant::now().as_millis()
}

// ---------------------------------------------------------------------------
// Settings that survive deep sleep (RTC RAM with a magic for cold boots)
// ---------------------------------------------------------------------------

const PERSIST_MAGIC: u32 = 0x6D69_6461; // "mida"
const PFLAG_PWR_OFF: u32 = 1 << 0;
const PFLAG_WIO_SLEEP: u32 = 1 << 1;
const PFLAG_GPS_SLEEP: u32 = 1 << 2;

#[derive(Clone, Copy)]
struct Persist {
    sleep_interval_s: u32,
    flags: u32,
}

// These live in RTC fast RAM and are not reinitialized on a deep-sleep
// wake; the magic word gates cold-boot garbage. esp-hal's Persistable
// marker only covers atomics and primitives, hence three statics.
use portable_atomic::{AtomicU32 as PersistU32, Ordering as PersistOrdering};
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_MAGIC_WORD: PersistU32 = PersistU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_INTERVAL: PersistU32 = PersistU32::new(0);
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static PERSIST_FLAGS: PersistU32 = PersistU32::new(0);

fn persist_get() -> Persist {
    if PERSIST_MAGIC_WORD.load(PersistOrdering::Relaxed) == PERSIST_MAGIC {
        Persist {
            sleep_interval_s: PERSIST_INTERVAL.load(PersistOrdering::Relaxed),
            flags: PERSIST_FLAGS.load(PersistOrdering::Relaxed),
        }
    } else {
        Persist {
            sleep_interval_s: 0,
            flags: 0,
        }
    }
}

fn persist_update(f: impl FnOnce(&mut Persist)) {
    let mut p = persist_get();
    f(&mut p);
    PERSIST_INTERVAL.store(p.sleep_interval_s, PersistOrdering::Relaxed);
    PERSIST_FLAGS.store(p.flags, PersistOrdering::Relaxed);
    PERSIST_MAGIC_WORD.store(PERSIST_MAGIC, PersistOrdering::Relaxed);
}

// ---------------------------------------------------------------------------
// Power / reset pins shared with the BLE handlers
// ---------------------------------------------------------------------------

static PWR_PIN: Mutex<CriticalSectionRawMutex, RefCell<Option<Output<'static>>>> =
    Mutex::new(RefCell::new(None));
static RST_PIN: Mutex<CriticalSectionRawMutex, RefCell<Option<Output<'static>>>> =
    Mutex::new(RefCell::new(None));

fn set_pwr_en(on: bool) {
    PWR_PIN.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            if on {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
    });
    persist_update(|p| {
        if on {
            p.flags &= !PFLAG_PWR_OFF;
        } else {
            p.flags |= PFLAG_PWR_OFF;
        }
    });
}

/// Hard-reset the WIO-E5 (NRST low pulse through the open-drain GPIO6).
async fn pulse_wio_reset() {
    RST_PIN.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            pin.set_low();
        }
    });
    Timer::after(Duration::from_millis(20)).await;
    RST_PIN.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            pin.set_high();
        }
    });
}

// ---------------------------------------------------------------------------
// LED D2 (GPIO3)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum Blink {
    /// Two blinks: woke up on the sleep interval.
    Wake,
    /// Short burst: info/config received from the phone.
    Info,
    /// One immediate toggle: firmware chunk forwarded (fast blink while
    /// chunks stream).
    FwToggle,
    /// Force the LED off (end of firmware upload).
    Off,
}

static LED_CHANNEL: Channel<CriticalSectionRawMutex, Blink, 8> = Channel::new();

fn blink(b: Blink) {
    let _ = LED_CHANNEL.try_send(b);
}

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        match LED_CHANNEL.receive().await {
            Blink::Wake => {
                for _ in 0..2 {
                    led.set_high();
                    Timer::after(Duration::from_millis(60)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(60)).await;
                }
            }
            Blink::Info => {
                for _ in 0..3 {
                    led.set_high();
                    Timer::after(Duration::from_millis(25)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(25)).await;
                }
            }
            Blink::FwToggle => led.toggle(),
            Blink::Off => led.set_low(),
        }
    }
}

// ---------------------------------------------------------------------------
// UART link to the WIO-E5
// ---------------------------------------------------------------------------

struct OutFrame {
    cmd: u8,
    payload: heapless::Vec<u8, 200>,
}

static OUT_CHANNEL: Channel<CriticalSectionRawMutex, OutFrame, 8> = Channel::new();

/// (is_ack, acked cmd, value-or-err) from the last ACK/NAK frame.
static ACK_SIGNAL: Signal<CriticalSectionRawMutex, (bool, u8, u16)> = Signal::new();

fn queue_frame(cmd: u8, payload: &[u8]) {
    let mut v = heapless::Vec::new();
    if v.extend_from_slice(payload).is_ok() {
        let _ = OUT_CHANNEL.try_send(OutFrame { cmd, payload: v });
    }
}

/// Send a command and wait for the WIO's ACK/NAK. Returns the ack value
/// or the midair-proto ble ack status to report.
async fn wio_request(cmd: u8, payload: &[u8], timeout_ms: u64) -> Result<u16, u8> {
    ACK_SIGNAL.reset();
    let mut v = heapless::Vec::new();
    v.extend_from_slice(payload).map_err(|_| ble::ACK_BAD_STATE)?;
    OUT_CHANNEL.send(OutFrame { cmd, payload: v }).await;
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left == Duration::from_ticks(0) {
            return Err(ble::ACK_WIO_TIMEOUT);
        }
        match with_timeout(left, ACK_SIGNAL.wait()).await {
            Ok((true, c, value)) if c == cmd => return Ok(value),
            Ok((false, c, _err)) if c == cmd => return Err(ble::ACK_WIO_ERROR),
            Ok(_) => continue, // ack for something else: keep waiting
            Err(_) => return Err(ble::ACK_WIO_TIMEOUT),
        }
    }
}

/// Handle one complete frame from the WIO.
fn handle_link_frame(cmd_id: u8, payload: &[u8]) {
    match cmd_id {
        msg::POSITION => {
            if payload.len() < 3 + packet::POSITION_PACKET_LEN {
                return;
            }
            let src = payload[0];
            if src == 0 {
                if let Some(p) = PositionPacket::decode(&payload[3..]) {
                    GPS_STATE.lock(|c| c.set(p));
                }
            } else {
                let mut buf = [0u8; ble::REMOTE_LEN];
                buf.copy_from_slice(&payload[..ble::REMOTE_LEN]);
                REMOTE_STATE.lock(|c| c.set(buf));
            }
        }
        msg::STATUS => {
            if let Some(t) = Telemetry::decode(payload) {
                TELEM_STATE.lock(|c| c.set(t));
            }
        }
        msg::RADIO_BUSY => {
            let busy = payload.first() == Some(&1);
            let until = if busy {
                Instant::now().as_millis() + link::RADIO_BUSY_TIMEOUT_MS as u64
            } else {
                0
            };
            WIO_BUSY_UNTIL.lock(|c| c.set(until));
        }
        msg::LORA_RX => {
            // Non-position mesh traffic: just log it on the console.
            println!("lora rx from {} ({} bytes)", payload.first().unwrap_or(&0), payload.len().saturating_sub(3));
        }
        link::resp::ACK => {
            if payload.len() >= 3 {
                let value = u16::from_le_bytes([payload[1], payload[2]]);
                ACK_SIGNAL.signal((true, payload[0], value));
            }
        }
        link::resp::NAK
            if payload.len() >= 2 => {
                ACK_SIGNAL.signal((false, payload[0], payload[1] as u16));
            }
        _ => {}
    }
}

#[embassy_executor::task]
async fn link_task(mut rx: UartRx<'static, Async>, mut tx: UartTx<'static, Async>) {
    let mut parser = FrameParser::new();
    let mut out = FrameBuf::new();
    let mut buf = [0u8; 64];
    loop {
        match select(rx.read_async(&mut buf), OUT_CHANNEL.receive()).await {
            Either::First(Ok(n)) => {
                for &b in &buf[..n] {
                    if parser.feed(b) {
                        let f = parser.frame();
                        // Copy out so the parser can be fed again next loop.
                        let mut p = [0u8; link::MAX_PAYLOAD];
                        let len = f.payload.len();
                        p[..len].copy_from_slice(f.payload);
                        handle_link_frame(f.cmd, &p[..len]);
                    }
                }
            }
            Either::First(Err(_)) => {
                // RX error (overrun/noise): the frame parser resyncs on
                // the next valid frame by CRC.
            }
            Either::Second(frame) => {
                out.build(frame.cmd, &frame.payload);
                use embedded_io_async::Write as _;
                let _ = tx.write_all(out.as_bytes()).await;
                let _ = tx.flush_async().await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------

#[gatt_server]
struct Server {
    gps: GpsService,
}

/// The gps-proto service, extended with the midair characteristics. UUIDs
/// come from the shared crates so firmware and app cannot drift.
#[gatt_service(uuid = packet::SERVICE_UUID_U128)]
struct GpsService {
    /// Position packets in the gps-proto wire format (local GPS).
    #[characteristic(uuid = packet::POSITION_UUID_U128, read, notify)]
    position: [u8; packet::POSITION_PACKET_LEN],
    /// Config commands: [id, len, value bytes].
    #[characteristic(uuid = packet::CONFIG_UUID_U128, write)]
    config: heapless::Vec<u8, 8>,
    /// Config/bulk acks: [id, status, applied value].
    #[characteristic(uuid = packet::ACK_UUID_U128, notify)]
    ack: [u8; packet::ACK_MAX_LEN],
    /// Link/radio telemetry in the midair-proto wire format.
    #[characteristic(uuid = ble::TELEMETRY_UUID_U128, read, notify)]
    telemetry: [u8; link::TELEMETRY_LEN],
    /// Bulk transfer ops (radio TOML / WIO firmware).
    #[characteristic(uuid = ble::BULK_UUID_U128, write)]
    bulk: heapless::Vec<u8, 200>,
    /// Last remote position heard over LoRa: [src, rssi i16le, packet].
    #[characteristic(uuid = ble::REMOTE_UUID_U128, read, notify)]
    remote: [u8; ble::REMOTE_LEN],
}

// Default app descriptor required by the esp-idf 2nd-stage bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The C6's reclaimed boot-RAM region is exactly 64 KiB.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let persist = persist_get();
    let woke_from_sleep = matches!(
        esp_hal::system::wakeup_cause(),
        esp_hal::system::SleepSource::Timer
    );

    // Power rail first so the WIO/GPS state matches what was configured
    // before the deep sleep; then release the deep-sleep pad holds.
    let pwr_level = if persist.flags & PFLAG_PWR_OFF != 0 {
        Level::Low
    } else {
        Level::High
    };
    let pwr = Output::new(peripherals.GPIO2, pwr_level, OutputConfig::default());
    let rst = Output::new(
        peripherals.GPIO6,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );
    unsafe {
        // The Output drivers above own the pins; the holds from the last
        // deep sleep must be released after reconfiguration.
        esp_hal::peripherals::GPIO2::steal().rtcio_pad_hold(false);
        esp_hal::peripherals::GPIO6::steal().rtcio_pad_hold(false);
    }
    PWR_PIN.lock(|p| p.borrow_mut().replace(pwr));
    RST_PIN.lock(|p| p.borrow_mut().replace(rst));

    let led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    spawner.spawn(led_task(led)).expect("spawn led task");
    if woke_from_sleep {
        blink(Blink::Wake);
    }

    let mut rtc = Rtc::new(peripherals.LPWR);

    // WIO-E5 link on UART0 (GPIO16 TX / GPIO17 RX). Console output stays
    // on the USB Serial/JTAG port.
    let uart_config = UartConfig::default().with_baudrate(link::BAUD);
    let uart = Uart::new(peripherals.UART0, uart_config)
        .expect("uart init")
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();
    spawner
        .spawn(link_task(uart_rx, uart_tx))
        .expect("spawn link task");

    let radio = esp_radio::init().expect("radio init");
    let transport =
        BleConnector::new(&radio, peripherals.BT, Default::default()).expect("ble connector");
    let controller = ExternalController::<_, 20>::new(transport);

    // Fixed static-random address (two MSBs set) for a stable identity.
    let address = Address::random([0x47, 0x50, 0x53, 0xa1, 0xc6, 0xff]);
    println!("BLE address: {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: ble::DEVICE_NAME,
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .expect("gatt server");

    select(
        ble_host_task(&mut runner),
        serve_task(&mut peripheral, &server, &mut rtc),
    )
    .await;
    unreachable!()
}

/// Drive the BLE host stack.
async fn ble_host_task<C: Controller, P: PacketPool>(runner: &mut Runner<'_, C, P>) {
    loop {
        if runner.run().await.is_err() {
            println!("ble host error, restarting");
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

/// Put the board to deep sleep for the configured interval. GPIO2 (power
/// rail) and GPIO6 (WIO reset) are pad-held so the WIO keeps running and
/// logging while the C6 sleeps.
fn enter_deep_sleep(rtc: &mut Rtc<'_>, interval_s: u32) -> ! {
    println!("deep sleep for {} s", interval_s);
    unsafe {
        esp_hal::peripherals::GPIO2::steal().rtcio_pad_hold(true);
        esp_hal::peripherals::GPIO6::steal().rtcio_pad_hold(true);
    }
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(interval_s as u64));
    rtc.sleep_deep(&[&timer])
}

/// Advertise, accept one central at a time, serve it until disconnect.
/// With sleep mode active, a bounded advertising window ends in deep
/// sleep instead of advertising forever.
async fn serve_task<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    server: &Server<'_>,
    rtc: &mut Rtc<'_>,
) {
    let mut adv_data = [0u8; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[packet::SERVICE_UUID_U128.to_le_bytes()]),
        ],
        &mut adv_data,
    )
    .expect("adv data fits");
    let mut scan_data = [0u8; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(ble::DEVICE_NAME.as_bytes())],
        &mut scan_data,
    )
    .expect("scan data fits");

    // First window after boot: with sleep mode on, stay reachable for the
    // advertising window and then sleep.
    loop {
        let sleep_interval = persist_get().sleep_interval_s;
        println!("advertising as {}", ble::DEVICE_NAME);
        let advertiser = match peripheral
            .advertise(
                &AdvertisementParameters::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..adv_len],
                    scan_data: &scan_data[..scan_len],
                },
            )
            .await
        {
            Ok(a) => a,
            Err(_) => {
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        let conn = if sleep_interval > 0 {
            match with_timeout(Duration::from_secs(WAKE_ADV_WINDOW_S), advertiser.accept()).await
            {
                Ok(Ok(c)) => c,
                Ok(Err(_)) => continue,
                Err(_) => {
                    // Window expired with nobody interested.
                    enter_deep_sleep(rtc, sleep_interval);
                }
            }
        } else {
            match advertiser.accept().await {
                Ok(c) => c,
                Err(_) => continue,
            }
        };

        let conn = match conn.with_attribute_server(server) {
            Ok(c) => c,
            Err(_) => continue,
        };
        println!("central connected");
        gatt_session(&conn, server).await;
        println!("central disconnected");

        // Sleep mode: give the central a moment to reconnect, then sleep.
        let sleep_interval = persist_get().sleep_interval_s;
        if sleep_interval > 0 {
            Timer::after(Duration::from_secs(SLEEP_LINGER_S)).await;
            enter_deep_sleep(rtc, sleep_interval);
        }
    }
}

/// One in-flight bulk transfer (TOML config or WIO firmware).
struct BulkState {
    kind: u8,
    crc32: u32,
    total: u32,
    received: u32,
    next_seq: u16,
}

/// Handle one connection: GATT events plus the notify ticker.
async fn gatt_session<P: PacketPool>(conn: &GattConnection<'_, '_, P>, server: &Server<'_>) {
    let events = async {
        let mut bulk: Option<BulkState> = None;
        loop {
            match conn.next().await {
                GattConnectionEvent::Disconnected { reason: _ } => break,
                GattConnectionEvent::Gatt { event } => {
                    let mut ack: Option<([u8; packet::ACK_MAX_LEN], usize)> = None;
                    if let GattEvent::Write(write) = &event {
                        if write.handle() == server.gps.config.handle {
                            ack = Some(apply_config(write.data()).await);
                            blink(Blink::Info);
                        } else if write.handle() == server.gps.bulk.handle {
                            ack = Some(handle_bulk(write.data(), &mut bulk).await);
                        }
                    }
                    // Accepting lets the attribute server process the
                    // request (reads answered from the table, writes
                    // stored to it).
                    if let Ok(reply) = event.accept() {
                        reply.send().await;
                    }
                    // The ack confirms the setting/step actually applied.
                    if let Some((ack, len)) = ack {
                        let _ = server.gps.ack.notify(conn, &ack).await;
                        let _ = len;
                    }
                }
                _ => {}
            }
        }
        // Session over: make sure a dangling transfer does not leave the
        // WIO waiting or the busy flag set.
        if bulk.is_some() {
            queue_frame(cmd::FW_ABORT, &[]);
            queue_frame(cmd::RADIO_BUSY, &[0]);
            blink(Blink::Off);
        }
    };

    let notifier = async {
        let mut first = true;
        loop {
            if first {
                // Push the cached data right away so a fresh central sees
                // the latest fix/telemetry without waiting an interval
                // (notify also refreshes the read-property values).
                first = false;
                Timer::after(Duration::from_millis(200)).await;
            } else {
                let interval = NOTIFY_INTERVAL_MS.lock(|c| c.get());
                Timer::after(Duration::from_millis(u64::from(interval))).await;
            }
            // Defer discretionary notifications while the LoRa radio
            // transmits (both radios share the power budget).
            while wio_busy() {
                Timer::after(Duration::from_millis(100)).await;
            }
            let pos = GPS_STATE.lock(|c| c.get()).encode();
            if server.gps.position.notify(conn, &pos).await.is_err() {
                break;
            }
            let telem = TELEM_STATE.lock(|c| c.get()).encode();
            let _ = server.gps.telemetry.notify(conn, &telem).await;
            let remote = REMOTE_STATE.lock(|c| c.get());
            // src 0 means nothing heard yet.
            if remote[0] != 0 {
                let _ = server.gps.remote.notify(conn, &remote).await;
            }
        }
    };

    // Either arm ending (disconnect / notify failure) ends the session.
    select(events, notifier).await;
}

/// Apply a config write and build the ack to send back.
async fn apply_config(data: &[u8]) -> ([u8; packet::ACK_MAX_LEN], usize) {
    // Board-specific ids first; gps-proto ids as the fallback.
    if data.len() >= 2 {
        let id = data[0];
        let len = data[1] as usize;
        let value = data.get(2..2 + len).unwrap_or(&[]);
        match id {
            ble::CFG_PWR_EN => {
                let on = value.first().copied().unwrap_or(1) != 0;
                set_pwr_en(on);
                println!("config: power rail {}", if on { "on" } else { "off" });
                return packet::encode_ack(id, packet::ACK_OK, &[on as u8]);
            }
            ble::CFG_WIO_SLEEP => {
                let sleep = value.first().copied().unwrap_or(0) != 0;
                persist_update(|p| {
                    if sleep {
                        p.flags |= PFLAG_WIO_SLEEP;
                    } else {
                        p.flags &= !PFLAG_WIO_SLEEP;
                    }
                });
                let result = wio_request(cmd::WIO_SLEEP, &[sleep as u8], 500).await;
                if result.is_err() && !sleep {
                    // Wake fallback: hard reset brings it back awake.
                    println!("config: wio wake timed out, pulsing reset");
                    pulse_wio_reset().await;
                    return packet::encode_ack(id, packet::ACK_OK, &[0]);
                }
                return match result {
                    Ok(_) => packet::encode_ack(id, packet::ACK_OK, &[sleep as u8]),
                    Err(status) => packet::encode_ack(id, status, &[]),
                };
            }
            ble::CFG_GPS_SLEEP => {
                let sleep = value.first().copied().unwrap_or(0) != 0;
                persist_update(|p| {
                    if sleep {
                        p.flags |= PFLAG_GPS_SLEEP;
                    } else {
                        p.flags &= !PFLAG_GPS_SLEEP;
                    }
                });
                return match wio_request(cmd::GPS_SLEEP, &[sleep as u8], 500).await {
                    Ok(_) => packet::encode_ack(id, packet::ACK_OK, &[sleep as u8]),
                    Err(status) => packet::encode_ack(id, status, &[]),
                };
            }
            ble::CFG_ESP_SLEEP_S => {
                if let Ok(bytes) = <[u8; 4]>::try_from(value) {
                    let mut secs = u32::from_le_bytes(bytes);
                    if secs > 0 {
                        secs = secs.clamp(ble::ESP_SLEEP_MIN_S, ble::ESP_SLEEP_MAX_S);
                    }
                    persist_update(|p| p.sleep_interval_s = secs);
                    println!("config: esp sleep interval {} s", secs);
                    return packet::encode_ack(id, packet::ACK_OK, &secs.to_le_bytes());
                }
                return packet::encode_ack(id, packet::ACK_BAD_VALUE, &[]);
            }
            _ => {}
        }
    }

    match packet::parse_config(data) {
        Ok(packet::ConfigCommand::UpdateIntervalMs(ms)) => {
            let applied = packet::clamp_interval(ms);
            NOTIFY_INTERVAL_MS.lock(|c| c.set(applied));
            println!("config: notify interval set to {} ms", applied);
            packet::encode_ack(
                packet::CFG_UPDATE_INTERVAL_MS,
                packet::ACK_OK,
                &applied.to_le_bytes(),
            )
        }
        Err(status) => {
            let id = data.first().copied().unwrap_or(0);
            println!("config: rejected write (status {})", status);
            packet::encode_ack(id, status, &[])
        }
    }
}

/// Handle one bulk characteristic write, forwarding to the WIO.
async fn handle_bulk(
    data: &[u8],
    bulk: &mut Option<BulkState>,
) -> ([u8; packet::ACK_MAX_LEN], usize) {
    let nak = |status: u8| packet::encode_ack(ble::ACK_ID_BULK, status, &[]);
    let Some(&op) = data.first() else {
        return nak(packet::ACK_BAD_VALUE);
    };
    match op {
        ble::OP_BEGIN => {
            if data.len() < 12 {
                return nak(packet::ACK_BAD_VALUE);
            }
            let kind = data[1];
            let total = u32::from_le_bytes(data[2..6].try_into().unwrap());
            let crc32 = u32::from_le_bytes(data[6..10].try_into().unwrap());
            let version = u16::from_le_bytes(data[10..12].try_into().unwrap());
            let result = match kind {
                ble::KIND_TOML => {
                    if total == 0 || total > 1024 {
                        return nak(packet::ACK_BAD_VALUE);
                    }
                    wio_request(cmd::CFG_BEGIN, &(total as u16).to_le_bytes(), 1000).await
                }
                ble::KIND_FIRMWARE => {
                    let mut p = [0u8; 10];
                    p[0..4].copy_from_slice(&total.to_le_bytes());
                    p[4..8].copy_from_slice(&crc32.to_le_bytes());
                    p[8..10].copy_from_slice(&version.to_le_bytes());
                    wio_request(cmd::FW_BEGIN, &p, 1000).await
                }
                _ => return nak(packet::ACK_BAD_VALUE),
            };
            match result {
                Ok(_) => {
                    // Keep the LoRa side quiet during the transfer.
                    queue_frame(cmd::RADIO_BUSY, &[1]);
                    println!("bulk: begin kind={} total={}", kind, total);
                    *bulk = Some(BulkState {
                        kind,
                        crc32,
                        total,
                        received: 0,
                        next_seq: 0,
                    });
                    packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_OK, &0u32.to_le_bytes())
                }
                Err(status) => nak(status),
            }
        }
        ble::OP_DATA => {
            let Some(state) = bulk.as_mut() else {
                return nak(ble::ACK_BAD_STATE);
            };
            if data.len() < 4 || data.len() - 3 > ble::BULK_DATA_MAX {
                return nak(packet::ACK_BAD_VALUE);
            }
            let seq = u16::from_le_bytes(data[1..3].try_into().unwrap());
            let chunk = &data[3..];
            if seq != state.next_seq {
                // Duplicate after a lost ack: re-ack; anything else is fatal.
                if seq.wrapping_add(1) == state.next_seq {
                    return packet::encode_ack(
                        ble::ACK_ID_BULK,
                        packet::ACK_OK,
                        &(state.next_seq as u32).to_le_bytes(),
                    );
                }
                return nak(packet::ACK_BAD_VALUE);
            }
            let link_cmd = if state.kind == ble::KIND_FIRMWARE {
                blink(Blink::FwToggle);
                cmd::FW_DATA
            } else {
                cmd::CFG_DATA
            };
            let mut p = [0u8; 2 + link::DATA_CHUNK];
            p[0..2].copy_from_slice(&seq.to_le_bytes());
            p[2..2 + chunk.len()].copy_from_slice(chunk);
            // Refresh the busy flag so it cannot expire mid-transfer.
            queue_frame(cmd::RADIO_BUSY, &[1]);
            match wio_request(link_cmd, &p[..2 + chunk.len()], 2000).await {
                Ok(next) => {
                    state.next_seq = next;
                    state.received += chunk.len() as u32;
                    packet::encode_ack(
                        ble::ACK_ID_BULK,
                        packet::ACK_OK,
                        &(next as u32).to_le_bytes(),
                    )
                }
                Err(status) => {
                    *bulk = None;
                    queue_frame(cmd::RADIO_BUSY, &[0]);
                    blink(Blink::Off);
                    nak(status)
                }
            }
        }
        ble::OP_END => {
            let Some(state) = bulk.take() else {
                return nak(ble::ACK_BAD_STATE);
            };
            queue_frame(cmd::RADIO_BUSY, &[0]);
            blink(Blink::Off);
            if state.received != state.total {
                return nak(packet::ACK_BAD_VALUE);
            }
            let result = if state.kind == ble::KIND_FIRMWARE {
                // CRC check over 112 KB of flash takes a moment.
                wio_request(cmd::FW_END, &[], 5000).await
            } else {
                wio_request(cmd::CFG_END, &state.crc32.to_le_bytes(), 2000).await
            };
            println!("bulk: end -> {:?}", result);
            match result {
                Ok(_) => packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_OK, &[]),
                Err(status) => nak(status),
            }
        }
        ble::OP_ABORT => {
            if bulk.take().is_some() {
                queue_frame(cmd::FW_ABORT, &[]);
                queue_frame(cmd::RADIO_BUSY, &[0]);
                blink(Blink::Off);
            }
            packet::encode_ack(ble::ACK_ID_BULK, packet::ACK_OK, &[])
        }
        _ => nak(packet::ACK_BAD_VALUE),
    }
}
