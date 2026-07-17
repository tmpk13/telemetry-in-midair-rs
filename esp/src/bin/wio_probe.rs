//! WIO-E5 aliveness probe (bench tool, not the shipping firmware).
//!
//! Flash with `cargo run --release --bin wio-probe` and watch the USB
//! Serial/JTAG console. It powers the GPS/LoRa rail, releases the WIO
//! reset line, pulses reset once to provoke a boot banner, then loops:
//!
//! - 9600 baud: prints anything the WIO says and sends `AT` - the Seeed
//!   factory AT firmware answers `+AT: OK` here (USART2 is its console).
//! - 115200 baud: sends the midair-proto link PING - the wio-e5-gps
//!   firmware acks with its version.
//!
//! Any response at all proves the module has power and is executing
//! firmware; which probe answers tells you which firmware is on it.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::Async;
use esp_println::println;
use midair_proto::link::{self, cmd, FrameBuf, FrameParser};

esp_bootloader_esp_idf::esp_app_desc!();

/// Collect and print everything the WIO sends within `window`.
/// Returns the bytes received (up to the buffer size).
async fn listen(
    uart: &mut Uart<'_, Async>,
    window: Duration,
    label: &str,
) -> heapless::Vec<u8, 256> {
    let mut collected: heapless::Vec<u8, 256> = heapless::Vec::new();
    let deadline = Instant::now() + window;
    let mut buf = [0u8; 64];
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left == Duration::from_ticks(0) {
            break;
        }
        match with_timeout(left, uart.read_async(&mut buf)).await {
            Ok(Ok(n)) if n > 0 => {
                for &b in &buf[..n] {
                    let _ = collected.push(b);
                }
            }
            Ok(_) => {}
            Err(_) => break, // window over
        }
    }
    if !collected.is_empty() {
        print_bytes(label, &collected);
    }
    collected
}

fn print_bytes(label: &str, bytes: &[u8]) {
    println!("{} ({} bytes):", label, bytes.len());
    for chunk in bytes.chunks(16) {
        let mut ascii: heapless::String<16> = heapless::String::new();
        for &b in chunk {
            let _ = ascii.push(if (0x20..0x7f).contains(&b) { b as char } else { '.' });
        }
        println!("  {:02x?}  |{}|", chunk, ascii);
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // Rail on, reset released (open drain, high = not driving).
    let _pwr = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    let mut rst = Output::new(
        peripherals.GPIO6,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );
    let mut led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    let mut uart = Uart::new(peripherals.UART0, UartConfig::default().with_baudrate(9600))
        .expect("uart init")
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .into_async();

    println!("wio-probe: rail on (GPIO2 high), reset released (GPIO6)");
    Timer::after(Duration::from_millis(300)).await;

    // One reset pulse: a factory AT firmware prints a boot banner.
    println!("wio-probe: pulsing WIO reset, listening at 9600 for a banner");
    rst.set_low();
    Timer::after(Duration::from_millis(20)).await;
    rst.set_high();
    listen(&mut uart, Duration::from_millis(1500), "boot banner @9600").await;

    let mut round: u32 = 0;
    loop {
        round += 1;
        led.toggle();
        println!("--- probe round {} ---", round);

        // Factory AT firmware check at 9600.
        uart.apply_config(&UartConfig::default().with_baudrate(9600))
            .expect("apply 9600");
        let _ = uart.write_async(b"AT\r\n").await;
        let _ = uart.flush_async().await;
        let resp = listen(&mut uart, Duration::from_millis(700), "AT response @9600").await;
        let mut any_response = !resp.is_empty();
        if !resp.is_empty() {
            if resp.windows(2).any(|w| w == b"OK") {
                println!("=> WIO alive: factory Seeed AT firmware (answers AT at 9600)");
            } else {
                println!("=> WIO is sending bytes at 9600 (alive, unidentified firmware)");
            }
        }

        // wio-e5-gps link check at 115200.
        uart.apply_config(&UartConfig::default().with_baudrate(link::BAUD))
            .expect("apply 115200");
        let mut out = FrameBuf::new();
        let _ = uart.write_async(out.build(cmd::PING, &[])).await;
        let _ = uart.flush_async().await;
        let resp = listen(&mut uart, Duration::from_millis(700), "PING response @115200").await;
        any_response |= !resp.is_empty();
        if !resp.is_empty() {
            let mut parser = FrameParser::new();
            for &b in resp.iter() {
                if parser.feed(b) {
                    let f = parser.frame();
                    if f.cmd == link::resp::ACK
                        && f.payload.first() == Some(&cmd::PING)
                        && f.payload.len() >= 3
                    {
                        let ver = u16::from_le_bytes([f.payload[1], f.payload[2]]);
                        println!("=> WIO alive: wio-e5-gps firmware v{} (link PING acked)", ver);
                    }
                }
            }
        }

        if !any_response {
            println!("(no response this round; check rail voltage at the WIO, TX/RX swap)");
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}
