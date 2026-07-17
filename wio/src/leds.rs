//! Radio activity LEDs: D6 (PA9) pulses on LoRa RX, D5 (PA10) on LoRa TX.
//!
//! The radio driver sets lock-free atomic flags (it must not touch GPIO
//! directly - it runs inside timing-sensitive TX/RX paths); the main loop
//! calls [`Leds::update`] to start and retire visible pulses.

use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::interrupt::CriticalSection;
use stm32wlxx_hal::gpio::{pins, Output, OutputArgs, PinState};

static TX_FLAG: AtomicBool = AtomicBool::new(false);
static RX_FLAG: AtomicBool = AtomicBool::new(false);

/// Called from the radio driver when a transmission starts.
pub fn note_tx() {
    TX_FLAG.store(true, Ordering::Relaxed);
}

/// Called from the radio driver when a packet is received.
pub fn note_rx() {
    RX_FLAG.store(true, Ordering::Relaxed);
}

/// LED pulse duration in ms - long enough to be visible.
const PULSE_MS: u32 = 30;

pub struct Leds {
    rx_led: Output<pins::A9>,  // D6
    tx_led: Output<pins::A10>, // D5
    rx_until: u32,
    tx_until: u32,
}

impl Leds {
    pub fn new(a9: pins::A9, a10: pins::A10, cs: &CriticalSection) -> Self {
        const ARGS: OutputArgs = OutputArgs {
            level: PinState::Low,
            ..OutputArgs::new()
        };
        Self {
            rx_led: Output::new(a9, &ARGS, cs),
            tx_led: Output::new(a10, &ARGS, cs),
            rx_until: 0,
            tx_until: 0,
        }
    }

    /// Start pulses for freshly flagged activity and retire expired ones.
    pub fn update(&mut self, now_ms: u32) {
        if TX_FLAG.swap(false, Ordering::Relaxed) {
            self.tx_led.set_level_high();
            self.tx_until = now_ms.wrapping_add(PULSE_MS);
        } else if self.tx_until != 0 && now_ms.wrapping_sub(self.tx_until) < 0x8000_0000 {
            self.tx_led.set_level_low();
            self.tx_until = 0;
        }

        if RX_FLAG.swap(false, Ordering::Relaxed) {
            self.rx_led.set_level_high();
            self.rx_until = now_ms.wrapping_add(PULSE_MS);
        } else if self.rx_until != 0 && now_ms.wrapping_sub(self.rx_until) < 0x8000_0000 {
            self.rx_led.set_level_low();
            self.rx_until = 0;
        }
    }
}
