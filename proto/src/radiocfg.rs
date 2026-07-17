//! Radio configuration: the `radio.toml` file format and its parser.
//!
//! The WIO-E5 loads this from the SD card at boot (`RADIO.TOML`) and/or
//! receives it over UART from the ESP32-C6 (which in turn gets it over
//! BLE). No TOML crate runs on these targets, so this is a small no_std
//! parser for the subset the file needs: `key = value` pairs with integer,
//! boolean and quoted-string values, `#` comments, and `[section]` headers
//! (accepted and ignored - keys are unique across sections).
//!
//! Example file:
//!
//! ```toml
//! [radio]
//! frequency_hz = 915000000
//! spreading_factor = 7      # 5-12
//! bandwidth_khz = 125       # 62, 125, 250, 500
//! coding_rate = 5           # 4/5 .. 4/8
//! power_dbm = 22            # -9 .. 22
//!
//! [mesh]
//! address = 1               # 1-255
//! listen_ms = 200
//! lifetime = 2              # broadcast hop count
//!
//! [beacon]
//! interval_s = 10           # position broadcast period
//! ```

/// Parsed and validated radio configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioConfig {
    /// RF frequency in Hz.
    pub frequency_hz: u32,
    /// LoRa spreading factor (5-12).
    pub spreading_factor: u8,
    /// LoRa bandwidth in kHz (62 means 62.5).
    pub bandwidth_khz: u16,
    /// LoRa coding rate denominator: 5-8 for 4/5..4/8.
    pub coding_rate: u8,
    /// TX power in dBm (-9..22 on the STM32WLE5 high-power PA).
    pub power_dbm: i8,
    /// Mesh node address (1-255).
    pub address: u8,
    /// Mesh listen period before transmitting (ms).
    pub listen_ms: u32,
    /// Broadcast hop-count lifetime (>= 2 lets nodes repeat).
    pub lifetime: u8,
    /// Position broadcast interval in seconds (0 disables the beacon).
    pub beacon_interval_s: u16,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 915_000_000,
            spreading_factor: 7,
            bandwidth_khz: 125,
            coding_rate: 5,
            power_dbm: 22,
            address: 1,
            listen_ms: 200,
            // 2 hops so any node also acts as a repeater by default.
            lifetime: 2,
            beacon_interval_s: 10,
        }
    }
}

impl RadioConfig {
    /// Low-data-rate optimization is required when the LoRa symbol time
    /// exceeds 16.38 ms (SF11/SF12 at BW125, SF12 at BW62.5).
    pub fn ldro(&self) -> bool {
        // symbol time ms = 2^sf / bw_khz; SF11/BW125 is exactly 16.384 ms,
        // so the 0.01 ms-resolution comparison must be inclusive.
        let num = 1u32 << self.spreading_factor;
        let bw = if self.bandwidth_khz == 62 { 62 } else { self.bandwidth_khz as u32 };
        num * 100 / bw >= 1638
    }

    /// Approximate scale of the on-air time relative to SF7/BW125,
    /// used to derive TX timeouts and safe listen periods.
    pub fn airtime_scale(&self) -> u32 {
        let sf = self.spreading_factor.clamp(5, 12);
        let shift = sf.saturating_sub(7) as u32;
        let base = 1u32 << shift; // 2^(sf-7), 1/4 floor for sf < 7
        let bw = if self.bandwidth_khz == 62 { 62 } else { self.bandwidth_khz as u32 };
        (base * 125 / bw).max(1)
    }

    /// Software poll deadline for TxDone (ms).
    pub fn tx_poll_timeout_ms(&self) -> u32 {
        150 * self.airtime_scale() + 100
    }

    /// Chip-level TX timeout (ms); longer than the poll deadline so the
    /// polling loop always exits first.
    pub fn tx_chip_timeout_ms(&self) -> u32 {
        self.tx_poll_timeout_ms() + 500
    }
}

/// Config parse/validation errors. The u32 is the offending line number
/// (1-based) where one applies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// Line is not `key = value`, a comment, a blank or a `[section]`.
    Syntax(u32),
    /// Value does not parse as the expected type.
    BadValue(u32),
    /// A recognized key holds an out-of-range value.
    OutOfRange(u32),
    /// The file is not valid UTF-8.
    Utf8,
}

/// Parse TOML text into a [`RadioConfig`], starting from the defaults so a
/// partial file is fine. Unknown keys are ignored (forward compatibility).
pub fn parse(text: &str) -> Result<RadioConfig, ConfigError> {
    let mut cfg = RadioConfig::default();
    for (idx, raw_line) in text.lines().enumerate() {
        let lineno = idx as u32 + 1;
        let line = match raw_line.split_once('#') {
            Some((before, _)) => before.trim(),
            None => raw_line.trim(),
        };
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') {
            if line.ends_with(']') {
                continue; // section headers are accepted and ignored
            }
            return Err(ConfigError::Syntax(lineno));
        }
        let (key, value) = line.split_once('=').ok_or(ConfigError::Syntax(lineno))?;
        let (key, value) = (key.trim(), value.trim());
        if key.is_empty() || value.is_empty() {
            return Err(ConfigError::Syntax(lineno));
        }

        match key {
            "frequency_hz" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                // Sub-GHz ISM range the SX126x covers.
                if !(150_000_000..=960_000_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.frequency_hz = v as u32;
            }
            "spreading_factor" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(5..=12).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.spreading_factor = v as u8;
            }
            "bandwidth_khz" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !matches!(v, 62 | 125 | 250 | 500) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.bandwidth_khz = v as u16;
            }
            "coding_rate" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(5..=8).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.coding_rate = v as u8;
            }
            "power_dbm" => {
                let v = parse_i64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(-9..=22).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.power_dbm = v as i8;
            }
            "address" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(1..=255).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.address = v as u8;
            }
            "listen_ms" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(10..=60_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.listen_ms = v as u32;
            }
            "lifetime" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(1..=16).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.lifetime = v as u8;
            }
            "interval_s" | "beacon_interval_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v > 3600 {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.beacon_interval_s = v as u16;
            }
            _ => {} // unknown key: ignore
        }
    }
    Ok(cfg)
}

/// Parse raw file bytes (validates UTF-8 first).
pub fn parse_bytes(bytes: &[u8]) -> Result<RadioConfig, ConfigError> {
    parse(core::str::from_utf8(bytes).map_err(|_| ConfigError::Utf8)?)
}

fn parse_u64(s: &str) -> Option<u64> {
    // Allow underscores as digit separators, as TOML does.
    let mut n: u64 = 0;
    let mut any = false;
    for b in s.bytes() {
        match b {
            b'0'..=b'9' => {
                n = n.checked_mul(10)?.checked_add((b - b'0') as u64)?;
                any = true;
            }
            b'_' if any => {}
            _ => return None,
        }
    }
    any.then_some(n)
}

fn parse_i64(s: &str) -> Option<i64> {
    let (neg, digits) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s),
    };
    let v = parse_u64(digits)? as i64;
    Some(if neg { -v } else { v })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_when_empty() {
        assert_eq!(parse("").unwrap(), RadioConfig::default());
        assert_eq!(parse("# just a comment\n\n").unwrap(), RadioConfig::default());
    }

    #[test]
    fn full_file() {
        let toml = r#"
            # telemetry-in-midair radio config
            [radio]
            frequency_hz = 915_000_000
            spreading_factor = 10
            bandwidth_khz = 125
            coding_rate = 8
            power_dbm = 14

            [mesh]
            address = 3
            listen_ms = 900
            lifetime = 3

            [beacon]
            interval_s = 30
        "#;
        let cfg = parse(toml).unwrap();
        assert_eq!(cfg.frequency_hz, 915_000_000);
        assert_eq!(cfg.spreading_factor, 10);
        assert_eq!(cfg.coding_rate, 8);
        assert_eq!(cfg.power_dbm, 14);
        assert_eq!(cfg.address, 3);
        assert_eq!(cfg.listen_ms, 900);
        assert_eq!(cfg.lifetime, 3);
        assert_eq!(cfg.beacon_interval_s, 30);
        assert!(!cfg.ldro());
    }

    #[test]
    fn ldro_and_timeouts_scale() {
        let mut cfg = RadioConfig::default();
        assert!(!cfg.ldro());
        assert_eq!(cfg.airtime_scale(), 1);

        cfg.spreading_factor = 12;
        assert!(cfg.ldro());
        assert_eq!(cfg.airtime_scale(), 32);
        assert!(cfg.tx_poll_timeout_ms() > 4_000);
        assert!(cfg.tx_chip_timeout_ms() > cfg.tx_poll_timeout_ms());

        cfg.bandwidth_khz = 62;
        assert!(cfg.ldro());
        assert_eq!(cfg.airtime_scale(), 64);

        cfg.spreading_factor = 11;
        cfg.bandwidth_khz = 125;
        assert!(cfg.ldro());
        cfg.spreading_factor = 10;
        assert!(!cfg.ldro());
    }

    #[test]
    fn rejects_bad_input() {
        assert_eq!(parse("frequency_hz = maybe"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("spreading_factor = 13"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("bandwidth_khz = 100"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("power_dbm = 23"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("address = 0"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("\nnot a kv line"), Err(ConfigError::Syntax(2)));
        assert_eq!(parse_bytes(&[0xFF, 0xFE]), Err(ConfigError::Utf8));
        // Unknown keys pass through untouched.
        assert!(parse("future_knob = 42").is_ok());
    }
}
