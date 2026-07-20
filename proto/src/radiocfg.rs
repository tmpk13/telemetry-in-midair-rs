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
//! rx_boost = false          # boosted RX gain, ~+2 dB for more RX current
//!
//! [mesh]
//! address = 1               # 1-255
//! role = "leaf"             # leaf | repeater
//! max_hops = 1              # retransmissions allowed per broadcast
//!
//! [beacon]
//! interval_s = 10           # position broadcast period
//! ```

/// What a node does with frames that did not originate on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Originates its own broadcasts and receives everyone else's, but
    /// never retransmits. A network of nothing but leaves works: every
    /// node hears every other one that is in direct range.
    Leaf,
    /// A leaf that additionally retransmits frames still carrying hops,
    /// extending the network past one radio horizon.
    Repeater,
}

/// GPS receiver power mode (u-blox M10 `CFG-PM-OPERATEMODE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerMode {
    /// Continuous tracking, lowest fix latency.
    Full,
    /// Power-save on/off: acquire a fix, then power down until the next
    /// update period.
    PsmOnOff,
    /// Power-save cyclic tracking: stays in a reduced-power tracking loop.
    PsmCyclic,
}

impl PowerMode {
    /// `CFG-PM-OPERATEMODE` enum value.
    pub fn operate_mode(self) -> u8 {
        match self {
            PowerMode::Full => 0,
            PowerMode::PsmOnOff => 1,
            PowerMode::PsmCyclic => 2,
        }
    }
}

/// GPS navigation dynamic model (u-blox M10 `CFG-NAVSPG-DYNMODEL`). Only the
/// subset useful for this tracker is exposed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynModel {
    Portable,
    Stationary,
    Pedestrian,
    Automotive,
    Sea,
    /// Airborne with < 1 g acceleration.
    Airborne1g,
    /// Airborne with < 2 g acceleration.
    Airborne2g,
    /// Airborne with < 4 g acceleration.
    Airborne4g,
}

impl DynModel {
    /// `CFG-NAVSPG-DYNMODEL` enum value.
    pub fn dynmodel(self) -> u8 {
        match self {
            DynModel::Portable => 0,
            DynModel::Stationary => 2,
            DynModel::Pedestrian => 3,
            DynModel::Automotive => 4,
            DynModel::Sea => 5,
            DynModel::Airborne1g => 6,
            DynModel::Airborne2g => 7,
            DynModel::Airborne4g => 8,
        }
    }
}

/// GPS receiver configuration (u-blox M10, applied via UBX-CFG-VALSET).
///
/// The constellation and power defaults match the M10 factory set, so an
/// absent `[gps]` section leaves the module at its out-of-box behavior.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpsConfig {
    pub gps_enabled: bool,
    pub glonass_enabled: bool,
    pub galileo_enabled: bool,
    pub beidou_enabled: bool,
    pub qzss_enabled: bool,
    pub sbas_enabled: bool,
    /// Receiver power mode.
    pub power_mode: PowerMode,
    /// Measurement period in milliseconds (25-10000, i.e. 40 Hz down to
    /// 0.1 Hz). The nav solution runs at the same rate.
    pub meas_rate_ms: u16,
    /// Navigation dynamic model.
    pub dyn_model: DynModel,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            gps_enabled: true,
            // GLONASS is off in the M10 default concurrent set (GPS +
            // Galileo + BeiDou + QZSS + SBAS).
            glonass_enabled: false,
            galileo_enabled: true,
            beidou_enabled: true,
            qzss_enabled: true,
            sbas_enabled: true,
            power_mode: PowerMode::Full,
            meas_rate_ms: 1000,
            dyn_model: DynModel::Portable,
        }
    }
}

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
    /// Receiver boosted gain (SX126x `RxGain` register). Roughly +2 dB of
    /// sensitivity for a few mA more while listening; the chip powers up
    /// with it off.
    ///
    /// Only two of the register's four settings are documented (power
    /// saving and boosted), so this is a bool rather than an enum - the
    /// intermediate values have no specified behavior to expose.
    pub rx_boost: bool,
    /// This node's address (1-255). Must be unique on the network: it is
    /// how receivers tell one sender's positions from another's.
    pub address: u8,
    /// Whether this node retransmits other nodes' frames.
    pub role: Role,
    /// Retransmissions allowed for a broadcast this node originates, i.e.
    /// the `hops_left` it stamps into the frame. 0 means no repeater will
    /// forward it; 1 covers the usual single-repeater deployment.
    ///
    /// This is a property of the sender, not of the repeaters, so raising
    /// it on one node does not require touching any other.
    pub max_hops: u8,
    /// Position broadcast interval in seconds (0 disables the beacon).
    pub beacon_interval_s: u16,
    /// GPS receiver configuration.
    pub gps: GpsConfig,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 915_000_000,
            spreading_factor: 7,
            bandwidth_khz: 125,
            coding_rate: 5,
            power_dbm: 22,
            // Off, matching the chip's power-up state: enabling it costs
            // receive current continuously on whichever node is listening,
            // which is a trade to opt into rather than inherit.
            rx_boost: false,
            address: 1,
            // Leaf by default: repeating is a job you give one well-placed
            // node, not something every node should do to every frame.
            role: Role::Leaf,
            // Allow one repeat, so dropping a repeater into an existing
            // fleet works without reconfiguring the nodes already deployed.
            max_hops: 1,
            beacon_interval_s: 10,
            gps: GpsConfig::default(),
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

    /// Upper bound of the random delay a repeater waits before forwarding
    /// a frame (ms).
    ///
    /// Two repeaters that hear the same broadcast would otherwise answer it
    /// at the same instant and collide every time, so the wait has to span
    /// enough air time for one of them to win outright - hence scaling with
    /// the modulation rather than a fixed number of milliseconds.
    pub fn repeat_jitter_ms(&self) -> u32 {
        100 * self.airtime_scale()
    }
}

/// Ceiling on [`RadioConfig::max_hops`]. Each hop costs another full
/// transmission of the same frame on a shared channel, so the useful range
/// is small and the limit exists to keep a typo from flooding the band.
pub const MAX_HOPS_LIMIT: u8 = 8;

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
            "rx_boost" => cfg.rx_boost = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "address" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(1..=255).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.address = v as u8;
            }
            "role" => {
                cfg.role = match unquote(value) {
                    "leaf" => Role::Leaf,
                    "repeater" => Role::Repeater,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            "max_hops" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v > MAX_HOPS_LIMIT as u64 {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.max_hops = v as u8;
            }
            // Accepted so cards written for the old mesh keep the hop count
            // their author intended: it counted transmissions, where
            // max_hops counts only the retransmissions after the first.
            "lifetime" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(1..=16).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.max_hops = ((v - 1) as u8).min(MAX_HOPS_LIMIT);
            }
            "interval_s" | "beacon_interval_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v > 3600 {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.beacon_interval_s = v as u16;
            }
            // -- [gps] ------------------------------------------------------
            "gps_enabled" => cfg.gps.gps_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "glonass_enabled" => cfg.gps.glonass_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "galileo_enabled" => cfg.gps.galileo_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "beidou_enabled" => cfg.gps.beidou_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "qzss_enabled" => cfg.gps.qzss_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "sbas_enabled" => cfg.gps.sbas_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "power_mode" => {
                cfg.gps.power_mode = match unquote(value) {
                    "full" => PowerMode::Full,
                    "psmoo" | "psm_onoff" => PowerMode::PsmOnOff,
                    "psmct" | "psm_cyclic" => PowerMode::PsmCyclic,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            "meas_rate_ms" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(25..=10_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.gps.meas_rate_ms = v as u16;
            }
            "dynamic_model" | "dyn_model" => {
                cfg.gps.dyn_model = match unquote(value) {
                    "portable" => DynModel::Portable,
                    "stationary" => DynModel::Stationary,
                    "pedestrian" => DynModel::Pedestrian,
                    "automotive" => DynModel::Automotive,
                    "sea" => DynModel::Sea,
                    "airborne1g" => DynModel::Airborne1g,
                    "airborne2g" => DynModel::Airborne2g,
                    "airborne4g" => DynModel::Airborne4g,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
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

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "true" => Some(true),
        "false" => Some(false),
        _ => None,
    }
}

/// Strip matching single or double quotes from a TOML string value.
fn unquote(s: &str) -> &str {
    let b = s.as_bytes();
    if b.len() >= 2 && (b[0] == b'"' || b[0] == b'\'') && b[b.len() - 1] == b[0] {
        &s[1..s.len() - 1]
    } else {
        s
    }
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
            rx_boost = true

            [mesh]
            address = 3
            role = "repeater"
            max_hops = 2

            [beacon]
            interval_s = 30
        "#;
        let cfg = parse(toml).unwrap();
        assert_eq!(cfg.frequency_hz, 915_000_000);
        assert_eq!(cfg.spreading_factor, 10);
        assert_eq!(cfg.coding_rate, 8);
        assert_eq!(cfg.power_dbm, 14);
        assert!(cfg.rx_boost);
        assert_eq!(cfg.address, 3);
        assert_eq!(cfg.role, Role::Repeater);
        assert_eq!(cfg.max_hops, 2);
        assert_eq!(cfg.beacon_interval_s, 30);
        assert!(!cfg.ldro());
    }

    #[test]
    fn role_defaults_to_leaf_with_one_repeat_allowed() {
        let cfg = RadioConfig::default();
        assert_eq!(cfg.role, Role::Leaf);
        assert_eq!(cfg.max_hops, 1);
        assert_eq!(parse("role = \"leaf\"").unwrap().role, Role::Leaf);
        assert_eq!(parse("max_hops = 0").unwrap().max_hops, 0);
        assert_eq!(parse("role = repeater").unwrap().role, Role::Repeater);
        assert_eq!(parse("role = \"gateway\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("max_hops = 9"), Err(ConfigError::OutOfRange(1)));
    }

    #[test]
    fn legacy_lifetime_maps_to_hop_count() {
        // The old key counted transmissions; 2 meant "one repeat".
        assert_eq!(parse("lifetime = 2").unwrap().max_hops, 1);
        assert_eq!(parse("lifetime = 1").unwrap().max_hops, 0);
        assert_eq!(parse("lifetime = 16").unwrap().max_hops, MAX_HOPS_LIMIT);
        assert_eq!(parse("lifetime = 0"), Err(ConfigError::OutOfRange(1)));
        // listen_ms is gone; an old card carrying it still parses.
        assert_eq!(parse("listen_ms = 900").unwrap(), RadioConfig::default());
    }

    #[test]
    fn repeat_jitter_tracks_air_time() {
        let mut cfg = RadioConfig::default();
        let fast = cfg.repeat_jitter_ms();
        cfg.spreading_factor = 12;
        assert!(cfg.repeat_jitter_ms() > fast);
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
    fn gps_defaults_when_section_absent() {
        assert_eq!(parse("frequency_hz = 915000000").unwrap().gps, GpsConfig::default());
    }

    #[test]
    fn gps_section() {
        let toml = r#"
            [gps]
            gps_enabled = true
            glonass_enabled = true
            galileo_enabled = false
            beidou_enabled = false
            qzss_enabled = false
            sbas_enabled = false
            power_mode = "psmoo"
            meas_rate_ms = 500
            dynamic_model = "airborne2g"
        "#;
        let g = parse(toml).unwrap().gps;
        assert!(g.gps_enabled);
        assert!(g.glonass_enabled);
        assert!(!g.galileo_enabled);
        assert!(!g.sbas_enabled);
        assert_eq!(g.power_mode, PowerMode::PsmOnOff);
        assert_eq!(g.power_mode.operate_mode(), 1);
        assert_eq!(g.meas_rate_ms, 500);
        assert_eq!(g.dyn_model, DynModel::Airborne2g);
        assert_eq!(g.dyn_model.dynmodel(), 7);
    }

    #[test]
    fn rejects_bad_gps_input() {
        assert_eq!(parse("gps_enabled = maybe"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("power_mode = \"turbo\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("dynamic_model = spaceship"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("meas_rate_ms = 10"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("meas_rate_ms = 20000"), Err(ConfigError::OutOfRange(1)));
    }

    #[test]
    fn rejects_bad_input() {
        assert_eq!(parse("frequency_hz = maybe"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("spreading_factor = 13"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("bandwidth_khz = 100"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("power_dbm = 23"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("address = 0"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("rx_boost = 1"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("\nnot a kv line"), Err(ConfigError::Syntax(2)));
        assert_eq!(parse_bytes(&[0xFF, 0xFE]), Err(ConfigError::Utf8));
        // Unknown keys pass through untouched.
        assert!(parse("future_knob = 42").is_ok());
    }
}
