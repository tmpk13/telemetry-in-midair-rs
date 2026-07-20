//! Radio configuration: the `RADIO.CFG` file format and its parser.
//!
//! The file is TOML-shaped but the name is not `.toml`: it lives in the root
//! of a FAT card, where 8.3 short names allow only a three-character
//! extension.
//!
//! The WIO-E5 loads this from the SD card at boot (`RADIO.CFG`) and/or
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
//! role = "leaf"             # leaf | repeater | tx_only | rx_only
//! max_hops = 1              # retransmissions allowed per broadcast
//!
//! [beacon]
//! interval_s = 10           # position broadcast period
//! ```

use crate::lora;

/// Which halves of the air interface a node uses.
///
/// Position reporting is one-way traffic, so a node does not have to do
/// both halves. A tracker that is only ever reported *on* can leave its
/// receiver off; a base station that only collects reports never needs to
/// transmit. Each saves the power the unused half costs, and on a tracker
/// that is the larger saving by far: continuous RX draws current every
/// second between beacons, while a beacon is milliseconds of TX.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Originates its own broadcasts and receives everyone else's, but
    /// never retransmits. A network of nothing but leaves works: every
    /// node hears every other one that is in direct range.
    Leaf,
    /// A leaf that additionally retransmits frames still carrying hops,
    /// extending the network past one radio horizon.
    Repeater,
    /// Beacons its own position and nothing else. The receiver is never
    /// enabled, so this node hears no one, repeats nothing, and reports no
    /// peers - it exists only in other nodes' logs.
    TxOnly,
    /// Listens and reports what it hears, never transmitting. Its own
    /// beacon is off whatever `beacon_interval_s` says, since a beacon is a
    /// transmission.
    RxOnly,
}

impl Role {
    /// Whether this node ever puts a frame on the air.
    pub fn transmits(self) -> bool {
        !matches!(self, Role::RxOnly)
    }

    /// Whether this node enables its receiver at all.
    pub fn receives(self) -> bool {
        !matches!(self, Role::TxOnly)
    }

    /// Whether this node retransmits other nodes' frames.
    pub fn repeats(self) -> bool {
        matches!(self, Role::Repeater)
    }
}

/// Supply voltage the radio drives the TCXO at (`SetTcxoMode` trim field).
///
/// This is a property of the crystal fitted to the board, not a preference.
/// The Wio-E5 module's TCXO is a 1.8 V part; the other values exist for
/// boards built around a different one, and setting a value the hardware
/// does not expect stops the oscillator starting, which takes the radio
/// with it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TcxoVolts {
    V1_6,
    V1_7,
    V1_8,
    V2_2,
    V2_4,
    V2_7,
    V3_0,
    V3_3,
}

impl TcxoVolts {
    /// `SetTcxoMode` trim enum value.
    pub fn trim(self) -> u8 {
        match self {
            TcxoVolts::V1_6 => 0x0,
            TcxoVolts::V1_7 => 0x1,
            TcxoVolts::V1_8 => 0x2,
            TcxoVolts::V2_2 => 0x3,
            TcxoVolts::V2_4 => 0x4,
            TcxoVolts::V2_7 => 0x5,
            TcxoVolts::V3_0 => 0x6,
            TcxoVolts::V3_3 => 0x7,
        }
    }
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
    /// how receivers tell one sender's positions from another's, and two
    /// nodes sharing one are mutually deaf - each drops the other's frames
    /// as its own echo.
    pub address: u8,
    /// Which halves of the air interface this node uses, and whether it
    /// retransmits other nodes' frames.
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
    /// Which [`PositionPacket`](gps_proto::packet::PositionPacket) fields the
    /// beacon puts on the air, as a mask of the `FIELD_*` bits in
    /// [`crate::lora`]. Every extra field is airtime paid on every
    /// broadcast, so the default carries position and nothing else.
    ///
    /// The mask travels in the frame, so nodes disagreeing about it is fine:
    /// a receiver decodes whatever the sender chose to include.
    pub beacon_fields: u8,
    /// Whether the SD card is used at all. False stops logging, config
    /// read-back and the card's power draw.
    pub sd_enabled: bool,
    /// Use the internal DC-DC (SMPS) rather than the LDO, roughly halving
    /// RX and TX current at 3.3 V.
    ///
    /// The SMPS needs the module's inductor fitted, so this is only safe to
    /// leave on for boards that have one - the Wio-E5 does. Turning it off
    /// costs current but is safe anywhere.
    pub dcdc_enabled: bool,
    /// Supply the radio drives the TCXO at.
    pub tcxo_volts: TcxoVolts,
    /// How long the radio waits for the TCXO to stabilize before it will
    /// use the clock, in milliseconds.
    ///
    /// Paid on every wake from sleep, so it is a real duty-cycle cost - but
    /// too short and the radio runs off an oscillator that has not settled,
    /// which shows up as a receiver that works warm and fails cold.
    pub tcxo_startup_ms: u16,
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
            // Position only. Everything else a fix produces is written to
            // the SD log, where a byte costs nothing, rather than spent on
            // air time that has to be paid on every single broadcast.
            beacon_fields: crate::lora::FIELDS_DEFAULT,
            sd_enabled: true,
            // The Wio-E5 carries the SMPS inductor and a 1.8 V TCXO; these
            // defaults are that module's hardware, not a tuning choice.
            dcdc_enabled: true,
            tcxo_volts: TcxoVolts::V1_8,
            tcxo_startup_ms: 10,
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
                    "tx_only" => Role::TxOnly,
                    "rx_only" => Role::RxOnly,
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
            "fields" | "beacon_fields" => {
                cfg.beacon_fields = parse_fields(value).ok_or(ConfigError::BadValue(lineno))?;
                // Position is the whole point of the broadcast, and a
                // receiver has nothing to plot without it.
                if cfg.beacon_fields & lora::FIELDS_REQUIRED != lora::FIELDS_REQUIRED {
                    return Err(ConfigError::OutOfRange(lineno));
                }
            }
            "dcdc_enabled" => {
                cfg.dcdc_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?
            }
            "tcxo_volts" => {
                cfg.tcxo_volts = match unquote(value) {
                    "1.6" => TcxoVolts::V1_6,
                    "1.7" => TcxoVolts::V1_7,
                    "1.8" => TcxoVolts::V1_8,
                    "2.2" => TcxoVolts::V2_2,
                    "2.4" => TcxoVolts::V2_4,
                    "2.7" => TcxoVolts::V2_7,
                    "3.0" => TcxoVolts::V3_0,
                    "3.3" => TcxoVolts::V3_3,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            "tcxo_startup_ms" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                // The ceiling is a stuck-oscillator guard: past this the
                // radio is not slow to start, it is not starting.
                if !(1..=1_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.tcxo_startup_ms = v as u16;
            }
            // -- [sd] -------------------------------------------------------
            "sd_enabled" => cfg.sd_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
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

/// Parse a comma-separated beacon field list ("lat,lon,altitude") into a
/// [`crate::lora`] field mask. An empty list is rejected: writing `""` reads
/// like "send nothing", which is not a thing a beacon can do, so it is a
/// mistake worth reporting rather than silently accepting.
fn parse_fields(value: &str) -> Option<u8> {
    let mut mask = 0u8;
    for name in unquote(value).split(',') {
        mask |= match name.trim() {
            "lat" | "latitude" => lora::FIELD_LAT,
            "lon" | "longitude" => lora::FIELD_LON,
            "alt" | "altitude" => lora::FIELD_ALT,
            "speed" => lora::FIELD_SPEED,
            "course" => lora::FIELD_COURSE,
            "sats" => lora::FIELD_SATS,
            "time" => lora::FIELD_TIME,
            _ => return None,
        };
    }
    Some(mask)
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

    /// RADIO.example.toml states in its header that its values are the
    /// firmware defaults, and it is written by hand, so nothing but this
    /// makes that true. It is also what gps-gui-rs lays down for a board with
    /// no config yet: since the app writes every key explicitly, a stale value
    /// here is not corrected by the default it drifted from - it silently
    /// becomes the board's setting.
    #[test]
    fn example_file_documents_the_real_defaults() {
        let example = include_str!("../../RADIO.example.toml");
        assert_eq!(parse(example).unwrap(), RadioConfig::default());
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
    fn one_way_roles_parse() {
        assert_eq!(parse("role = \"tx_only\"").unwrap().role, Role::TxOnly);
        assert_eq!(parse("role = \"rx_only\"").unwrap().role, Role::RxOnly);
        // Near misses are typos, not a mode to guess at.
        assert_eq!(parse("role = \"tx\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("role = \"txonly\""), Err(ConfigError::BadValue(1)));
    }

    /// Each role uses exactly the halves of the air interface it names, and
    /// only a repeater forwards. These drive the radio's idle state and the
    /// beacon gate, so getting one wrong is a node that is silently deaf or
    /// silently mute.
    #[test]
    fn roles_use_the_halves_they_name() {
        for (role, tx, rx, repeats) in [
            (Role::Leaf, true, true, false),
            (Role::Repeater, true, true, true),
            (Role::TxOnly, true, false, false),
            (Role::RxOnly, false, true, false),
        ] {
            assert_eq!(role.transmits(), tx, "{role:?} transmits");
            assert_eq!(role.receives(), rx, "{role:?} receives");
            assert_eq!(role.repeats(), repeats, "{role:?} repeats");
        }
    }

    /// A repeater has to hear a frame before it can forward one, so no role
    /// may repeat without receiving.
    #[test]
    fn repeating_implies_receiving() {
        for role in [Role::Leaf, Role::Repeater, Role::TxOnly, Role::RxOnly] {
            assert!(!role.repeats() || role.receives(), "{role:?}");
        }
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
    fn beacon_fields_default_to_position_only() {
        let cfg = RadioConfig::default();
        assert_eq!(cfg.beacon_fields, lora::FIELD_LAT | lora::FIELD_LON);
        assert_eq!(lora::position_msg_len(cfg.beacon_fields), 10);
    }

    #[test]
    fn beacon_fields_parse() {
        let mask = parse("fields = \"lat,lon,altitude,time\"").unwrap().beacon_fields;
        assert_eq!(
            mask,
            lora::FIELD_LAT | lora::FIELD_LON | lora::FIELD_ALT | lora::FIELD_TIME
        );
        // Whitespace, the short spellings and the aliased key all work.
        assert_eq!(
            parse("beacon_fields = \"lat, lon, alt\"").unwrap().beacon_fields,
            lora::FIELD_LAT | lora::FIELD_LON | lora::FIELD_ALT
        );
        assert_eq!(parse("fields = \"lat,lon\"").unwrap(), RadioConfig::default());
    }

    #[test]
    fn beacon_fields_reject_nonsense() {
        // Unknown field name.
        assert_eq!(parse("fields = \"lat,lon,heading\""), Err(ConfigError::BadValue(1)));
        // A broadcast without a position is not a position broadcast.
        assert_eq!(parse("fields = \"altitude\""), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("fields = \"lat\""), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("fields = \"\""), Err(ConfigError::BadValue(1)));
    }

    #[test]
    fn tcxo_and_regulator_defaults_match_the_wio_e5() {
        let cfg = RadioConfig::default();
        assert!(cfg.dcdc_enabled);
        assert_eq!(cfg.tcxo_volts, TcxoVolts::V1_8);
        assert_eq!(cfg.tcxo_volts.trim(), 0x2);
        assert_eq!(cfg.tcxo_startup_ms, 10);
    }

    #[test]
    fn tcxo_and_regulator_parse() {
        assert!(!parse("dcdc_enabled = false").unwrap().dcdc_enabled);
        assert_eq!(parse("tcxo_volts = \"3.3\"").unwrap().tcxo_volts, TcxoVolts::V3_3);
        assert_eq!(parse("tcxo_volts = \"1.6\"").unwrap().tcxo_volts.trim(), 0x0);
        assert_eq!(parse("tcxo_startup_ms = 50").unwrap().tcxo_startup_ms, 50);
        // A voltage the chip has no trim setting for is a typo, not a
        // value to round to the nearest one.
        assert_eq!(parse("tcxo_volts = \"3.0V\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("tcxo_volts = \"5.0\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("tcxo_startup_ms = 0"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("tcxo_startup_ms = 2000"), Err(ConfigError::OutOfRange(1)));
    }

    #[test]
    fn sd_can_be_disabled() {
        assert!(RadioConfig::default().sd_enabled);
        assert!(!parse("sd_enabled = false").unwrap().sd_enabled);
        assert_eq!(parse("sd_enabled = yes"), Err(ConfigError::BadValue(1)));
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
