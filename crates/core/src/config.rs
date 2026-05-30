//! The serde configuration tree (TOML on disk) for the whole engine.
//!
//! This is the single source of truth the engine snapshots into an `ArcSwap` and the GUI
//! edits via `ControlMsg`. It lives in the pure core so it can be validated/clamped and
//! round-tripped on Linux CI with no OS dependency. The layout mirrors DESIGN §9:
//!
//! ```toml
//! active_device = "dse_primary"
//! [thread] ...
//! [hidhide] ...
//! [devices.<id>]  vid/pid/report_rate_hz/stick_bits + [.ls] [.rs] stick configs
//! ```
//!
//! Every struct field carries `#[serde(default)]` and every enum a defensive
//! `#[serde(rename_all = "PascalCase")]` with a fallback variant, so a partial or
//! slightly-stale TOML never fails to load — missing keys take the [`Default`] value and
//! an unknown enum string resolves to the safe variant (`StickMode::None`,
//! `RcMode::UltimateDt`, etc.) instead of erroring.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::rc::RcConfig;

/// How the hot loop waits for the next HID report.
///
/// Deserialization is defensive: an unrecognized string falls back to [`WaitMode::HybridSpin`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum WaitMode {
    /// `WaitForSingleObject(INFINITE)` — lowest CPU.
    Blocking,
    /// Bounded busy-poll of the OVERLAPPED with a QPC deadline, then a real blocking wait.
    #[default]
    #[serde(other)]
    HybridSpin,
}

/// Where the per-report `dt` comes from.
///
/// Deserialization is defensive: an unrecognized string falls back to [`DtSource::QpcOnly`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum DtSource {
    /// The 16-bit hardware report timestamp (validated DS4-compat path), QPC fallback on dupes.
    DeviceTimestamp,
    /// QPC only — the safe default until the device-timestamp tick is hardware-verified.
    #[default]
    #[serde(other)]
    QpcOnly,
}

/// Threading / scheduling / timing policy for the hot loop and GUI.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
// `ThreadConfig`/`HidHideConfig` carry no `RcConfig`, so they keep value equality; the
// device/stick/engine structs below cannot (RcConfig is not `PartialEq` per the core
// contract), so tests there compare serialized TOML instead.
pub struct ThreadConfig {
    /// Physical core to pin the hot thread to. `None` => auto (avoid the SMT sibling).
    pub hot_core: Option<usize>,
    /// Physical core to pin the GUI thread to. `None` => a different physical core.
    pub gui_core: Option<usize>,
    /// `NtSetTimerResolution` target, microseconds.
    pub timer_resolution_us: u32,
    /// Register the hot thread with MMCSS.
    pub use_mmcss: bool,
    /// MMCSS task name (e.g. `"Pro Audio"`).
    pub mmcss_task: String,
    /// Hot-loop wait strategy.
    pub wait_mode: WaitMode,
    /// HybridSpin busy-poll budget, microseconds (`0` => effectively Blocking).
    pub spin_budget_us: u32,
    /// Optional CPU saver: skip filter work on byte-identical duplicate reports.
    pub skip_duplicate_reports: bool,
    /// Where the per-report `dt` comes from.
    pub dt_source: DtSource,
}

impl Default for ThreadConfig {
    fn default() -> Self {
        Self {
            hot_core: None,
            gui_core: None,
            timer_resolution_us: 500,
            use_mmcss: true,
            mmcss_task: "Pro Audio".to_string(),
            wait_mode: WaitMode::HybridSpin,
            spin_budget_us: 80,
            skip_duplicate_reports: false,
            dt_source: DtSource::QpcOnly,
        }
    }
}

/// HidHide cloaking policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct HidHideConfig {
    /// Enable HidHide cloaking of the physical pad.
    pub enabled: bool,
    /// Use the `HidHideCLI.exe` shell-out (bring-up / fallback) instead of direct IOCTLs.
    pub use_cli: bool,
    /// Path to `HidHideCLI.exe` for the CLI fallback.
    pub cli_path: String,
}

impl Default for HidHideConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            use_cli: false,
            cli_path:
                "C:\\Program Files\\Nefarius Software Solutions\\HidHide\\x64\\HidHideCLI.exe"
                    .to_string(),
        }
    }
}

/// Per-stick processing mode. `None` bypasses the RC filter entirely.
///
/// Deserialization is defensive: any unrecognized string (legacy `"FireBirdRC"`, a typo, a
/// removed feature like `"OrbitAimAssist"`) falls back to [`StickMode::None`] (filter off)
/// via `#[serde(other)]` rather than erroring — mirroring the C# `Enum.TryParse` fallback.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum StickMode {
    /// Run the RC stick filter.
    Rc,
    /// No filtering — pass the stick through untouched (also the unknown-string fallback).
    #[default]
    #[serde(other)]
    None,
}

/// One stick's configuration: its mode plus the RC parameters.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct StickConfig {
    /// Whether (and how) this stick is filtered.
    pub mode: StickMode,
    /// RC filter parameters (used when `mode == Rc`).
    pub rc: RcConfig,
}

impl StickConfig {
    /// Return a copy with the RC parameters clamped to their valid ranges.
    pub fn clamped(&self) -> StickConfig {
        StickConfig {
            mode: self.mode,
            rc: self.rc.clamped(),
        }
    }
}

/// One physical device profile.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct DeviceConfig {
    /// USB vendor id.
    pub vid: u16,
    /// USB product id.
    pub pid: u16,
    /// Target report rate, Hz (informational / hint for the I/O layer).
    pub report_rate_hz: u32,
    /// Stick bit depth the device source delivers (8 for DS4-compat).
    pub stick_bits: u8,
    /// Left-stick configuration.
    pub ls: StickConfig,
    /// Right-stick configuration.
    pub rs: StickConfig,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            vid: 0x054C,
            pid: 0x0DF2,
            report_rate_hz: 4000,
            stick_bits: 8,
            ls: StickConfig::default(),
            rs: StickConfig::default(),
        }
    }
}

impl DeviceConfig {
    /// Return a copy with both sticks' RC parameters clamped.
    pub fn clamped(&self) -> DeviceConfig {
        DeviceConfig {
            ls: self.ls.clamped(),
            rs: self.rs.clamped(),
            ..*self
        }
    }
}

/// The whole engine configuration: the active device id, global policy, and the
/// device-id → profile map.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct EngineConfig {
    /// Key (into `devices`) of the device the engine should drive.
    pub active_device: String,
    /// Threading / scheduling / timing policy.
    pub thread: ThreadConfig,
    /// HidHide cloaking policy.
    pub hidhide: HidHideConfig,
    /// Device id → profile.
    pub devices: BTreeMap<String, DeviceConfig>,
}

impl EngineConfig {
    /// Return a copy with every device's RC parameters clamped to their valid ranges.
    pub fn clamped(&self) -> EngineConfig {
        EngineConfig {
            active_device: self.active_device.clone(),
            thread: self.thread.clone(),
            hidhide: self.hidhide.clone(),
            devices: self
                .devices
                .iter()
                .map(|(k, v)| (k.clone(), v.clamped()))
                .collect(),
        }
    }
}

/// Parse an [`EngineConfig`] from a TOML string. Missing keys take their defaults and
/// unknown enum strings fall back to the safe variant; only structurally invalid TOML errors.
pub fn load_toml(s: &str) -> Result<EngineConfig, toml::de::Error> {
    toml::from_str(s)
}

/// Serialize an [`EngineConfig`] to a pretty TOML string.
///
/// Uses `toml::to_string_pretty`, which is total for this tree, so this never fails in
/// practice; on the impossible serialization error it returns an empty string rather than
/// panicking on the hot/config path.
pub fn to_toml(c: &EngineConfig) -> String {
    toml::to_string_pretty(c).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_round_trips_through_toml() {
        // `RcConfig` is not `PartialEq` (core contract), so compare by re-serialization.
        let cfg = EngineConfig::default();
        let text = to_toml(&cfg);
        let back = load_toml(&text).expect("default config must round-trip");
        assert_eq!(
            to_toml(&back),
            text,
            "default config must round-trip through TOML"
        );
    }

    #[test]
    fn embedded_default_toml_loads_to_expected_shape() {
        // A representative on-disk profile from DESIGN §9.
        let text = r#"
active_device = "dse_primary"

[thread]
hot_core = 2
gui_core = 4
timer_resolution_us = 500
use_mmcss = true
mmcss_task = "Pro Audio"
wait_mode = "HybridSpin"
spin_budget_us = 80
skip_duplicate_reports = false
dt_source = "DeviceTimestamp"

[hidhide]
enabled = true
use_cli = false
cli_path = "C:\\HidHideCLI.exe"

[devices.dse_primary]
vid = 0x054C
pid = 0x0DF2
report_rate_hz = 4000
stick_bits = 8

[devices.dse_primary.ls]
mode = "Rc"

[devices.dse_primary.ls.rc]
enabled = true
mode = "UltimateDt"
use_dynamic_curve = false
period_us = 4000
fixed_param = 100

[devices.dse_primary.rs]
mode = "Rc"
"#;
        let cfg = load_toml(text).expect("section-table TOML must load");
        assert_eq!(cfg.active_device, "dse_primary");
        assert_eq!(cfg.thread.hot_core, Some(2));
        assert_eq!(cfg.thread.dt_source, DtSource::DeviceTimestamp);
        let dev = cfg.devices.get("dse_primary").expect("device present");
        assert_eq!(dev.vid, 0x054C);
        assert_eq!(dev.pid, 0x0DF2);
        assert_eq!(dev.ls.mode, StickMode::Rc);
        assert_eq!(dev.rs.mode, StickMode::Rc);
    }

    #[test]
    fn unknown_enum_strings_fall_back_defensively() {
        // Stick mode + dt_source given garbage; should resolve to the safe defaults, not error.
        let text = r#"
active_device = "x"
[thread]
dt_source = "QpcOnly"
[devices.x]
[devices.x.ls]
mode = "OrbitAimAssist"
"#;
        let cfg = load_toml(text).expect("unknown enum string must not fail the load");
        let dev = cfg.devices.get("x").expect("device present");
        assert_eq!(dev.ls.mode, StickMode::None);
    }
}
