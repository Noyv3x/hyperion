//! The serde configuration tree (TOML on disk) for the whole engine.
//!
//! This is the single source of truth the engine snapshots into an `ArcSwap` and the GUI
//! edits via `ControlMsg`. It lives in the pure core so it can be validated/clamped and
//! round-tripped on Linux CI with no OS dependency. The layout mirrors DESIGN-REMAP §3.6/§7.1/§9:
//! the sticks/triggers/bindings that used to live on `DeviceConfig` now live inside a named
//! [`Profile`] (`crate::map::Profile`), `DeviceConfig` keeps hardware identity only, and
//! `EngineConfig` grows the profile tree + per-device assignments + auto-switch rules:
//!
//! ```toml
//! active_device = "dse_primary"
//! [thread] ...
//! [hidhide] ...
//! [devices.<id>]              # vid/pid/report_rate_hz/stick_bits ONLY (no [.ls]/[.rs])
//! [profiles.<name>] ...       # full stick/trigger/binding tree (see `crate::map::Profile`)
//! [assignments]               # device-id -> profile-name
//! [auto_switch] ...
//! ```
//!
//! Every struct field carries `#[serde(default)]` and every enum a defensive
//! `#[serde(rename_all = "PascalCase")]` with a fallback variant, so a partial or slightly-stale
//! TOML never fails to load — missing keys take the [`Default`] value and an unknown enum string
//! resolves to the safe variant instead of erroring.
//!
//! **Legacy migration (§9).** An OLD-shape TOML — one that still carries `[devices.<id>.ls]` /
//! `[devices.<id>.rs]` stick tables and no `[profiles]` — is migrated on load: a single
//! `"default"` [`Profile`] is synthesized carrying the first such device's `ls`/`rs` RC settings
//! (`StickMode::Rc` → `rc_mode_on = true`), and every legacy device with a stick table is assigned
//! to it. This keeps every existing on-disk config (and the embedded-default test) loading.
//!
//! The runtime-only [`EngineConfig::resolved`] cache (`device -> Arc<ResolvedProfile>`) is
//! `#[serde(skip)]` (rebuilt by [`EngineConfig::clamped`]), so it never appears on disk and the
//! `to_toml(next) == to_toml(current)` no-op detection in the config store is unaffected.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde::{Deserialize, Serialize};

use crate::map::{Profile, ResolvedProfile};
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

/// Legacy per-stick processing mode — **back-compat serde shim only** (§9).
///
/// Previous on-disk configs wrote `[devices.<id>.ls] mode = "Rc"` to select the RC filter. The
/// sticks now live in a [`Profile`] whose [`StickSettings`](crate::stick::settings::StickSettings)
/// carries an explicit `rc_mode_on` flag, so this enum survives **only** to deserialize that legacy
/// key during migration: `Rc` → `rc_mode_on = true`, anything else (or an unknown string via
/// `#[serde(other)]`) → `false`. It is never written by the new tree.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum StickMode {
    /// Run the RC stick filter (legacy `mode = "Rc"` → `rc_mode_on = true`).
    Rc,
    /// No filtering — pass the stick through (also the unknown-string fallback).
    #[default]
    #[serde(other)]
    None,
}

/// One physical device's hardware identity.
///
/// **The sticks (`ls`/`rs`) moved into [`Profile`]** (§9); a `DeviceConfig` now carries only the
/// hardware identity the I/O layer needs, so it is `Copy` + `PartialEq` again (no `RcConfig`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
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
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            vid: 0x054C,
            pid: 0x0DF2,
            report_rate_hz: 4000,
            stick_bits: 8,
        }
    }
}

/// A single auto-profile-switch rule (foreground exe / window-title → profile).
///
/// The pure matcher (`core::autoswitch::match_rules`, M5) consumes these; M3 carries the data
/// model and serde shape so the rule table round-trips and later milestones are purely additive.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct AutoSwitchRule {
    /// Device id this rule applies to (empty => any device).
    pub device: String,
    /// Case-insensitive substring matched against the foreground exe path (empty => ignored).
    pub exe_substr: String,
    /// Case-insensitive substring matched against the foreground window title (empty => ignored).
    pub title_substr: String,
    /// Profile name to switch to on a match.
    pub profile: String,
}

/// Auto-profile-switch policy (off the hot path; the `ForegroundWatcher` consumes it in M5).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct AutoSwitchConfig {
    /// Master enable for the foreground watcher.
    pub enabled: bool,
    /// Foreground poll rate, Hz.
    pub poll_hz: u32,
    /// Ordered rule list (first match wins).
    pub rules: Vec<AutoSwitchRule>,
}

impl Default for AutoSwitchConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            poll_hz: 4,
            rules: Vec::new(),
        }
    }
}

/// The whole engine configuration: the active device id, global policy, the device-identity map,
/// the named [`Profile`] tree, per-device profile assignments, and auto-switch rules.
///
/// `profiles` is `Arc<BTreeMap<…>>` so the per-generation `EngineConfig::clone()` (and the freed
/// old `Arc<EngineConfig>` Drop on the hot thread) is a refcount bump, not a deep tree copy
/// (verifier latency FIX 7 / §13 conflict 13).
///
/// Not `PartialEq`: `Profile` transitively contains `RcConfig`, which is `Copy`-only by the core
/// contract; config equality is taken via re-serialized TOML where needed.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct EngineConfig {
    /// Key (into `devices`) of the device the engine should drive.
    pub active_device: String,
    /// Threading / scheduling / timing policy.
    pub thread: ThreadConfig,
    /// HidHide cloaking policy.
    pub hidhide: HidHideConfig,
    /// Device id → hardware identity (no stick settings anymore).
    pub devices: BTreeMap<String, DeviceConfig>,
    /// Named editable profiles (sticks/triggers/bindings/mouse/gyro/macros/specials).
    pub profiles: Arc<BTreeMap<String, Profile>>,
    /// Device id → active profile name.
    pub assignments: BTreeMap<String, String>,
    /// Auto-profile-switch policy.
    pub auto_switch: AutoSwitchConfig,
    /// Runtime-only hot-facing cache: device id → resolved profile. **Absent from TOML**
    /// (`#[serde(skip)]`); rebuilt for assigned devices by [`clamped`](Self::clamped).
    #[serde(skip)]
    pub resolved: BTreeMap<String, Arc<ResolvedProfile>>,
}

impl EngineConfig {
    /// Return a copy with the runtime `resolved` cache rebuilt for **assigned devices only**.
    ///
    /// The editable `profiles` tree is `Arc`-shared through unchanged (a refcount bump, not a deep
    /// copy — verifier latency FIX 7 / §13 conflict 13); per-profile stick/trigger ranges are
    /// clamped at the moment they are resolved ([`Profile::resolve`] clamps each stage internally),
    /// so the hot-facing `ResolvedProfile` is always clamped while the on-disk editable form is left
    /// verbatim. `resolved` is `#[serde(skip)]`, so rebuilding it here does not perturb the on-disk
    /// form and the config store's `to_toml(next) == to_toml(current)` no-op detection still holds.
    pub fn clamped(&self) -> EngineConfig {
        // Clamp the RAW profiles too, not just the resolved cache, so the persisted value always
        // equals the runtime-clamped value (C# F12 intent): an edit of period_us=999 is stored as
        // 1000. Each Profile's ls/rs/l2/r2 carry their own `clamped()`.
        let profiles: BTreeMap<String, Profile> = self
            .profiles
            .iter()
            .map(|(id, p)| {
                (
                    id.clone(),
                    Profile {
                        ls: p.ls.clamped(),
                        rs: p.rs.clamped(),
                        l2: p.l2.clamped(),
                        r2: p.r2.clamped(),
                        ..p.clone()
                    },
                )
            })
            .collect();

        // Rebuild the hot-facing cache for assigned devices only (verifier latency FIX 1/2/10):
        // skip any assignment whose profile id is missing. `resolve()` clamps each profile.
        let resolved: BTreeMap<String, Arc<ResolvedProfile>> = self
            .assignments
            .iter()
            .filter_map(|(dev, pid)| {
                profiles
                    .get(pid)
                    .map(|p| (dev.clone(), Arc::new(p.resolve())))
            })
            .collect();

        EngineConfig {
            active_device: self.active_device.clone(),
            thread: self.thread.clone(),
            hidhide: self.hidhide.clone(),
            devices: self.devices.clone(),
            profiles: Arc::new(profiles),
            assignments: self.assignments.clone(),
            auto_switch: self.auto_switch.clone(),
            resolved,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Legacy migration shim (§9).
//
// The OLD on-disk shape carried sticks on the device (`[devices.<id>.ls] / [.rs]`). To keep those
// configs loading, `load_toml` first parses into a permissive raw form that captures BOTH the new
// `profiles`/`assignments` keys AND the legacy per-device `ls`/`rs` stick tables, then synthesizes
// a `"default"` profile + assignments when (and only when) the new tree is empty but legacy stick
// tables exist. New-shape configs deserialize unchanged (no legacy tables => no synthesis).
// ---------------------------------------------------------------------------------------------

/// Legacy `[devices.<id>.ls]` / `[.rs]` table: a `StickMode` selector + the RC parameters.
#[derive(Clone, Copy, Debug, Default, Deserialize)]
#[serde(default)]
struct LegacyStickConfig {
    mode: StickMode,
    rc: RcConfig,
}

/// Legacy device table that still carries `ls`/`rs`. Captures the new-shape identity fields too,
/// so a new-shape `[devices.<id>]` (no `ls`/`rs`) parses with `ls`/`rs == None`.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(default)]
struct RawDevice {
    vid: u16,
    pid: u16,
    report_rate_hz: u32,
    stick_bits: u8,
    ls: Option<LegacyStickConfig>,
    rs: Option<LegacyStickConfig>,
}

impl Default for RawDevice {
    fn default() -> Self {
        let d = DeviceConfig::default();
        Self {
            vid: d.vid,
            pid: d.pid,
            report_rate_hz: d.report_rate_hz,
            stick_bits: d.stick_bits,
            ls: None,
            rs: None,
        }
    }
}

impl RawDevice {
    /// Strip the legacy stick tables down to the new-shape hardware identity.
    fn identity(&self) -> DeviceConfig {
        DeviceConfig {
            vid: self.vid,
            pid: self.pid,
            report_rate_hz: self.report_rate_hz,
            stick_bits: self.stick_bits,
        }
    }

    /// Whether this device carried a legacy stick table (the migration trigger).
    fn has_legacy_sticks(&self) -> bool {
        self.ls.is_some() || self.rs.is_some()
    }
}

/// Permissive raw config: the full new shape plus the legacy per-device stick tables.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
struct RawConfig {
    active_device: String,
    thread: ThreadConfig,
    hidhide: HidHideConfig,
    devices: BTreeMap<String, RawDevice>,
    profiles: BTreeMap<String, Profile>,
    assignments: BTreeMap<String, String>,
    auto_switch: AutoSwitchConfig,
}

/// Build a `StickSettings` from a legacy stick table: fold the RC params in and set `rc_mode_on`
/// from the legacy `mode` (`Rc` => true). Everything else is the pass-through default.
fn stick_settings_from_legacy(legacy: &LegacyStickConfig) -> crate::stick::settings::StickSettings {
    crate::stick::settings::StickSettings {
        rc: legacy.rc,
        rc_mode_on: matches!(legacy.mode, StickMode::Rc),
        ..crate::stick::settings::StickSettings::default()
    }
}

impl RawConfig {
    /// Lower the raw form into an [`EngineConfig`], synthesizing the `"default"` profile +
    /// assignments from any legacy stick tables when the new profile tree is empty.
    fn into_engine_config(self) -> EngineConfig {
        let RawConfig {
            active_device,
            thread,
            hidhide,
            devices: raw_devices,
            profiles: new_profiles,
            assignments: new_assignments,
            auto_switch,
        } = self;

        // Hardware identity for every device (legacy stick tables dropped).
        let devices: BTreeMap<String, DeviceConfig> = raw_devices
            .iter()
            .map(|(id, d)| (id.clone(), d.identity()))
            .collect();

        let mut profiles = new_profiles;
        let mut assignments = new_assignments;

        // MIGRATE only when the new tree carries no profiles AND a legacy stick table exists.
        let has_legacy = raw_devices.values().any(RawDevice::has_legacy_sticks);
        if profiles.is_empty() && has_legacy {
            // Seed the synthesized "default" profile from the FIRST device with a stick table
            // (BTreeMap iterates by sorted id, so this is deterministic). Per-stick: a present
            // legacy table maps to its StickSettings, an absent one to the pass-through default.
            let seed = raw_devices
                .values()
                .find(|d| d.has_legacy_sticks())
                .expect("has_legacy implies at least one device with a stick table");

            let mut default_profile = Profile {
                name: "default".to_string(),
                ..Profile::default()
            };
            if let Some(ls) = seed.ls.as_ref() {
                default_profile.ls = stick_settings_from_legacy(ls);
            }
            if let Some(rs) = seed.rs.as_ref() {
                default_profile.rs = stick_settings_from_legacy(rs);
            }
            profiles.insert("default".to_string(), default_profile);

            // Assign every legacy-stick device to the synthesized profile (don't clobber an
            // assignment the user may already have written by hand).
            for (id, d) in &raw_devices {
                if d.has_legacy_sticks() {
                    assignments
                        .entry(id.clone())
                        .or_insert_with(|| "default".to_string());
                }
            }
        }

        EngineConfig {
            active_device,
            thread,
            hidhide,
            devices,
            profiles: Arc::new(profiles),
            assignments,
            auto_switch,
            // Filled by `clamped()`; never deserialized.
            resolved: BTreeMap::new(),
        }
    }
}

/// Parse an [`EngineConfig`] from a TOML string. Missing keys take their defaults and unknown enum
/// strings fall back to the safe variant; only structurally invalid TOML errors.
///
/// A legacy-shape TOML (per-device `ls`/`rs` stick tables, no `[profiles]`) is migrated to a
/// synthesized `"default"` profile + assignments (§9). The runtime `resolved` cache is left empty
/// here — call [`EngineConfig::clamped`] to populate it.
pub fn load_toml(s: &str) -> Result<EngineConfig, toml::de::Error> {
    let raw: RawConfig = toml::from_str(s)?;
    Ok(raw.into_engine_config())
}

/// Serialize an [`EngineConfig`] to a pretty TOML string.
///
/// Uses `toml::to_string_pretty`, which is total for this tree, so this never fails in practice;
/// on the impossible serialization error it returns an empty string rather than panicking on the
/// hot/config path. The `resolved` cache is `#[serde(skip)]` and never appears in the output.
pub fn to_toml(c: &EngineConfig) -> String {
    toml::to_string_pretty(c).unwrap_or_default()
}

impl EngineConfig {
    /// The shipped starter configuration: one `"default"` profile, a `dse_primary` device, and the
    /// `dse_primary -> "default"` assignment, so the shipped `config.toml` concept works out of the
    /// box. Use this to seed an empty config file on first run.
    ///
    /// Distinct from [`Default`]: the serde `#[serde(default)]` path uses `EngineConfig::default()`
    /// for missing top-level keys, so `Default` must stay the empty tree to keep that path total.
    /// `resolved` is left empty (call [`clamped`](Self::clamped) to populate it).
    pub fn default_shipped() -> EngineConfig {
        let mut devices = BTreeMap::new();
        devices.insert("dse_primary".to_string(), DeviceConfig::default());

        let mut profiles = BTreeMap::new();
        profiles.insert(
            "default".to_string(),
            Profile {
                name: "default".to_string(),
                ..Profile::default()
            },
        );

        let mut assignments = BTreeMap::new();
        assignments.insert("dse_primary".to_string(), "default".to_string());

        EngineConfig {
            active_device: "dse_primary".to_string(),
            thread: ThreadConfig::default(),
            hidhide: HidHideConfig::default(),
            devices,
            profiles: Arc::new(profiles),
            assignments,
            auto_switch: AutoSwitchConfig::default(),
            resolved: BTreeMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::binding::{BindTarget, BindingSlot, KeyKind, PadBtn};
    use crate::output::PadTarget;

    #[test]
    fn default_round_trips_through_toml() {
        // `EngineConfig`/`Profile` are not `PartialEq` (RcConfig is Copy-only), so compare by
        // re-serialization.
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
    fn shipped_default_round_trips_and_has_default_profile() {
        let cfg = EngineConfig::default_shipped();
        assert_eq!(cfg.active_device, "dse_primary");
        assert!(cfg.profiles.contains_key("default"));
        assert_eq!(
            cfg.assignments.get("dse_primary").map(String::as_str),
            Some("default")
        );

        let text = to_toml(&cfg);
        let back = load_toml(&text).expect("shipped default must round-trip");
        assert_eq!(to_toml(&back), text, "shipped default must round-trip");
    }

    #[test]
    fn full_profile_tree_round_trips() {
        // Build a non-trivial tree (a real binding + a renamed profile + an assignment + a rule)
        // and assert TOML -> load -> re-serialize is stable.
        let mut profiles = BTreeMap::new();
        let mut fps = Profile {
            name: "fps".to_string(),
            output_kind: PadTarget::X360,
            ..Profile::default()
        };
        fps.bindings.insert(
            crate::input::Control::Cross,
            BindingSlot::from_bind(BindTarget::GamepadButton(PadBtn::B)),
        );
        fps.bindings.insert(
            crate::input::Control::Square,
            BindingSlot::from_bind(BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            }),
        );
        fps.ls.rc_mode_on = true;
        fps.ls.rc.enabled = true;
        profiles.insert("fps".to_string(), fps);

        let mut assignments = BTreeMap::new();
        assignments.insert("dse_primary".to_string(), "fps".to_string());

        let mut devices = BTreeMap::new();
        devices.insert("dse_primary".to_string(), DeviceConfig::default());

        let cfg = EngineConfig {
            active_device: "dse_primary".to_string(),
            devices,
            profiles: Arc::new(profiles),
            assignments,
            auto_switch: AutoSwitchConfig {
                enabled: true,
                poll_hz: 4,
                rules: vec![AutoSwitchRule {
                    device: "dse_primary".to_string(),
                    exe_substr: "game.exe".to_string(),
                    title_substr: String::new(),
                    profile: "fps".to_string(),
                }],
            },
            ..EngineConfig::default()
        };

        let text = to_toml(&cfg);
        let back = load_toml(&text).expect("full profile tree must load");
        assert_eq!(to_toml(&back), text, "full profile tree must round-trip");

        // Spot-check the load preserved structure.
        let p = back.profiles.get("fps").expect("fps profile present");
        assert_eq!(p.output_kind, PadTarget::X360);
        assert!(p.ls.rc_mode_on);
        assert_eq!(
            p.bindings
                .get(&crate::input::Control::Cross)
                .map(|s| s.bind),
            Some(BindTarget::GamepadButton(PadBtn::B))
        );
        assert_eq!(back.auto_switch.rules.len(), 1);
    }

    #[test]
    fn legacy_device_stick_toml_migrates_to_default_profile() {
        // The pre-M3 on-disk shape: sticks on the device, no [profiles]. Must synthesize a
        // "default" profile carrying the RC settings + an assignment.
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
        let cfg = load_toml(text).expect("legacy section-table TOML must load");
        assert_eq!(cfg.active_device, "dse_primary");
        assert_eq!(cfg.thread.hot_core, Some(2));
        assert_eq!(cfg.thread.dt_source, DtSource::DeviceTimestamp);

        // Device kept only its hardware identity.
        let dev = cfg.devices.get("dse_primary").expect("device present");
        assert_eq!(dev.vid, 0x054C);
        assert_eq!(dev.pid, 0x0DF2);
        assert_eq!(dev.report_rate_hz, 4000);
        assert_eq!(dev.stick_bits, 8);

        // A "default" profile was synthesized with the legacy RC settings folded in.
        let profile = cfg.profiles.get("default").expect("synthesized profile");
        assert!(profile.ls.rc_mode_on, "ls legacy mode=Rc => rc_mode_on");
        assert!(profile.ls.rc.enabled, "ls legacy rc.enabled carried");
        assert_eq!(profile.ls.rc.fixed_param, 100);
        assert!(profile.rs.rc_mode_on, "rs legacy mode=Rc => rc_mode_on");

        // And the device was assigned to it.
        assert_eq!(
            cfg.assignments.get("dse_primary").map(String::as_str),
            Some("default")
        );
    }

    #[test]
    fn legacy_unknown_stick_mode_falls_back_to_passthrough() {
        // Legacy stick mode given garbage; the migrated profile must read rc_mode_on = false (the
        // StickMode::None fallback), not error.
        let text = r#"
active_device = "x"
[thread]
dt_source = "QpcOnly"
[devices.x]
[devices.x.ls]
mode = "OrbitAimAssist"
"#;
        let cfg = load_toml(text).expect("unknown enum string must not fail the load");
        let profile = cfg.profiles.get("default").expect("synthesized profile");
        assert!(
            !profile.ls.rc_mode_on,
            "unknown legacy stick mode => rc_mode_on false"
        );
        assert_eq!(
            cfg.assignments.get("x").map(String::as_str),
            Some("default")
        );
    }

    #[test]
    fn new_shape_with_profiles_is_not_migrated() {
        // A config that already has [profiles] must load verbatim — no "default" synthesis even if
        // (hypothetically) a device still listed a stick table.
        let text = r#"
active_device = "dse_primary"

[devices.dse_primary]
vid = 0x054C
pid = 0x0DF2

[profiles.custom]
name = "custom"

[assignments]
dse_primary = "custom"
"#;
        let cfg = load_toml(text).expect("new-shape TOML must load");
        assert!(cfg.profiles.contains_key("custom"));
        assert!(
            !cfg.profiles.contains_key("default"),
            "no synthesized default when profiles already present"
        );
        assert_eq!(
            cfg.assignments.get("dse_primary").map(String::as_str),
            Some("custom")
        );
    }

    #[test]
    fn clamped_rebuilds_resolved_for_assigned_devices_only() {
        let mut profiles = BTreeMap::new();
        let mut p = Profile {
            name: "default".to_string(),
            ..Profile::default()
        };
        // Out-of-range stick values to verify clamping flows into the resolved profile.
        p.ls.dead_zone.dead_zone = 9999;
        p.ls.sensitivity = 2.0;
        profiles.insert("default".to_string(), p);

        let mut assignments = BTreeMap::new();
        assignments.insert("dse_primary".to_string(), "default".to_string());
        // An assignment to a MISSING profile must not produce a resolved entry.
        assignments.insert("ghost".to_string(), "nonexistent".to_string());

        let cfg = EngineConfig {
            active_device: "dse_primary".to_string(),
            profiles: Arc::new(profiles),
            assignments,
            ..EngineConfig::default()
        };
        let clamped = cfg.clamped();

        // Resolved built for the assigned+existing device only.
        assert!(clamped.resolved.contains_key("dse_primary"));
        assert!(
            !clamped.resolved.contains_key("ghost"),
            "assignment to a missing profile yields no resolved entry"
        );
        assert_eq!(clamped.resolved.len(), 1);

        // The resolved stick settings are clamped.
        let rp = clamped
            .resolved
            .get("dse_primary")
            .expect("resolved present");
        assert_eq!(rp.ls.dead_zone.dead_zone, 127, "dead_zone clamped to 127");
        assert_eq!(rp.ls.sensitivity, 2.0, "sensitivity preserved");
    }

    #[test]
    fn resolved_is_absent_from_serialized_toml() {
        let cfg = EngineConfig::default_shipped().clamped();
        assert!(
            !cfg.resolved.is_empty(),
            "clamped() should have populated resolved for the assigned device"
        );
        let text = to_toml(&cfg);
        assert!(
            !text.contains("resolved"),
            "the runtime resolved cache must not appear on disk:\n{text}"
        );
    }
}
