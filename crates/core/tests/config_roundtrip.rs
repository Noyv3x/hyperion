//! Config serde tree: default round-trips losslessly through TOML, `clamped()` rebuilds the
//! hot-facing `resolved` cache (pinning the per-profile stick ranges), legacy on-disk shapes
//! migrate to a synthesized `"default"` profile, and unknown enum strings fall back defensively
//! (DESIGN-REMAP §9).
//!
//! The sticks moved out of `DeviceConfig` into a named `Profile` (`hyperion_core::map::Profile`),
//! so these tests exercise the profile tree + the legacy-migration shim rather than the old
//! per-device `StickConfig`.

use std::collections::BTreeMap;
use std::sync::Arc;

use hyperion_core::config::{
    load_toml, to_toml, DeviceConfig, DtSource, EngineConfig, StickMode, ThreadConfig, WaitMode,
};
use hyperion_core::map::Profile;
use hyperion_core::rc::RcMode;

// `RcConfig` is not `PartialEq` (core contract) and `Profile` transitively contains it, so config
// equality is checked by comparing the re-serialized TOML: `load(to_toml(x))` must serialize back
// to the same text as `x`.
fn assert_round_trips(cfg: &EngineConfig) {
    let text = to_toml(cfg);
    let back = load_toml(&text).expect("config must parse");
    assert_eq!(
        to_toml(&back),
        text,
        "config must survive a TOML round-trip"
    );
}

#[test]
fn default_round_trips_through_toml() {
    assert_round_trips(&EngineConfig::default());
}

#[test]
fn shipped_default_round_trips_through_toml() {
    assert_round_trips(&EngineConfig::default_shipped());
}

#[test]
fn populated_config_round_trips() {
    // The new shape: hardware identity on the device, sticks/bindings inside a named profile.
    let mut devices = BTreeMap::new();
    devices.insert(
        "dse_primary".to_string(),
        DeviceConfig {
            vid: 0x054C,
            pid: 0x0DF2,
            report_rate_hz: 4000,
            stick_bits: 8,
        },
    );

    let mut ls_profile = Profile {
        name: "default".to_string(),
        ..Profile::default()
    };
    ls_profile.ls.rc_mode_on = true;
    ls_profile.ls.rc.enabled = true;
    ls_profile.ls.rc.mode = RcMode::UltimateDt;
    ls_profile.ls.rc.period_us = 4000;
    ls_profile.ls.rc.fixed_param = 100;
    ls_profile.rs.rc_mode_on = true;

    let mut profiles = BTreeMap::new();
    profiles.insert("default".to_string(), ls_profile);

    let mut assignments = BTreeMap::new();
    assignments.insert("dse_primary".to_string(), "default".to_string());

    let cfg = EngineConfig {
        active_device: "dse_primary".to_string(),
        thread: ThreadConfig {
            hot_core: Some(2),
            gui_core: Some(4),
            wait_mode: WaitMode::Blocking,
            dt_source: DtSource::DeviceTimestamp,
            ..ThreadConfig::default()
        },
        hidhide: Default::default(),
        devices,
        profiles: Arc::new(profiles),
        assignments,
        ..EngineConfig::default()
    };

    assert_round_trips(&cfg);
}

#[test]
fn defaults_match_design_section_9() {
    let t = ThreadConfig::default();
    assert_eq!(t.timer_resolution_us, 500);
    assert_eq!(t.mmcss_task, "Pro Audio");
    assert_eq!(t.wait_mode, WaitMode::HybridSpin);
    assert_eq!(t.dt_source, DtSource::QpcOnly);
    assert!(t.use_mmcss);
    assert!(!t.skip_duplicate_reports);
    // `StickMode` survives only as the legacy migration shim; its default is still `None`.
    assert_eq!(StickMode::default(), StickMode::None);
    assert_eq!(RcMode::default(), RcMode::UltimateDt);
}

#[test]
fn clamped_pins_rc_ranges_in_resolved_profile() {
    // Out-of-range RC params on the assigned profile must be clamped when `clamped()` rebuilds the
    // hot-facing `resolved` cache (resolve() clamps each stage internally).
    let mut ls_profile = Profile {
        name: "default".to_string(),
        ..Profile::default()
    };
    ls_profile.ls.rc.period_us = 50_000; // over MAX_PERIOD_US (8000)
    ls_profile.ls.rc.fixed_param = 9_000; // over MAX_PARAM (500)
    ls_profile.rs.rc.period_us = 10; // under MIN_PERIOD_US (1000)
    ls_profile.rs.rc.fixed_param = -9_000; // under MIN_PARAM (-500)

    let mut profiles = BTreeMap::new();
    profiles.insert("default".to_string(), ls_profile);

    let mut assignments = BTreeMap::new();
    assignments.insert("x".to_string(), "default".to_string());

    let cfg = EngineConfig {
        active_device: "x".to_string(),
        profiles: Arc::new(profiles),
        assignments,
        ..EngineConfig::default()
    };

    let clamped = cfg.clamped();
    let rp = clamped.resolved.get("x").expect("assigned device resolved");
    assert_eq!(rp.ls.rc.period_us, 8000);
    assert_eq!(rp.ls.rc.fixed_param, 500);
    assert_eq!(rp.rs.rc.period_us, 1000);
    assert_eq!(rp.rs.rc.fixed_param, -500);
}

#[test]
fn legacy_unknown_stick_mode_migrates_with_passthrough() {
    // An OLD-shape config with a legacy stick table whose `mode` is garbage migrates to a
    // synthesized "default" profile reading rc_mode_on = false (the StickMode::None fallback).
    let text = r#"
active_device = "x"
[devices.x]
[devices.x.ls]
mode = "OrbitAimAssist"
"#;
    let cfg = load_toml(text).expect("unknown enum string must not error");
    let profile = cfg.profiles.get("default").expect("synthesized profile");
    assert!(!profile.ls.rc_mode_on);
    assert_eq!(
        cfg.assignments.get("x").map(String::as_str),
        Some("default")
    );
}

#[test]
fn unknown_dt_source_and_wait_mode_fall_back() {
    let text = r#"
active_device = "x"
[thread]
dt_source = "SomeFutureClock"
wait_mode = "TurboSpin"
[devices.x]
"#;
    let cfg = load_toml(text).expect("unknown thread enums must not error");
    assert_eq!(cfg.thread.dt_source, DtSource::QpcOnly);
    assert_eq!(cfg.thread.wait_mode, WaitMode::HybridSpin);
}

#[test]
fn partial_toml_fills_missing_with_defaults() {
    // Only active_device given; everything else defaults (no legacy sticks => no migration).
    let cfg = load_toml(r#"active_device = "solo""#).expect("partial config parses");
    assert_eq!(cfg.active_device, "solo");
    assert_eq!(cfg.thread, ThreadConfig::default());
    assert_eq!(cfg.hidhide, Default::default());
    assert!(cfg.devices.is_empty());
    assert!(cfg.profiles.is_empty());
    assert!(cfg.assignments.is_empty());
}

#[test]
fn known_rc_mode_strings_migrate_in_pascal_case() {
    // The legacy `[..].ls.rc` RcMode string parses in PascalCase and is carried into the
    // synthesized profile's RC settings.
    for (s, expected) in [
        ("FireBirdInteger", RcMode::FireBirdInteger),
        ("UltimateLegacy", RcMode::UltimateLegacy),
        ("UltimateDt", RcMode::UltimateDt),
    ] {
        let text = format!(
            "active_device = \"x\"\n[devices.x]\n[devices.x.ls]\nmode = \"Rc\"\n\
             [devices.x.ls.rc]\nenabled = true\nmode = \"{s}\"\n"
        );
        let cfg = load_toml(&text).expect("known rc mode must parse");
        let profile = cfg.profiles.get("default").expect("synthesized profile");
        assert!(profile.ls.rc_mode_on, "legacy mode=Rc => rc_mode_on");
        assert_eq!(profile.ls.rc.mode, expected, "mode string {s}");
    }
}

#[test]
fn missing_rc_mode_takes_default_through_migration() {
    // A legacy rc table that omits `mode` falls back to the RcMode default (UltimateDt) via the
    // `#[serde(default)]` field attribute, and the migration carries it into the default profile.
    let text = r#"
active_device = "x"
[devices.x]
[devices.x.ls]
mode = "Rc"
[devices.x.ls.rc]
enabled = true
period_us = 4000
"#;
    let cfg = load_toml(text).expect("rc table without mode must parse");
    let profile = cfg.profiles.get("default").expect("synthesized profile");
    assert_eq!(profile.ls.rc.mode, RcMode::UltimateDt);
}
