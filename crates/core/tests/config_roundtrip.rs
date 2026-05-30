//! Config serde tree: default round-trips losslessly through TOML, `clamped()` pins the RC
//! parameter ranges, and unknown enum strings fall back defensively (DESIGN §9).

use std::collections::BTreeMap;

use hyperion_core::config::{
    load_toml, to_toml, DeviceConfig, DtSource, EngineConfig, StickConfig, StickMode, ThreadConfig,
    WaitMode,
};
use hyperion_core::rc::{RcConfig, RcMode};

// `RcConfig` is not `PartialEq` (core contract), so config equality is checked by comparing
// the re-serialized TOML: `load(to_toml(x))` must serialize back to the same text as `x`.
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
fn populated_config_round_trips() {
    let mut devices = BTreeMap::new();
    devices.insert(
        "dse_primary".to_string(),
        DeviceConfig {
            vid: 0x054C,
            pid: 0x0DF2,
            report_rate_hz: 4000,
            stick_bits: 8,
            ls: StickConfig {
                mode: StickMode::Rc,
                rc: RcConfig {
                    enabled: true,
                    mode: RcMode::UltimateDt,
                    use_dynamic_curve: false,
                    period_us: 4000,
                    fixed_param: 100,
                    ..RcConfig::default()
                },
            },
            rs: StickConfig {
                mode: StickMode::Rc,
                rc: RcConfig::default(),
            },
        },
    );
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
    assert_eq!(StickMode::default(), StickMode::None);
    assert_eq!(RcMode::default(), RcMode::UltimateDt);
}

#[test]
fn clamped_pins_rc_ranges() {
    let mut devices = BTreeMap::new();
    devices.insert(
        "x".to_string(),
        DeviceConfig {
            ls: StickConfig {
                mode: StickMode::Rc,
                rc: RcConfig {
                    period_us: 50_000,  // over MAX_PERIOD_US (8000)
                    fixed_param: 9_000, // over MAX_PARAM (500)
                    ..RcConfig::default()
                },
            },
            rs: StickConfig {
                mode: StickMode::Rc,
                rc: RcConfig {
                    period_us: 10,       // under MIN_PERIOD_US (1000)
                    fixed_param: -9_000, // under MIN_PARAM (-500)
                    ..RcConfig::default()
                },
            },
            ..DeviceConfig::default()
        },
    );
    let cfg = EngineConfig {
        active_device: "x".to_string(),
        devices,
        ..EngineConfig::default()
    };

    let clamped = cfg.clamped();
    let ls = clamped.devices["x"].ls.rc;
    let rs = clamped.devices["x"].rs.rc;
    assert_eq!(ls.period_us, 8000);
    assert_eq!(ls.fixed_param, 500);
    assert_eq!(rs.period_us, 1000);
    assert_eq!(rs.fixed_param, -500);
}

#[test]
fn unknown_stick_mode_falls_back_to_none() {
    let text = r#"
active_device = "x"
[devices.x]
[devices.x.ls]
mode = "OrbitAimAssist"
"#;
    let cfg = load_toml(text).expect("unknown enum string must not error");
    assert_eq!(cfg.devices["x"].ls.mode, StickMode::None);
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
    // Only active_device given; everything else defaults.
    let cfg = load_toml(r#"active_device = "solo""#).expect("partial config parses");
    assert_eq!(cfg.active_device, "solo");
    assert_eq!(cfg.thread, ThreadConfig::default());
    assert_eq!(cfg.hidhide, Default::default());
    assert!(cfg.devices.is_empty());
}

#[test]
fn known_rc_mode_strings_parse_in_pascal_case() {
    // The RcMode enum serializes/parses in PascalCase under the [..].rc table.
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
        assert_eq!(cfg.devices["x"].ls.rc.mode, expected, "mode string {s}");
    }
}

#[test]
fn missing_rc_mode_takes_default() {
    // An rc table that omits `mode` falls back to the RcMode default (UltimateDt) via
    // the `#[serde(default)]` field attribute, without erroring.
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
    assert_eq!(cfg.devices["x"].ls.rc.mode, RcMode::UltimateDt);
}
