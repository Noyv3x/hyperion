//! The `Control` mapping-table key — one dense `#[repr(u8)]` enum mirroring `DS4Controls`.
//!
//! `Control` is the single index for every `[T; Control::COUNT]` table the mapping engine
//! keeps (bindings, turbo state, per-control shift). Stick half-axes and triggers appear as
//! BOTH analog axes/triggers and digital/directional controls so a direction can map
//! independently (left-stick-left → key while left-stick-right stays passthrough).
//!
//! The discriminant ordering is **append-only** (`Control` is a serde key and a persisted
//! table index): never renumber an existing variant; only add below the marked line. Variants
//! whose physical bit is not yet decoded (DualSense Edge Fn/paddles, touch regions, gyro
//! directions) are valid table indices immediately but read `false`/`0.0` until their decode
//! lands (gated by [`SourceMeta`](crate::input::SourceMeta) capability, blueprint §3.5).

/// The mapping-table key: a dense `#[repr(u8)]` enum mirroring DS4Windows' `DS4Controls`.
///
/// Every value in `0..COUNT` is a valid index; [`ALL`](Self::ALL) enumerates them in
/// discriminant order. Use [`as_index`](Self::as_index) to index `[T; Control::COUNT]`.
#[repr(u8)]
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "PascalCase")]
pub enum Control {
    None = 0,
    // Analog stick half-axes (AxisDir) — split so a direction maps independently. 1..=8.
    LxNeg,
    LxPos,
    LyNeg,
    LyPos,
    RxNeg,
    RxPos,
    RyNeg,
    RyPos,
    // Shoulders / analog triggers / stick clicks. L2/R2 are the analog triggers.
    L1,
    L2,
    L3,
    R1,
    R2,
    R3,
    // Face buttons.
    Square,
    Triangle,
    Circle,
    Cross,
    // D-pad.
    DpadUp,
    DpadRight,
    DpadDown,
    DpadLeft,
    // System.
    Ps,
    Share,
    Options,
    Mute,
    Capture,
    // DualSense Edge (gated by SourceMeta capability; see §3.5 RISK).
    FnL,
    FnR,
    Blp,
    Brp,
    SideL,
    SideR,
    // Trigger-as-button (full pull == 255).
    L2FullPull,
    R2FullPull,
    // Stick outer-ring as trigger.
    LsOuter,
    RsOuter,
    // Touchpad click + finger-region buttons.
    TouchButton,
    TouchLeft,
    TouchRight,
    TouchUpper,
    TouchMulti,
    // Gyro directions (gyro-as-button / gyro→mouse trigger).
    GyroXPos,
    GyroXNeg,
    GyroZPos,
    GyroZNeg,
    // APPEND-ONLY below this line (serde key / table index stability); never renumber the above.
}

/// The kind of a [`Control`], which selects the digital-activation [`Thresholds`] to apply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlKind {
    /// A plain digital button (read directly as pressed/released).
    Button,
    /// A stick half-axis (digitized against [`Thresholds::stick_dir`]).
    AxisDir,
    /// An analog trigger (digitized against [`Thresholds::trigger`]).
    Trigger,
    /// A touchpad click / finger-region control.
    Touch,
    /// A gyro direction (digitized against [`Thresholds::gyro_dir`]).
    GyroDir,
}

impl Control {
    /// Number of variants — the size of every `[T; Control::COUNT]` mapping table.
    pub const COUNT: usize = Self::GyroZNeg as usize + 1;

    /// This control's dense table index (`self as usize`).
    #[inline]
    pub const fn as_index(self) -> usize {
        self as usize
    }

    /// Classify this control for kind-dependent threshold selection (verifier FIX 3).
    pub fn kind(self) -> ControlKind {
        use Control::*;
        match self {
            LxNeg | LxPos | LyNeg | LyPos | RxNeg | RxPos | RyNeg | RyPos => ControlKind::AxisDir,
            L2 | R2 | L2FullPull | R2FullPull | LsOuter | RsOuter => ControlKind::Trigger,
            TouchButton | TouchLeft | TouchRight | TouchUpper | TouchMulti => ControlKind::Touch,
            GyroXPos | GyroXNeg | GyroZPos | GyroZNeg => ControlKind::GyroDir,
            // None plus every plain digital button.
            _ => ControlKind::Button,
        }
    }

    /// For half-axis identity suppression (verifier FIX 1, `ResetToDefaultValue`): which
    /// [`OutputState`](crate::output::OutputState) stick axis this control belongs to
    /// (`0`=Lx, `1`=Ly, `2`=Rx, `3`=Ry), or `None` for non-stick controls.
    pub fn stick_axis(self) -> Option<usize> {
        use Control::*;
        match self {
            LxNeg | LxPos => Some(0),
            LyNeg | LyPos => Some(1),
            RxNeg | RxPos => Some(2),
            RyNeg | RyPos => Some(3),
            // `use Control::*` shadows `Option::None` with `Control::None`, so disambiguate.
            _ => Option::None,
        }
    }

    /// Every control in dense discriminant order (`ALL[i].as_index() == i`).
    pub const ALL: [Control; Self::COUNT] = {
        use Control::*;
        [
            None,
            LxNeg,
            LxPos,
            LyNeg,
            LyPos,
            RxNeg,
            RxPos,
            RyNeg,
            RyPos,
            L1,
            L2,
            L3,
            R1,
            R2,
            R3,
            Square,
            Triangle,
            Circle,
            Cross,
            DpadUp,
            DpadRight,
            DpadDown,
            DpadLeft,
            Ps,
            Share,
            Options,
            Mute,
            Capture,
            FnL,
            FnR,
            Blp,
            Brp,
            SideL,
            SideR,
            L2FullPull,
            R2FullPull,
            LsOuter,
            RsOuter,
            TouchButton,
            TouchLeft,
            TouchRight,
            TouchUpper,
            TouchMulti,
            GyroXPos,
            GyroXNeg,
            GyroZPos,
            GyroZNeg,
        ]
    };
}

/// Per-kind digital activation thresholds.
///
/// **Verifier FIX 3:** axes use `55/127` (`GetBoolMapping`'s `cState.LX < 128-55`), triggers
/// use `100/255` (`triggers > 100`) — these are NOT the same value, so they live in separate
/// fields keyed by [`ControlKind`].
#[derive(Clone, Copy, Debug)]
pub struct Thresholds {
    /// Stick half-axis activation point in `[0,1]` native units.
    pub stick_dir: f64,
    /// Analog-trigger activation point in `[0,1]` native units.
    pub trigger: f64,
    /// Gyro-direction activation point in native units.
    pub gyro_dir: f64,
}

impl Thresholds {
    /// DS4Windows axis digital threshold: `55/127 ≈ 0.4331` (`GetBoolMapping` `LX < 128-55`).
    pub const STICK_DIR_DEFAULT: f64 = 55.0 / 127.0;
    /// DS4Windows trigger digital threshold: `100/255 ≈ 0.3922` (`triggers > 100`).
    pub const TRIGGER_DEFAULT: f64 = 100.0 / 255.0;
}

impl Default for Thresholds {
    fn default() -> Self {
        Self {
            stick_dir: Self::STICK_DIR_DEFAULT,
            trigger: Self::TRIGGER_DEFAULT,
            gyro_dir: 0.5,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn count_is_last_variant_plus_one() {
        assert_eq!(Control::COUNT, Control::GyroZNeg as usize + 1);
        // None occupies index 0; COUNT covers every discriminant inclusively.
        assert_eq!(Control::None.as_index(), 0);
        assert_eq!(Control::GyroZNeg.as_index(), Control::COUNT - 1);
    }

    #[test]
    fn all_is_dense_and_ordered() {
        assert_eq!(Control::ALL.len(), Control::COUNT);
        for (i, c) in Control::ALL.iter().enumerate() {
            assert_eq!(c.as_index(), i, "ALL[{i}] = {c:?} is out of order");
        }
    }

    #[test]
    fn as_index_round_trips_through_all() {
        for c in Control::ALL {
            assert_eq!(Control::ALL[c.as_index()], c);
        }
    }

    #[test]
    fn discriminants_are_append_only_anchors() {
        // Pin a few load-bearing discriminants so an accidental reorder is caught.
        assert_eq!(Control::None as u8, 0);
        assert_eq!(Control::LxNeg as u8, 1);
        assert_eq!(Control::LxPos as u8, 2);
        assert_eq!(Control::Cross as u8, 18);
    }

    #[test]
    fn kind_classification() {
        assert_eq!(Control::None.kind(), ControlKind::Button);
        assert_eq!(Control::Cross.kind(), ControlKind::Button);
        assert_eq!(Control::DpadUp.kind(), ControlKind::Button);
        assert_eq!(Control::LxNeg.kind(), ControlKind::AxisDir);
        assert_eq!(Control::RyPos.kind(), ControlKind::AxisDir);
        assert_eq!(Control::L2.kind(), ControlKind::Trigger);
        assert_eq!(Control::R2.kind(), ControlKind::Trigger);
        assert_eq!(Control::L2FullPull.kind(), ControlKind::Trigger);
        assert_eq!(Control::LsOuter.kind(), ControlKind::Trigger);
        assert_eq!(Control::TouchButton.kind(), ControlKind::Touch);
        assert_eq!(Control::TouchMulti.kind(), ControlKind::Touch);
        assert_eq!(Control::GyroXPos.kind(), ControlKind::GyroDir);
        // L3/R3 are stick CLICKS (digital buttons), not axes.
        assert_eq!(Control::L3.kind(), ControlKind::Button);
        assert_eq!(Control::R3.kind(), ControlKind::Button);
    }

    #[test]
    fn stick_axis_pairs_both_halves() {
        assert_eq!(Control::LxNeg.stick_axis(), Some(0));
        assert_eq!(Control::LxPos.stick_axis(), Some(0));
        assert_eq!(Control::LyNeg.stick_axis(), Some(1));
        assert_eq!(Control::LyPos.stick_axis(), Some(1));
        assert_eq!(Control::RxNeg.stick_axis(), Some(2));
        assert_eq!(Control::RxPos.stick_axis(), Some(2));
        assert_eq!(Control::RyNeg.stick_axis(), Some(3));
        assert_eq!(Control::RyPos.stick_axis(), Some(3));
        assert_eq!(Control::Cross.stick_axis(), None);
        assert_eq!(Control::L2.stick_axis(), None);
        assert_eq!(Control::L3.stick_axis(), None);
    }

    #[test]
    fn serde_round_trips_pascal_case() {
        // PascalCase serde keys are the persisted form. Round-trip every variant through TOML
        // (the on-disk format), wrapped in a struct so the enum is a value.
        #[derive(serde::Serialize, serde::Deserialize, PartialEq, Debug)]
        struct W {
            c: Control,
        }
        for c in Control::ALL {
            let s = toml::to_string(&W { c }).expect("serialize");
            let back: W = toml::from_str(&s).expect("deserialize");
            assert_eq!(back.c, c, "round-trip failed for {c:?} ({s})");
        }
        // Pin the literal key spelling for a couple of variants.
        assert_eq!(
            toml::to_string(&W { c: Control::LxNeg }).unwrap().trim(),
            "c = \"LxNeg\""
        );
        assert_eq!(
            toml::to_string(&W { c: Control::Ps }).unwrap().trim(),
            "c = \"Ps\""
        );
        assert_eq!(
            toml::to_string(&W {
                c: Control::L2FullPull
            })
            .unwrap()
            .trim(),
            "c = \"L2FullPull\""
        );
    }

    #[test]
    fn threshold_defaults_match_ds4windows_constants() {
        let t = Thresholds::default();
        assert!((t.stick_dir - 55.0 / 127.0).abs() < 1e-15);
        assert!((t.trigger - 100.0 / 255.0).abs() < 1e-15);
        assert!((t.stick_dir - 0.433_070_866).abs() < 1e-6);
        assert!((t.trigger - 0.392_156_862).abs() < 1e-6);
        // The two constants are genuinely different (the verifier-FIX-3 point).
        assert!((t.stick_dir - t.trigger).abs() > 0.04);
    }
}
