//! The mapping engine — the DS4Windows-class remapper layer.
//!
//! Three modules:
//! * [`binding`] — the closed, append-only [`BindTarget`] enum and the per-control [`BindingSlot`]
//!   (base bind + per-control shift trigger/bind + turbo).
//! * [`profile`] — the editable serde [`Profile`] and the hot-facing immutable [`ResolvedProfile`]
//!   (fixed arrays keyed by `Control`, no map/hash on the hot path).
//! * [`engine`] — [`apply`], the pure, alloc-free remap entry point, plus its resident
//!   [`MapState`].
//!
//! M3 [`apply`] resolves only `Passthrough` + `GamepadButton` + `Key` (button→button, button→key);
//! every other [`BindTarget`] variant exists in the enum but is a documented no-op until its
//! milestone, so shift/turbo/macro/mouse (M4) and DS4/gyro (M5) are purely additive.

pub mod binding;
pub mod engine;
pub mod profile;

pub use binding::GamepadAxis;
pub use binding::{
    AxisDir, BindTarget, Binding, BindingSlot, KeyKind, MouseMoveSrc, OutputBinding, PadBtn,
    ShiftTrigger, TurboCfg, WheelDir,
};
pub use engine::{apply, MapState, MouseAccumulator, TurboState, VkLatch};
pub use profile::{GyroSettings, MacroDef, MouseSettings, Profile, ResolvedProfile, SpecialAction};
