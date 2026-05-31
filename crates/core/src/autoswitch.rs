//! Auto-profile-switch matcher — the pure foreground→profile rule resolver (blueprint §7.4, §12 M5).
//!
//! Pure, allocation-free*, OS-free (Linux-CI-tested). The `cfg(windows)` `ForegroundWatcher`
//! (engine/supervisor) reads the foreground window's executable path + title at `auto_switch.poll_hz`
//! (~4 Hz) and calls [`match_rules`] to pick the profile to switch to. On a change it sends
//! `ControlMsg::SetActiveProfile{device, name}` to the single config writer — **never on the hot
//! path** (the hot loop only sees the resulting generation bump). A failed/elevated-process read
//! yields empty strings → no match → the current profile is kept (the watcher just doesn't switch).
//!
//! \* The case-insensitive compare lower-cases via [`char::to_ascii_lowercase`] on the fly (no
//! `String` allocation), so the matcher itself never touches the heap.
//!
//! ## Rule semantics (ground truth: DS4Windows `AutoProfileChecker` / the `AutoProfiles.xml`
//! concept, reconciled to the single-TOML [`AutoSwitchRule`] in [`crate::config`])
//!
//! * **First match wins.** Rules are an ordered list; the first rule that matches the foreground
//!   info returns its profile. This mirrors DS4Windows evaluating its auto-profile list top-down and
//!   taking the first program/title hit.
//! * **Case-insensitive substring.** Each rule's `exe_substr` / `title_substr` is matched as an
//!   ASCII-case-insensitive substring of the foreground exe path / window title. An **empty**
//!   substring field is "don't care" (ignored), matching DS4Windows treating a blank match key as
//!   not-a-constraint.
//! * **Both fields are an AND.** A rule with both `exe_substr` and `title_substr` set requires BOTH
//!   to match. A rule with neither set never matches (a blank rule is inert, not a catch-all) — this
//!   avoids a stray empty rule hijacking every window.
//! * **Per-device scoping is the caller's job.** [`match_rules`] takes the already-device-filtered
//!   slice; the [`AutoSwitchRule::device`] field is *not* consulted here. The watcher pre-filters by
//!   device (see [`match_rules_for_device`] for the convenience wrapper that does it inline).

use crate::config::AutoSwitchRule;

/// ASCII-case-insensitive substring test, allocation-free.
///
/// Returns `true` if `needle` occurs in `haystack` ignoring ASCII case. An **empty** `needle` is
/// "not a constraint" and returns `true` (the caller decides whether an all-empty rule should
/// match — [`match_rules`] additionally requires at least one non-empty field per rule). Non-ASCII
/// bytes compare by their raw value (case-folding is ASCII-only, which is sufficient for Windows
/// exe paths / Latin window titles and keeps the matcher pure + alloc-free).
#[must_use]
pub fn contains_ignore_ascii_case(haystack: &str, needle: &str) -> bool {
    if needle.is_empty() {
        return true;
    }
    let h = haystack.as_bytes();
    let n = needle.as_bytes();
    if n.len() > h.len() {
        return false;
    }
    // Slide a window of len(needle) over haystack, comparing case-insensitively (ASCII).
    let last = h.len() - n.len();
    let mut i = 0;
    while i <= last {
        let mut j = 0;
        while j < n.len() {
            if !h[i + j].eq_ignore_ascii_case(&n[j]) {
                break;
            }
            j += 1;
        }
        if j == n.len() {
            return true;
        }
        i += 1;
    }
    false
}

/// `true` if `rule` (already device-scoped) matches the foreground `exe` path + window `title`.
///
/// A rule matches when **every non-empty** match field is an ASCII-case-insensitive substring of
/// the corresponding foreground field, and **at least one** match field is non-empty. A rule with
/// both `exe_substr` and `title_substr` empty never matches (a blank rule is inert).
#[must_use]
pub fn rule_matches(rule: &AutoSwitchRule, exe: &str, title: &str) -> bool {
    let has_exe = !rule.exe_substr.is_empty();
    let has_title = !rule.title_substr.is_empty();
    if !has_exe && !has_title {
        // A rule with no constraints is inert — it must not hijack every foreground window.
        return false;
    }
    let exe_ok = !has_exe || contains_ignore_ascii_case(exe, &rule.exe_substr);
    let title_ok = !has_title || contains_ignore_ascii_case(title, &rule.title_substr);
    exe_ok && title_ok
}

/// First-match-wins foreground→profile resolver (blueprint §7.4, §12 M5).
///
/// Walks `rules` in order and returns the `profile` id of the **first** rule that matches the
/// foreground executable path `exe` and window `title` (case-insensitive substring; empty match
/// fields are "don't care"; an all-empty rule is inert). Returns `None` when no rule matches (the
/// watcher keeps the current profile). **Per-device scoping is the caller's responsibility** — pass
/// the device-filtered slice (or use [`match_rules_for_device`]).
///
/// The returned `&str` borrows from `rules`, so no allocation occurs.
#[must_use]
pub fn match_rules<'a>(rules: &'a [AutoSwitchRule], exe: &str, title: &str) -> Option<&'a str> {
    rules
        .iter()
        .find(|r| rule_matches(r, exe, title))
        .map(|r| r.profile.as_str())
}

/// Device-scoped convenience wrapper around [`match_rules`].
///
/// Considers only rules whose [`AutoSwitchRule::device`] is empty (any device) **or** equals
/// `device`, preserving the first-match-wins order across the *original* rule list (so a global rule
/// listed before a device-specific one still wins if it matches first). This is the form the
/// `ForegroundWatcher` calls per active device.
#[must_use]
pub fn match_rules_for_device<'a>(
    rules: &'a [AutoSwitchRule],
    device: &str,
    exe: &str,
    title: &str,
) -> Option<&'a str> {
    rules
        .iter()
        .filter(|r| r.device.is_empty() || r.device == device)
        .find(|r| rule_matches(r, exe, title))
        .map(|r| r.profile.as_str())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rule(exe: &str, title: &str, profile: &str) -> AutoSwitchRule {
        AutoSwitchRule {
            device: String::new(),
            exe_substr: exe.to_string(),
            title_substr: title.to_string(),
            profile: profile.to_string(),
        }
    }

    #[test]
    fn contains_ignore_case_basic() {
        assert!(contains_ignore_ascii_case(
            "C:/Games/Valorant.exe",
            "valorant"
        ));
        assert!(contains_ignore_ascii_case("VALORANT", "valorant"));
        assert!(contains_ignore_ascii_case("valorant", "VALORANT"));
        assert!(!contains_ignore_ascii_case("csgo.exe", "valorant"));
        // Empty needle is "not a constraint".
        assert!(contains_ignore_ascii_case("anything", ""));
        // Needle longer than haystack never matches.
        assert!(!contains_ignore_ascii_case("ab", "abc"));
        // Match at the very end.
        assert!(contains_ignore_ascii_case("path/to/app", "app"));
        // Match at the very start.
        assert!(contains_ignore_ascii_case("app/path", "app"));
    }

    #[test]
    fn exe_substr_first_match_wins() {
        // Two rules could both match by exe substring; the FIRST one in the list wins.
        let rules = [
            rule("game", "", "first"),
            rule("game", "", "second"),
            rule("valorant", "", "third"),
        ];
        assert_eq!(
            match_rules(&rules, "C:/MyGame/game.exe", "Some Window"),
            Some("first")
        );
        // And a later, more specific rule wins only when the earlier ones don't match.
        assert_eq!(
            match_rules(&rules, "C:/valorant.exe", "Some Window"),
            Some("third")
        );
    }

    #[test]
    fn title_substr_matches() {
        let rules = [rule("", "Counter-Strike", "cs")];
        assert_eq!(
            match_rules(&rules, "C:/steam/csgo.exe", "Counter-Strike 2"),
            Some("cs")
        );
        // No title hit -> None (exe is "don't care" here, but title is required).
        assert_eq!(match_rules(&rules, "C:/steam/csgo.exe", "Desktop"), None);
    }

    #[test]
    fn case_insensitive_exe_and_title() {
        let rules = [rule("VaLoRaNt", "RIOT", "fps")];
        assert_eq!(
            match_rules(&rules, "c:/riot games/valorant.EXE", "valorant - riot"),
            Some("fps")
        );
    }

    #[test]
    fn both_fields_are_anded() {
        // Rule requires BOTH exe AND title; only one matching is not enough.
        let rules = [rule("game.exe", "Ranked", "ranked")];
        assert_eq!(
            match_rules(&rules, "game.exe", "Ranked Lobby"),
            Some("ranked")
        );
        // exe matches but title does not -> no match.
        assert_eq!(match_rules(&rules, "game.exe", "Main Menu"), None);
        // title matches but exe does not -> no match.
        assert_eq!(match_rules(&rules, "other.exe", "Ranked Lobby"), None);
    }

    #[test]
    fn no_match_returns_none() {
        let rules = [rule("valorant", "", "fps"), rule("csgo", "", "cs")];
        assert_eq!(
            match_rules(&rules, "C:/desktop/explorer.exe", "Desktop"),
            None
        );
    }

    #[test]
    fn empty_rules_returns_none() {
        let rules: [AutoSwitchRule; 0] = [];
        assert_eq!(match_rules(&rules, "anything.exe", "Any Title"), None);
    }

    #[test]
    fn all_empty_rule_is_inert_not_a_catch_all() {
        // A rule with neither exe nor title set must NOT match every window (guards against a stray
        // blank rule hijacking the foreground).
        let rules = [rule("", "", "ghost"), rule("valorant", "", "fps")];
        assert_eq!(match_rules(&rules, "x.exe", "X"), None);
        assert_eq!(match_rules(&rules, "valorant.exe", "X"), Some("fps"));
    }

    #[test]
    fn per_device_scoping_filters_rules() {
        let mut global = rule("game", "", "global-profile");
        global.device = String::new(); // any device
        let mut dev_a = rule("game", "", "a-profile");
        dev_a.device = "padA".to_string();
        let mut dev_b = rule("game", "", "b-profile");
        dev_b.device = "padB".to_string();

        // Global rule is first -> wins for any device (first-match-wins across the original order).
        let rules = [global, dev_a, dev_b];
        assert_eq!(
            match_rules_for_device(&rules, "padA", "game.exe", "W"),
            Some("global-profile")
        );

        // Drop the global rule: padA sees only its own rule; padB sees only its own.
        let rules2 = [rules[1].clone(), rules[2].clone()];
        assert_eq!(
            match_rules_for_device(&rules2, "padA", "game.exe", "W"),
            Some("a-profile")
        );
        assert_eq!(
            match_rules_for_device(&rules2, "padB", "game.exe", "W"),
            Some("b-profile")
        );
        // A device with no matching rule -> None.
        assert_eq!(
            match_rules_for_device(&rules2, "padC", "game.exe", "W"),
            None
        );
    }

    #[test]
    fn returned_str_borrows_profile() {
        // The returned &str is the rule's profile id (borrow, no allocation).
        let rules = [rule("app", "", "my-profile")];
        let got = match_rules(&rules, "app.exe", "T").unwrap();
        assert_eq!(got, "my-profile");
    }
}
