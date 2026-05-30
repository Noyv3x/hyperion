//! The system-tray icon + context menu, integrated into the shared eframe/winit event loop.
//!
//! On Windows the tray's menu/click events are delivered to the same thread that pumps the winit
//! message loop, so we do **not** spawn a separate event loop: we build the [`TrayIcon`] inside
//! the eframe creation closure (on the winit thread) and poll
//! [`tray_icon::menu::MenuEvent::receiver`] each GUI frame from
//! [`super::HyperionApp::update`].
//!
//! The menu offers **Show**, **Hide**, and **Quit**. Show/Hide toggle the window's viewport
//! visibility via `egui::ViewportCommand::Visible`; Quit sets the process-wide quit flag
//! ([`super::request_quit`]) which the next `update` turns into a `ViewportCommand::Close`, after
//! which `main` runs the engine shutdown.

use eframe::egui;
use tray_icon::menu::{Menu, MenuEvent, MenuId, MenuItem, PredefinedMenuItem};
use tray_icon::{TrayIcon, TrayIconBuilder};

/// Owns the live tray icon and the ids of its menu items so received [`MenuEvent`]s can be routed.
pub struct TrayState {
    /// Kept alive for the program's lifetime; dropping it removes the icon from the tray.
    _icon: TrayIcon,
    show_id: MenuId,
    hide_id: MenuId,
    quit_id: MenuId,
}

impl TrayState {
    /// Build the tray icon + Show/Hide/Quit menu. Returns `None` (and the GUI runs tray-less) if
    /// the platform refuses to create a tray icon.
    pub fn build() -> Option<Self> {
        let menu = Menu::new();
        let show = MenuItem::new("Show", true, None);
        let hide = MenuItem::new("Hide", true, None);
        let quit = MenuItem::new("Quit", true, None);

        // Layout: Show / Hide / --- / Quit.
        menu.append(&show).ok()?;
        menu.append(&hide).ok()?;
        menu.append(&PredefinedMenuItem::separator()).ok()?;
        menu.append(&quit).ok()?;

        let show_id = show.id().clone();
        let hide_id = hide.id().clone();
        let quit_id = quit.id().clone();

        let icon = tray_icon::Icon::from_rgba(solid_icon_rgba(), ICON_SIDE, ICON_SIDE).ok()?;

        let tray = TrayIconBuilder::new()
            .with_menu(Box::new(menu))
            .with_tooltip("Hyperion")
            .with_icon(icon)
            .build()
            .ok()?;

        Some(Self {
            _icon: tray,
            show_id,
            hide_id,
            quit_id,
        })
    }

    /// Drain pending tray menu events and act on them (called once per GUI frame).
    ///
    /// `ctx` is used to send viewport visibility commands for Show/Hide. Quit routes through the
    /// shared quit flag so the close happens uniformly in [`super::HyperionApp::update`].
    pub fn handle_events(&self, ctx: &egui::Context) {
        while let Ok(event) = MenuEvent::receiver().try_recv() {
            if event.id == self.show_id {
                ctx.send_viewport_cmd(egui::ViewportCommand::Visible(true));
                ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
            } else if event.id == self.hide_id {
                ctx.send_viewport_cmd(egui::ViewportCommand::Visible(false));
            } else if event.id == self.quit_id {
                super::request_quit();
            }
        }
    }
}

/// Tray icon edge length in pixels.
const ICON_SIDE: u32 = 16;

/// Generate a simple solid RGBA icon (an amber square) so the tray has a recognizable mark
/// without shipping an image asset. Length is `ICON_SIDE * ICON_SIDE * 4` RGBA bytes.
fn solid_icon_rgba() -> Vec<u8> {
    let mut buf = Vec::with_capacity((ICON_SIDE * ICON_SIDE * 4) as usize);
    for y in 0..ICON_SIDE {
        for x in 0..ICON_SIDE {
            // A filled amber square with a 1px transparent border for a cleaner silhouette.
            let edge = x == 0 || y == 0 || x == ICON_SIDE - 1 || y == ICON_SIDE - 1;
            if edge {
                buf.extend_from_slice(&[0, 0, 0, 0]);
            } else {
                buf.extend_from_slice(&[255, 176, 64, 255]);
            }
        }
    }
    buf
}
