//! Shared, render-neutral **simulation playback speed** state.
//!
//! Lives outside `ui/` (which is `#[cfg(feature = "visual")]`) so the value is
//! available to the network protocol (`SetSimSpeedCommand`) and the future
//! authoritative server, which scales `Time<Virtual>` server-side. The visual
//! HUD ([`crate::ui::time_control`]) re-exports and renders it.

use bevy::prelude::*;

/// Discrete simulation playback speeds selectable from the HUD.
#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub enum SimSpeed {
    /// Simulation frozen (`Time<Virtual>` paused).
    Paused,
    /// Real-time playback.
    #[default]
    X1,
    /// 5× real-time.
    X5,
    /// 10× real-time.
    X10,
}

impl SimSpeed {
    /// Virtual-time relative speed to apply when not paused.
    pub fn relative_speed(self) -> f32 {
        match self {
            SimSpeed::Paused => 1.0,
            SimSpeed::X1 => 1.0,
            SimSpeed::X5 => 5.0,
            SimSpeed::X10 => 10.0,
        }
    }

    /// Whether the clock should be frozen.
    pub fn is_paused(self) -> bool {
        matches!(self, SimSpeed::Paused)
    }

    /// Human-readable button label.
    pub fn label(self) -> &'static str {
        match self {
            SimSpeed::Paused => "Pause",
            SimSpeed::X1 => "1x",
            SimSpeed::X5 => "5x",
            SimSpeed::X10 => "10x",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_real_time() {
        assert_eq!(SimSpeed::default(), SimSpeed::X1);
    }

    #[test]
    fn relative_speeds() {
        assert_eq!(SimSpeed::X1.relative_speed(), 1.0);
        assert_eq!(SimSpeed::X5.relative_speed(), 5.0);
        assert_eq!(SimSpeed::X10.relative_speed(), 10.0);
    }

    #[test]
    fn only_paused_is_paused() {
        assert!(SimSpeed::Paused.is_paused());
        assert!(!SimSpeed::X1.is_paused());
        assert!(!SimSpeed::X5.is_paused());
        assert!(!SimSpeed::X10.is_paused());
    }

    #[test]
    fn labels() {
        assert_eq!(SimSpeed::Paused.label(), "Pause");
        assert_eq!(SimSpeed::X1.label(), "1x");
        assert_eq!(SimSpeed::X5.label(), "5x");
        assert_eq!(SimSpeed::X10.label(), "10x");
    }
}
