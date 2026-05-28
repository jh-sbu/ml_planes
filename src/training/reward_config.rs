use serde::{Deserialize, Serialize};

/// Reward weights, scales, and termination thresholds for `LevelHoldEnv`.
/// Loaded from `assets/training/level_hold.reward.ron` at training startup.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LevelHoldRewardConfig {
    pub alt_error_scale: f32,
    pub alt_error_weight: f32,
    pub speed_error_scale: f32,
    pub speed_error_weight: f32,
    pub roll_scale: f32,
    pub roll_weight: f32,
    pub beta_scale: f32,
    pub beta_weight: f32,
    pub alive_bonus: f32,
    pub min_altitude: f32,
    pub max_altitude_error: f32,
    pub max_episode_steps: u32,
}

impl LevelHoldRewardConfig {
    pub fn log_fields(&self) -> Vec<(&'static str, String)> {
        vec![
            ("alt_error_scale", self.alt_error_scale.to_string()),
            ("alt_error_weight", self.alt_error_weight.to_string()),
            ("speed_error_scale", self.speed_error_scale.to_string()),
            ("speed_error_weight", self.speed_error_weight.to_string()),
            ("roll_scale", self.roll_scale.to_string()),
            ("roll_weight", self.roll_weight.to_string()),
            ("beta_scale", self.beta_scale.to_string()),
            ("beta_weight", self.beta_weight.to_string()),
            ("alive_bonus", self.alive_bonus.to_string()),
            ("min_altitude", self.min_altitude.to_string()),
            ("max_altitude_error", self.max_altitude_error.to_string()),
            ("max_episode_steps", self.max_episode_steps.to_string()),
        ]
    }
}

impl Default for LevelHoldRewardConfig {
    fn default() -> Self {
        Self {
            alt_error_scale: 200.0,
            alt_error_weight: 1.0,
            speed_error_scale: 50.0,
            speed_error_weight: 0.5,
            roll_scale: 0.5,
            roll_weight: 0.3,
            beta_scale: 0.5,
            beta_weight: 0.1,
            alive_bonus: 0.01,
            min_altitude: 10.0,
            max_altitude_error: 500.0,
            max_episode_steps: 3_000,
        }
    }
}

/// Reward weights, scales, and termination thresholds for `OrbitEnv`.
/// Loaded from `assets/training/orbit.reward.ron` at training startup.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OrbitRewardConfig {
    pub radial_reward_scale: f32,
    pub radial_reward_weight: f32,
    pub heading_reward_scale: f32,
    pub heading_reward_weight: f32,
    pub altitude_reward_scale: f32,
    pub altitude_reward_weight: f32,
    pub speed_reward_scale: f32,
    pub speed_reward_weight: f32,
    pub roll_reward_scale: f32,
    pub roll_reward_weight: f32,
    pub beta_reward_scale: f32,
    pub beta_reward_weight: f32,
    pub pitch_accel_reward_scale: f32,
    pub pitch_accel_reward_weight: f32,
    pub roll_accel_reward_scale: f32,
    pub roll_accel_reward_weight: f32,
    pub yaw_accel_reward_scale: f32,
    pub yaw_accel_reward_weight: f32,
    pub alive_reward: f32,
    pub terminal_failure_penalty: f32,
    pub min_altitude: f32,
    pub max_altitude_error: f32,
    pub max_radial_error: f32,
    pub min_airspeed: f32,
    pub max_episode_steps: u32,
    pub residual_scale: f32,
}

impl OrbitRewardConfig {
    pub fn log_fields(&self) -> Vec<(&'static str, String)> {
        vec![
            ("radial_reward_scale", self.radial_reward_scale.to_string()),
            (
                "radial_reward_weight",
                self.radial_reward_weight.to_string(),
            ),
            (
                "heading_reward_scale",
                self.heading_reward_scale.to_string(),
            ),
            (
                "heading_reward_weight",
                self.heading_reward_weight.to_string(),
            ),
            (
                "altitude_reward_scale",
                self.altitude_reward_scale.to_string(),
            ),
            (
                "altitude_reward_weight",
                self.altitude_reward_weight.to_string(),
            ),
            ("speed_reward_scale", self.speed_reward_scale.to_string()),
            ("speed_reward_weight", self.speed_reward_weight.to_string()),
            ("roll_reward_scale", self.roll_reward_scale.to_string()),
            ("roll_reward_weight", self.roll_reward_weight.to_string()),
            ("beta_reward_scale", self.beta_reward_scale.to_string()),
            ("beta_reward_weight", self.beta_reward_weight.to_string()),
            (
                "pitch_accel_reward_scale",
                self.pitch_accel_reward_scale.to_string(),
            ),
            (
                "pitch_accel_reward_weight",
                self.pitch_accel_reward_weight.to_string(),
            ),
            (
                "roll_accel_reward_scale",
                self.roll_accel_reward_scale.to_string(),
            ),
            (
                "roll_accel_reward_weight",
                self.roll_accel_reward_weight.to_string(),
            ),
            (
                "yaw_accel_reward_scale",
                self.yaw_accel_reward_scale.to_string(),
            ),
            (
                "yaw_accel_reward_weight",
                self.yaw_accel_reward_weight.to_string(),
            ),
            ("alive_reward", self.alive_reward.to_string()),
            (
                "terminal_failure_penalty",
                self.terminal_failure_penalty.to_string(),
            ),
            ("min_altitude", self.min_altitude.to_string()),
            ("max_altitude_error", self.max_altitude_error.to_string()),
            ("max_radial_error", self.max_radial_error.to_string()),
            ("min_airspeed", self.min_airspeed.to_string()),
            ("max_episode_steps", self.max_episode_steps.to_string()),
            ("residual_scale", self.residual_scale.to_string()),
        ]
    }
}

impl Default for OrbitRewardConfig {
    fn default() -> Self {
        Self {
            radial_reward_scale: 500.0,
            radial_reward_weight: 1.4,
            heading_reward_scale: 0.5,
            heading_reward_weight: 1.2,
            altitude_reward_scale: 200.0,
            altitude_reward_weight: 1.0,
            speed_reward_scale: 50.0,
            speed_reward_weight: 0.4,
            roll_reward_scale: 0.8,
            roll_reward_weight: 0.1,
            beta_reward_scale: 0.5,
            beta_reward_weight: 0.1,
            pitch_accel_reward_scale: 5.0,
            pitch_accel_reward_weight: 0.3,
            roll_accel_reward_scale: 5.0,
            roll_accel_reward_weight: 0.15,
            yaw_accel_reward_scale: 5.0,
            yaw_accel_reward_weight: 0.05,
            alive_reward: 0.01,
            terminal_failure_penalty: -25.0,
            min_altitude: 10.0,
            max_altitude_error: 700.0,
            max_radial_error: 1000.0,
            min_airspeed: 20.0,
            max_episode_steps: 3_600,
            residual_scale: 0.3,
        }
    }
}

/// Load a reward config struct from a RON file on disk.
/// Falls back gracefully: callers may use `unwrap_or_else(|_| T::default())`.
pub fn load_reward_config<T: serde::de::DeserializeOwned>(
    path: &str,
) -> Result<T, Box<dyn std::error::Error>> {
    let file = std::fs::File::open(path)?;
    Ok(ron::de::from_reader(file)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_hold_log_fields_covers_all_fields() {
        let cfg = LevelHoldRewardConfig::default();
        let fields = cfg.log_fields();
        assert_eq!(fields.len(), 12);
        assert!(fields.iter().any(|(k, _)| *k == "alt_error_scale"));
        assert!(fields.iter().any(|(k, _)| *k == "max_episode_steps"));
    }

    #[test]
    fn orbit_log_fields_covers_all_fields() {
        let cfg = OrbitRewardConfig::default();
        let fields = cfg.log_fields();
        assert_eq!(fields.len(), 26);
        assert!(fields.iter().any(|(k, _)| *k == "radial_reward_scale"));
        assert!(fields.iter().any(|(k, _)| *k == "max_episode_steps"));
        assert!(fields.iter().any(|(k, _)| *k == "terminal_failure_penalty"));
    }

    #[test]
    fn level_hold_ron_parses() {
        let src = r#"(
            alt_error_scale: 200.0,
            alt_error_weight: 1.0,
            speed_error_scale: 50.0,
            speed_error_weight: 0.5,
            roll_scale: 0.5,
            roll_weight: 0.3,
            beta_scale: 0.5,
            beta_weight: 0.1,
            alive_bonus: 0.01,
            min_altitude: 10.0,
            max_altitude_error: 500.0,
            max_episode_steps: 3000,
        )"#;
        let cfg: LevelHoldRewardConfig = ron::de::from_str(src).expect("parse failed");
        assert_eq!(cfg.alt_error_scale, 200.0);
        assert_eq!(cfg.max_episode_steps, 3000);
    }

    #[test]
    fn orbit_ron_parses() {
        let src = r#"(
            radial_reward_scale: 500.0,
            radial_reward_weight: 1.4,
            heading_reward_scale: 0.5,
            heading_reward_weight: 1.2,
            altitude_reward_scale: 200.0,
            altitude_reward_weight: 1.0,
            speed_reward_scale: 50.0,
            speed_reward_weight: 0.4,
            roll_reward_scale: 0.8,
            roll_reward_weight: 0.1,
            beta_reward_scale: 0.5,
            beta_reward_weight: 0.1,
            pitch_accel_reward_scale: 5.0,
            pitch_accel_reward_weight: 0.3,
            roll_accel_reward_scale: 5.0,
            roll_accel_reward_weight: 0.15,
            yaw_accel_reward_scale: 5.0,
            yaw_accel_reward_weight: 0.05,
            alive_reward: 0.01,
            terminal_failure_penalty: -25.0,
            min_altitude: 10.0,
            max_altitude_error: 700.0,
            max_radial_error: 1000.0,
            min_airspeed: 20.0,
            max_episode_steps: 3600,
            residual_scale: 0.3,
        )"#;
        let cfg: OrbitRewardConfig = ron::de::from_str(src).expect("parse failed");
        assert_eq!(cfg.terminal_failure_penalty, -25.0);
        assert_eq!(cfg.max_episode_steps, 3600);
    }
}
