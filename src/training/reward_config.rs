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
    pub pitch_rate_reward_scale: f32,
    pub pitch_rate_reward_weight: f32,
    pub roll_rate_reward_scale: f32,
    pub roll_rate_reward_weight: f32,
    pub yaw_rate_reward_scale: f32,
    pub yaw_rate_reward_weight: f32,
    pub alive_reward: f32,
    pub terminal_failure_penalty: f32,
    pub min_altitude: f32,
    pub max_altitude_error: f32,
    pub min_airspeed: f32,
    pub max_episode_steps: u32,
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
            pitch_rate_reward_scale: 1.0,
            pitch_rate_reward_weight: 0.3,
            roll_rate_reward_scale: 1.0,
            roll_rate_reward_weight: 0.15,
            yaw_rate_reward_scale: 1.0,
            yaw_rate_reward_weight: 0.05,
            alive_reward: 0.01,
            terminal_failure_penalty: -25.0,
            min_altitude: 10.0,
            max_altitude_error: 700.0,
            min_airspeed: 20.0,
            max_episode_steps: 3_600,
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
            pitch_rate_reward_scale: 1.0,
            pitch_rate_reward_weight: 0.3,
            roll_rate_reward_scale: 1.0,
            roll_rate_reward_weight: 0.15,
            yaw_rate_reward_scale: 1.0,
            yaw_rate_reward_weight: 0.05,
            alive_reward: 0.01,
            terminal_failure_penalty: -25.0,
            min_altitude: 10.0,
            max_altitude_error: 700.0,
            min_airspeed: 20.0,
            max_episode_steps: 3600,
        )"#;
        let cfg: OrbitRewardConfig = ron::de::from_str(src).expect("parse failed");
        assert_eq!(cfg.terminal_failure_penalty, -25.0);
        assert_eq!(cfg.max_episode_steps, 3600);
    }
}
