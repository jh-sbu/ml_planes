//! International Standard Atmosphere (ISA) density model.
//!
//! Air density thins with altitude, reducing the dynamic pressure
//! `q̄ = ½·ρ·V²` that drives every aerodynamic force, and (for air-breathing
//! engines) the available thrust. This module is the single source of truth for
//! `ρ(altitude)` and the standard atmospheric constants it derives from.

/// Sea-level air density, ρ₀ [kg/m³].
pub const SEA_LEVEL_DENSITY: f32 = 1.225;
/// Sea-level temperature, T₀ [K].
pub const SEA_LEVEL_TEMP: f32 = 288.15;
/// Sea-level pressure, p₀ [Pa].
pub const SEA_LEVEL_PRESSURE: f32 = 101_325.0;
/// Tropospheric temperature lapse rate, L [K/m].
pub const LAPSE_RATE: f32 = 0.0065;
/// Specific gas constant for dry air, R [J/(kg·K)].
pub const GAS_CONSTANT_AIR: f32 = 287.05;
/// Standard gravity used by the ISA model, g [m/s²].
pub const GRAVITY_ISA: f32 = 9.80665;
/// Tropopause altitude (top of the troposphere) [m].
pub const TROPOPAUSE_ALT: f32 = 11_000.0;
/// Tropopause temperature (constant through the lower stratosphere) [K].
pub const TROPOPAUSE_TEMP: f32 = 216.65;

/// Air density [kg/m³] at the given geopotential altitude [m], per the ISA.
///
/// - `h ≤ 0`        → sea-level density (the world floor is the y=0 death plane;
///   guards against negative altitudes producing nonsense).
/// - `0 < h ≤ 11 km` → troposphere: `ρ = ρ₀·(T/T₀)^(g/(L·R) − 1)`.
/// - `h > 11 km`     → isothermal stratosphere: `ρ = ρ₁₁·exp(−g·(h−11km)/(R·T₁₁))`.
pub fn air_density(altitude_m: f32) -> f32 {
    if altitude_m <= 0.0 {
        return SEA_LEVEL_DENSITY;
    }

    if altitude_m <= TROPOPAUSE_ALT {
        // Troposphere: linear temperature lapse + barometric power law.
        let temp = SEA_LEVEL_TEMP - LAPSE_RATE * altitude_m;
        let exponent = GRAVITY_ISA / (LAPSE_RATE * GAS_CONSTANT_AIR) - 1.0;
        SEA_LEVEL_DENSITY * (temp / SEA_LEVEL_TEMP).powf(exponent)
    } else {
        // Isothermal lower stratosphere: exponential decay from the tropopause.
        let rho_tropopause = air_density(TROPOPAUSE_ALT);
        let scale = -GRAVITY_ISA / (GAS_CONSTANT_AIR * TROPOPAUSE_TEMP);
        rho_tropopause * (scale * (altitude_m - TROPOPAUSE_ALT)).exp()
    }
}

/// Density ratio ρ(h)/ρ₀ — used to scale thrust with altitude.
pub fn density_ratio(altitude_m: f32) -> f32 {
    air_density(altitude_m) / SEA_LEVEL_DENSITY
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn density_at_sea_level_is_1225() {
        assert!((air_density(0.0) - 1.225).abs() < 1e-3);
    }

    #[test]
    fn density_at_tropopause() {
        // ISA tabulated ρ(11 km) ≈ 0.3639 kg/m³.
        assert!(
            (air_density(11_000.0) - 0.3639).abs() < 1e-3,
            "got {}",
            air_density(11_000.0)
        );
    }

    #[test]
    fn density_at_5km() {
        // ISA tabulated ρ(5 km) ≈ 0.7364 kg/m³.
        assert!(
            (air_density(5_000.0) - 0.7364).abs() < 3e-3,
            "got {}",
            air_density(5_000.0)
        );
    }

    #[test]
    fn density_monotonically_decreases() {
        let mut prev = air_density(0.0);
        for h in (500..=20_000).step_by(500) {
            let rho = air_density(h as f32);
            assert!(
                rho < prev,
                "density not decreasing at {h} m: {rho} >= {prev}"
            );
            prev = rho;
        }
    }

    #[test]
    fn stratosphere_continuous_at_tropopause() {
        let below = air_density(TROPOPAUSE_ALT - 1.0);
        let above = air_density(TROPOPAUSE_ALT + 1.0);
        assert!(
            (below - above).abs() < 1e-3,
            "discontinuity at tropopause: {below} vs {above}"
        );
    }

    #[test]
    fn density_ratio_is_one_at_sea_level() {
        assert!((density_ratio(0.0) - 1.0).abs() < 1e-4);
    }

    #[test]
    fn negative_altitude_clamps_to_sea_level() {
        assert!((air_density(-500.0) - SEA_LEVEL_DENSITY).abs() < 1e-6);
    }
}
