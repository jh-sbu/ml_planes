use bevy::math::Vec3;
use bevy::reflect::Reflect;
use serde::{Deserialize, Serialize};

/// Aviation fuel grade. Differences between kerosene grades are minor (<1% in both
/// mass-specific energy and density), so the grade does **not** enter the burn math —
/// fuel is consumed as mass (kg) directly. The enum exists for HUD labelling and to
/// leave room for future volume-limited / energy-based range modelling.
#[derive(Reflect, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum FuelType {
    JetA,
    Jp8,
    Jp5,
}

/// Physical properties of a fuel grade.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FuelProperties {
    pub density_kg_per_l: f32,
    pub specific_energy_mj_per_kg: f32,
}

impl FuelType {
    /// Density and mass-specific energy for this grade (typical published values).
    pub fn properties(self) -> FuelProperties {
        match self {
            FuelType::JetA => FuelProperties {
                density_kg_per_l: 0.804,
                specific_energy_mj_per_kg: 43.0,
            },
            FuelType::Jp8 => FuelProperties {
                density_kg_per_l: 0.810,
                specific_energy_mj_per_kg: 43.2,
            },
            FuelType::Jp5 => FuelProperties {
                density_kg_per_l: 0.810,
                specific_energy_mj_per_kg: 42.8,
            },
        }
    }

    /// Short human-readable label for the HUD.
    pub fn label(self) -> &'static str {
        match self {
            FuelType::JetA => "Jet A",
            FuelType::Jp8 => "JP-8",
            FuelType::Jp5 => "JP-5",
        }
    }
}

/// Energy store + consumption model for a plane's engine.
///
/// - [`Powerplant::JetFuel`] burns fuel **mass**: effective mass = empty mass + remaining
///   fuel, so the airframe gets lighter as it burns and flames out when empty.
/// - [`Powerplant::Electric`] depletes **charge** but its airframe mass is constant
///   (battery mass is baked into `PlaneConfig.mass`).
#[derive(Reflect, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub enum Powerplant {
    /// Combustion engine. `capacity_kg` = full tank [kg]; `tsfc` = thrust-specific fuel
    /// consumption [kg/(N·s)]; `fuel_type` selects the grade (display/future fidelity).
    JetFuel {
        capacity_kg: f32,
        tsfc: f32,
        fuel_type: FuelType,
    },
    /// Battery-electric. `capacity` = full charge [kWh]; `consumption` = charge drawn per
    /// unit thrust-second [kWh/(N·s)]. Mass does not vary with charge.
    Electric { capacity: f32, consumption: f32 },
}

impl Default for Powerplant {
    /// Matches the generic-jet airframe, so `PlaneConfig` literals / RON files without a
    /// `powerplant` field deserialize to a sensible jet powerplant.
    fn default() -> Self {
        Powerplant::JetFuel {
            capacity_kg: 2000.0,
            tsfc: 2.0e-5,
            fuel_type: FuelType::JetA,
        }
    }
}

impl Powerplant {
    /// Full consumable load (kg of fuel for jets, kWh of charge for electric).
    pub fn capacity(&self) -> f32 {
        match self {
            Powerplant::JetFuel { capacity_kg, .. } => *capacity_kg,
            Powerplant::Electric { capacity, .. } => *capacity,
        }
    }

    /// Whether the remaining consumable adds to airframe mass (true only for fuel).
    pub fn contributes_mass(&self) -> bool {
        matches!(self, Powerplant::JetFuel { .. })
    }

    /// Total flying mass given the airframe's empty/dry mass and the remaining
    /// consumable. Fuel adds on top; charge does not. A non-finite `remaining`
    /// (the [`FlightState`](crate::plane::FlightState) default sentinel for an
    /// unmodelled / unlimited tank) contributes no mass, preserving the constant-mass
    /// behaviour of code paths that never opt into the fuel model.
    pub fn effective_mass(&self, empty_mass: f32, remaining: f32) -> f32 {
        if self.contributes_mass() && remaining.is_finite() {
            empty_mass + remaining.max(0.0)
        } else {
            empty_mass
        }
    }

    /// Consumable consumed per second at the given engine thrust [N] (thrust-specific).
    pub fn burn_rate(&self, thrust: f32) -> f32 {
        match self {
            Powerplant::JetFuel { tsfc, .. } => tsfc * thrust.max(0.0),
            Powerplant::Electric { consumption, .. } => consumption * thrust.max(0.0),
        }
    }
}

/// Runtime-loaded plane configuration asset.
/// Field names must exactly match `assets/planes/*.plane.ron`.
#[derive(bevy::asset::Asset, Reflect, Serialize, Deserialize, Debug, Clone)]
pub struct PlaneConfig {
    // Geometry
    pub wing_area: f32,  // S  [m²]
    pub mean_chord: f32, // c̄  [m]
    pub wing_span: f32,  // b  [m]
    // Mass / Inertia
    pub mass: f32,     // [kg]
    pub inertia: Vec3, // Ixx, Iyy, Izz  [kg·m²]
    // Longitudinal aero
    pub cl0: f32,
    pub cl_alpha: f32,
    pub cl_delta_e: f32,
    pub cl_max: f32,
    pub cd0: f32,
    pub cd_induced: f32,
    pub cm0: f32,
    pub cm_alpha: f32,
    pub cm_q: f32,
    pub cm_delta_e: f32,
    // Lateral-directional aero
    pub cl_beta: f32,
    pub cl_p: f32,
    pub cl_r: f32,
    pub cl_delta_a: f32,
    pub cn_beta: f32,
    pub cn_r: f32,
    pub cn_delta_r: f32,
    // Engine
    pub thrust_max: f32, // [N]
    // Powerplant / consumable. `#[serde(default)]` lets older `.plane.ron` files (and
    // struct literals via `..`) omit it and fall back to the generic-jet powerplant.
    #[serde(default)]
    pub powerplant: Powerplant,
    // Control limits
    pub aileron_limit: f32,  // [rad]
    pub elevator_limit: f32, // [rad]
    pub rudder_limit: f32,   // [rad]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jet_fuel_adds_mass_and_reports_capacity() {
        let pp = Powerplant::JetFuel {
            capacity_kg: 2000.0,
            tsfc: 2.0e-5,
            fuel_type: FuelType::JetA,
        };
        assert!(pp.contributes_mass());
        assert_eq!(pp.capacity(), 2000.0);
        // Effective mass = empty + remaining fuel.
        assert_eq!(pp.effective_mass(3500.0, 2000.0), 5500.0);
        assert_eq!(pp.effective_mass(3500.0, 0.0), 3500.0);
        // Burn rate is thrust-specific.
        assert!((pp.burn_rate(40_000.0) - 2.0e-5 * 40_000.0).abs() < 1e-9);
    }

    #[test]
    fn electric_keeps_mass_constant() {
        let pp = Powerplant::Electric {
            capacity: 120.0,
            consumption: 1.0e-5,
        };
        assert!(!pp.contributes_mass());
        assert_eq!(pp.capacity(), 120.0);
        // Charge never adds mass.
        assert_eq!(pp.effective_mass(3500.0, 120.0), 3500.0);
        assert_eq!(pp.effective_mass(3500.0, 0.0), 3500.0);
        assert!((pp.burn_rate(40_000.0) - 1.0e-5 * 40_000.0).abs() < 1e-9);
    }

    #[test]
    fn fuel_type_properties_distinct_per_grade() {
        assert!((FuelType::JetA.properties().specific_energy_mj_per_kg - 43.0).abs() < 1e-6);
        assert!((FuelType::Jp5.properties().specific_energy_mj_per_kg - 42.8).abs() < 1e-6);
        assert_eq!(FuelType::Jp8.label(), "JP-8");
    }

    #[test]
    fn plane_config_parses_powerplant_from_ron() {
        let ron = r#"(
            wing_area: 20.0, mean_chord: 2.0, wing_span: 10.0,
            mass: 3500.0, inertia: (10000.0, 40000.0, 45000.0),
            cl0: 0.1, cl_alpha: 4.5, cl_delta_e: 0.4, cl_max: 1.4,
            cd0: 0.02, cd_induced: 0.05,
            cm0: -0.02, cm_alpha: 0.6, cm_q: -14.0, cm_delta_e: -1.2,
            cl_beta: -0.08, cl_p: -0.45, cl_r: 0.12, cl_delta_a: 0.18,
            cn_beta: 0.10, cn_r: -0.12, cn_delta_r: -0.10,
            thrust_max: 60000.0,
            powerplant: JetFuel(capacity_kg: 2000.0, tsfc: 2.0e-5, fuel_type: JetA),
            aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
        )"#;
        let cfg: PlaneConfig = ron::de::from_str(ron).expect("parse");
        assert_eq!(
            cfg.powerplant,
            Powerplant::JetFuel {
                capacity_kg: 2000.0,
                tsfc: 2.0e-5,
                fuel_type: FuelType::JetA,
            }
        );
    }

    #[test]
    fn plane_config_without_powerplant_falls_back_to_default() {
        let ron = r#"(
            wing_area: 20.0, mean_chord: 2.0, wing_span: 10.0,
            mass: 5000.0, inertia: (10000.0, 40000.0, 45000.0),
            cl0: 0.1, cl_alpha: 4.5, cl_delta_e: 0.4, cl_max: 1.4,
            cd0: 0.02, cd_induced: 0.05,
            cm0: -0.02, cm_alpha: 0.6, cm_q: -14.0, cm_delta_e: -1.2,
            cl_beta: -0.08, cl_p: -0.45, cl_r: 0.12, cl_delta_a: 0.18,
            cn_beta: 0.10, cn_r: -0.12, cn_delta_r: -0.10,
            thrust_max: 60000.0,
            aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
        )"#;
        let cfg: PlaneConfig = ron::de::from_str(ron).expect("parse");
        assert_eq!(cfg.powerplant, Powerplant::default());
    }
}
