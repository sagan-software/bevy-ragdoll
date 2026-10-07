//! Store Avian-only solver and body setup options.

use bevy::prelude::Resource;

/// Avian-only tuning values that do not belong to the shared physics settings.
///
/// The defaults are the values that passed the most physics-tier conformance
/// cases on Avian 0.7 (see `docs/plan/phases/12-avian3d.md`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Resource)]
pub struct AvianRagdollSettings {
    /// Avian `SubstepCount` written every fixed step. Values below one are
    /// raised to one. More substeps stiffen joints and contacts at a roughly
    /// linear solver cost. The default is 20; 8 left knees 30 degrees short of
    /// their torque-driven target.
    pub substep_count: u32,
    /// Whether bodies get `SweptCcd` when `RagdollPhysicsSettings::is_ccd_enabled`
    /// is set. Off by default: Avian moves each swept body back to its own time
    /// of impact after the solve, which separated joints by up to 17 cm on
    /// landing. Speculative contacts still prevent most tunneling.
    pub use_swept_ccd: bool,
}

impl Default for AvianRagdollSettings {
    fn default() -> Self {
        Self {
            substep_count: 20,
            use_swept_ccd: false,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Checks the Avian adapter defaults.

    use super::AvianRagdollSettings;

    /// Twenty substeps and no swept CCD by default.
    #[test]
    fn default_settings_match_the_measured_profile() {
        let settings = AvianRagdollSettings::default();
        assert_eq!(settings.substep_count, 20);
        assert!(!settings.use_swept_ccd);
    }
}
