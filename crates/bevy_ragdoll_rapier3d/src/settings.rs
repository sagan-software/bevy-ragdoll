//! Store Rapier-specific solver and body setup options shared by adapter systems.

use bevy::prelude::Resource;

/// Rapier-only tuning values that do not belong to the shared physics settings.
///
/// Applications can change CCD substeps and root-body solver iterations
/// without adding Rapier types to the backend-neutral runtime API. A value of
/// zero disables Rapier's CCD substeps or the extra root-body iterations.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Resource, bevy::prelude::Reflect)]
pub struct RapierRagdollSettings {
    /// Maximum number of continuous-collision subdivisions Rapier may run per body
    /// during one simulation step. Higher values reduce tunneling risk but add work.
    pub max_ccd_substeps: usize,
    /// Additional constraint passes applied to each root body at spawn.
    /// Positive values can improve mass-ratio convergence; zero disables them.
    /// Each pass adds solver work across bodies connected to that root.
    pub root_additional_solver_iterations: usize,
}

impl Default for RapierRagdollSettings {
    fn default() -> Self {
        Self {
            max_ccd_substeps: 1,
            root_additional_solver_iterations: 4,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Checks Rapier adapter defaults for CCD and root constraint solving.

    use super::RapierRagdollSettings;

    /// CCD keeps one subdivision and roots add four solver passes by default.
    #[test]
    fn default_settings_match_the_backend_profile() {
        let settings = RapierRagdollSettings::default();
        assert_eq!(settings.max_ccd_substeps, 1);
        assert_eq!(settings.root_additional_solver_iterations, 4);
    }
}
