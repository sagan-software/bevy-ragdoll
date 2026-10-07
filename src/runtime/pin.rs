//! Per-character pin targets and optional pin-controller tuning.
//!
//! [`PinTargets`] selects profile bodies that receive world-space animation
//! forces. [`PinSettings`] overrides the shared pin controller values for one
//! character when present.
//! The target mask uses checked body indexes and defaults to all profile bodies.
//! Controller values are finite and nonnegative before fixed-step drive reads
//! them, so backend adapters never receive invalid tuning through this API.

use bevy::prelude::{Component, Reflect};

use crate::profile::BodyIndex;

use super::settings::RagdollPhysicsSettings;

/// Profile bodies that receive world-space pin forces.
///
/// The mask follows validated parent-first [`BodyIndex`] values. Its default
/// selects every valid profile body; bits above
/// [`crate::profile::MAX_BODIES`] have no body to address and are ignored.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[component(storage = "SparseSet")]
pub struct PinTargets(u64);

impl Default for PinTargets {
    fn default() -> Self {
        Self::all()
    }
}

impl PinTargets {
    /// Selects every checked profile body for animation pin forces and preserves
    /// full-body following while valid target poses exist during dynamic drive.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyIndex, runtime::pin::PinTargets};
    ///
    /// let targets = PinTargets::all();
    /// assert!(targets.is_targeted(BodyIndex::try_from(0).expect("zero is valid")));
    /// ```
    #[must_use]
    pub const fn all() -> Self {
        Self(u64::MAX)
    }

    /// Selects no checked profile body, so drive calculations publish no pin
    /// force. Use this mask when the character should receive no world-space
    /// pin forces while muscle drive remains active.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyIndex, runtime::pin::PinTargets};
    ///
    /// let targets = PinTargets::none();
    /// assert!(!targets.is_targeted(BodyIndex::try_from(0).expect("zero is valid")));
    /// ```
    #[must_use]
    pub const fn none() -> Self {
        Self(0)
    }

    /// Selects only checked body indexes yielded by the caller without
    /// allocating a collection. Other profile positions stay excluded until
    /// the caller includes them explicitly.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyIndex, runtime::pin::PinTargets};
    ///
    /// let pelvis = BodyIndex::try_from(0).expect("zero is valid");
    /// let targets = PinTargets::only([pelvis]);
    /// assert!(targets.is_targeted(pelvis));
    /// ```
    pub fn only(indexes: impl IntoIterator<Item = BodyIndex>) -> Self {
        let mut targets = Self::none();
        // Consume indexes as supplied so this constructor needs no temporary collection.
        for index in indexes {
            targets.set(index, true);
        }
        targets
    }

    /// Returns whether the checked profile body receives animation pin forces
    /// during drive calculation. False means this mask excludes that index
    /// from the character's pin targets.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyIndex, runtime::pin::PinTargets};
    ///
    /// let pelvis = BodyIndex::try_from(0).expect("zero is valid");
    /// assert!(PinTargets::default().is_targeted(pelvis));
    /// ```
    #[must_use]
    pub const fn is_targeted(self, index: BodyIndex) -> bool {
        self.0 & (1_u64 << index.get()) != 0
    }

    /// Includes or excludes one checked profile body from animation pin forces.
    /// The predicate changes only its bit and preserves every other body choice.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyIndex, runtime::pin::PinTargets};
    ///
    /// let pelvis = BodyIndex::try_from(0).expect("zero is valid");
    /// let mut targets = PinTargets::none();
    /// targets.set(pelvis, true);
    /// assert!(targets.is_targeted(pelvis));
    /// ```
    pub const fn set(&mut self, index: BodyIndex, is_targeted: bool) {
        // Update one mask bit while preserving all other profile body choices.
        let mask = 1_u64 << index.get();
        if is_targeted {
            self.0 |= mask;
        } else {
            self.0 &= !mask;
        }
    }
}

/// Optional per-character overrides for the world-space pin controller.
///
/// Values are finite and nonnegative. A non-finite value becomes zero, which
/// disables that controller contribution without propagating invalid floats.
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[component(storage = "SparseSet")]
pub struct PinSettings {
    /// Natural frequency in hertz.
    frequency_hz: f32,
    /// Nonnegative damping ratio.
    damping_ratio: f32,
    /// Maximum pin force in newtons.
    max_force: f32,
    /// Maximum pin torque in newton metres.
    max_torque: f32,
    /// Force falloff per metre of positional error.
    distance_falloff: f32,
}

impl Default for PinSettings {
    fn default() -> Self {
        Self {
            frequency_hz: 1.5,
            damping_ratio: 1.0,
            max_force: 340.0,
            max_torque: 400.0,
            distance_falloff: 2.0,
        }
    }
}

impl PinSettings {
    /// Creates per-character tuning after mapping non-finite and negative values
    /// to zero. The arguments use hertz, ratio, newtons, newton metres, and
    /// falloff per metre.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::pin::PinSettings;
    ///
    /// let settings = PinSettings::new(2.0, 1.0, 300.0, 400.0, 2.0);
    /// assert_eq!(settings.frequency_hz(), 2.0);
    /// ```
    #[must_use]
    pub const fn new(
        frequency_hz: f32,
        damping_ratio: f32,
        max_force: f32,
        max_torque: f32,
        distance_falloff: f32,
    ) -> Self {
        Self {
            frequency_hz: finite_nonnegative(frequency_hz),
            damping_ratio: finite_nonnegative(damping_ratio),
            max_force: finite_nonnegative(max_force),
            max_torque: finite_nonnegative(max_torque),
            distance_falloff: finite_nonnegative(distance_falloff),
        }
    }

    /// Returns the finite natural frequency in hertz used by the pin controller.
    /// It sets how quickly this character's controller follows captured targets.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::pin::PinSettings;
    ///
    /// assert_eq!(PinSettings::default().frequency_hz(), 1.5);
    /// ```
    #[must_use]
    pub const fn frequency_hz(self) -> f32 {
        self.frequency_hz
    }

    /// Returns the finite nonnegative damping ratio that controls pin
    /// oscillation. It controls response damping at the selected frequency.
    /// The value governs oscillation settling for this character.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::pin::PinSettings;
    ///
    /// assert_eq!(PinSettings::default().damping_ratio(), 1.0);
    /// ```
    #[must_use]
    pub const fn damping_ratio(self) -> f32 {
        self.damping_ratio
    }

    /// Returns the finite maximum pin force in newtons applied to one character.
    /// This cap bounds translation correction sent to each backend adapter on
    /// every physics step.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::pin::PinSettings;
    ///
    /// assert_eq!(PinSettings::default().max_force(), 340.0);
    /// ```
    #[must_use]
    pub const fn max_force(self) -> f32 {
        self.max_force
    }

    /// Returns the finite maximum pin torque in newton metres applied to one
    /// character. It limits angular correction before the backend physics step.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::pin::PinSettings;
    ///
    /// assert_eq!(PinSettings::default().max_torque(), 400.0);
    /// ```
    #[must_use]
    pub const fn max_torque(self) -> f32 {
        self.max_torque
    }

    /// Returns finite force falloff per metre of error from the animation
    /// target. It reduces pin force as positional error from the captured pose
    /// grows.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::pin::PinSettings;
    ///
    /// assert_eq!(PinSettings::default().distance_falloff(), 2.0);
    /// ```
    #[must_use]
    pub const fn distance_falloff(self) -> f32 {
        self.distance_falloff
    }

    /// Applies these checked values to a copy of the shared physics settings.
    pub(crate) const fn override_shared(self, settings: &mut RagdollPhysicsSettings) {
        // Copy each character-level override before drive reads the shared controller settings.
        settings.pin_frequency_hz = self.frequency_hz;
        settings.pin_damping_ratio = self.damping_ratio;
        settings.pin_max_force = self.max_force;
        settings.pin_max_torque = self.max_torque;
        settings.pin_distance_falloff = self.distance_falloff;
    }
}

/// Replaces non-finite tuning values with zero and clamps finite values below zero.
const fn finite_nonnegative(value: f32) -> f32 {
    if value.is_finite() {
        value.max(0.0)
    } else {
        0.0
    }
}
