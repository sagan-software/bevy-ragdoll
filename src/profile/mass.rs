//! A finite positive mass measured in kilograms.

/// A mass value that is finite, positive, and measured in kilograms.
///
/// Construct this semantic value with [`TryFrom<f32>`], which rejects zero,
/// negative, infinite, and NaN inputs before callers store or compare mass.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, bevy::prelude::Reflect)]
pub struct Mass(f32);

/// The numeric value cannot represent a finite positive mass in kilograms.
///
/// `TryFrom<f32>` returns this error for zero, negative, infinite, or NaN
/// values. Callers can preserve their own body or file context when reporting it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("mass must be finite and positive")]
pub struct MassError;

impl TryFrom<f32> for Mass {
    type Error = MassError;

    fn try_from(kilograms: f32) -> Result<Self, Self::Error> {
        if kilograms.is_finite() && kilograms > 0.0 {
            Ok(Self(kilograms))
        } else {
            Err(MassError)
        }
    }
}

impl Mass {
    /// Returns this validated mass in kilograms without converting its unit.
    ///
    /// The returned `f32` remains finite and positive because `Mass` can only
    /// enter through its checked `TryFrom<f32>` implementation.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{Mass, MassError};
    ///
    /// let mass = Mass::try_from(12.5_f32)?;
    /// assert_eq!(mass.kilograms(), 12.5);
    /// # Ok::<(), MassError>(())
    /// ```
    #[must_use]
    pub const fn kilograms(self) -> f32 {
        self.0
    }
}
