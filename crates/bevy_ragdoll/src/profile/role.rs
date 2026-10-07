//! Closed body roles shared by profile authoring and active ragdoll policies.

/// Anatomical role inferred from a profile bone name or supplied explicitly.
///
/// Hit floors, recovery order, and later balance policies use this closed
/// vocabulary instead of repeating bone-name guesses during every fixed step.
#[derive(
    bevy::prelude::Component,
    Clone,
    Copy,
    Debug,
    Default,
    PartialEq,
    Eq,
    Hash,
    bevy::prelude::Reflect,
)]
#[cfg_attr(
    feature = "serialize",
    derive(serde::Deserialize, serde::Serialize),
    serde(rename_all = "snake_case")
)]
pub enum BodyRole {
    /// The central hip body commonly roots biped rigs. It receives early
    /// recovery and a higher muscle floor so impacts preserve support near
    /// the skeleton root.
    Pelvis,
    /// An axial vertebra between pelvis and chest. Spine bodies recover with
    /// the core group and keep a higher muscle floor than distal limbs after
    /// hits.
    Spine,
    /// The thorax or upper torso body above the spine. It starts recovery
    /// after core bodies and before thigh and calf bodies in the default role
    /// order.
    Chest,
    /// The segment between chest and head. It shares the head recovery delay
    /// and uses the standard distal-body muscle floor when hit reactions
    /// reduce support.
    Neck,
    /// The body above the neck. It uses a delayed recovery slot and the
    /// standard distal-body muscle floor after a validated hit reduces local
    /// strength.
    Head,
    /// The shoulder or upper-arm body between torso and elbow. It recovers
    /// with thigh and foot roles, before lower-arm and hand roles.
    UpperArm,
    /// The forearm body between elbow and wrist. It recovers later than
    /// upper-arm bodies and earlier than the hand in the staggered recovery
    /// schedule.
    LowerArm,
    /// The distal hand or wrist body at the end of an arm chain. It receives
    /// the latest standard recovery delay after an accepted hit.
    Hand,
    /// The upper-leg body between pelvis and calf. It keeps a higher muscle
    /// floor than distal leg segments and begins recovery after chest bodies.
    Thigh,
    /// The lower-leg body below the thigh, including shin segments. It retains
    /// a role-specific muscle floor and recovers later than the thigh after a
    /// hit.
    Calf,
    /// The distal foot body below the calf. It recovers after upper-arm bodies
    /// and before lower-arm bodies in the default staggered recovery policy.
    Foot,
    /// A tail segment outside the standard human limb chain. It shares the
    /// hand recovery delay and uses the distal-body muscle floor after hits.
    Tail,
    /// A body with no recognized bone-name hint. It retains the default
    /// distal-body muscle floor and starts recovery with the core bodies after
    /// a hit.
    #[default]
    Other,
}

/// Bone-name hints are ordered from specific body regions to broad limb names.
const ROLE_HINTS: &[(&[&str], BodyRole)] = &[
    (&["pelvis", "hip"], BodyRole::Pelvis),
    (&["spine"], BodyRole::Spine),
    (&["chest", "thorax"], BodyRole::Chest),
    (&["neck"], BodyRole::Neck),
    (&["head"], BodyRole::Head),
    (&["hand", "wrist"], BodyRole::Hand),
    (&["forearm", "lowerarm", "lower_arm"], BodyRole::LowerArm),
    (
        &["upperarm", "upper_arm", "shoulder", "clavicle", "arm"],
        BodyRole::UpperArm,
    ),
    (&["calf", "shin", "lowerleg", "lower_leg"], BodyRole::Calf),
    (&["foot"], BodyRole::Foot),
    (&["thigh", "upperleg", "upper_leg", "leg"], BodyRole::Thigh),
    (&["tail"], BodyRole::Tail),
];

impl From<&str> for BodyRole {
    /// Converts a profile bone name into the first matching anatomical role.
    ///
    /// Matching ignores ASCII letter case and checks specific limb names
    /// before broad `arm` and `leg` hints. Unknown names map to [`BodyRole::Other`].
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::BodyRole;
    ///
    /// assert_eq!(BodyRole::from("spine_03"), BodyRole::Spine);
    /// ```
    fn from(name: &str) -> Self {
        // Normalize once so each ordered hint uses the same case-insensitive comparison.
        let name = name.to_ascii_lowercase();
        for (hints, role) in ROLE_HINTS {
            // Keep narrower limb names ahead of the broader arm and leg hints.
            if hints.iter().any(|hint| name.contains(hint)) {
                return *role;
            }
        }
        // Preserve an explicit unknown role when no profile hint matches.
        Self::Other
    }
}

impl BodyRole {
    /// Returns the minimum muscle multiplier retained after a hit.
    ///
    /// The value is normalized to `0..=1`; pelvis, spine, thigh, and calf
    /// floors preserve more support than distal bodies.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::BodyRole;
    ///
    /// assert_eq!(BodyRole::Pelvis.muscle_floor(), 0.15);
    /// ```
    pub const fn muscle_floor(self) -> f32 {
        match self {
            Self::Pelvis => 0.15,
            Self::Spine | Self::Chest | Self::Thigh => 0.10,
            Self::Calf => 0.08,
            Self::Neck
            | Self::Head
            | Self::UpperArm
            | Self::LowerArm
            | Self::Hand
            | Self::Foot
            | Self::Tail
            | Self::Other => 0.05,
        }
    }

    /// Returns this role's delay after core recovery begins.
    ///
    /// The default order follows hit-reaction design section 2.4. Neck uses
    /// the head delay, tail uses the hand delay, and an unclassified body uses
    /// the core delay because the source table assigns it no staggered delay.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::BodyRole;
    /// use std::time::Duration;
    ///
    /// assert_eq!(BodyRole::Hand.recovery_order_delay(), Duration::from_millis(300));
    /// ```
    pub const fn recovery_order_delay(self) -> std::time::Duration {
        let milliseconds = match self {
            Self::Pelvis | Self::Spine | Self::Other => 0,
            Self::Chest => 50,
            Self::Thigh => 100,
            Self::Calf => 150,
            Self::Foot | Self::UpperArm => 200,
            Self::LowerArm | Self::Head | Self::Neck => 250,
            Self::Hand | Self::Tail => 300,
        };
        std::time::Duration::from_millis(milliseconds)
    }
}
