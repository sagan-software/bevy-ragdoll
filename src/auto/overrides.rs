//! Sparse per-bone overrides from components, code or RON files.

use std::collections::BTreeMap;

use bevy::prelude::{Component, ReflectComponent, ReflectDefault};

use crate::profile::{BodyRole, JointLimits};

/// How the profile generator treats one bone when it selects bodies.
///
/// The default lets the generator decide; the other variants force a body,
/// merge the bone into its parent body, or drop the bone and its subtree.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
#[reflect(Default)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
pub enum BoneBody {
    /// The generator decides from the bone name, its length relative to the
    /// character height, and its position in the skeleton topology.
    #[default]
    Auto,
    /// The bone always gets its own physics body, even when the generator
    /// would merge it because it is short or has an unrecognized name.
    Body,
    /// The bone never gets a body. Its length and geometry join the body of
    /// its nearest ancestor that does get one.
    Merge,
    /// The bone and all its descendants are ignored, so no body, joint, or
    /// merged geometry comes from that part of the skeleton.
    Skip,
}

/// Optional ragdoll overrides for one bone; every `None` keeps the generated value.
///
/// Insert it on a bone entity from Rust, or add it to a bone in Blender with
/// Skein: the plugin registers this type, so no other setup is needed.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, bevy::prelude::Reflect)]
#[reflect(Component, Default)]
#[cfg_attr(
    feature = "serialize",
    derive(serde::Deserialize, serde::Serialize),
    serde(default, deny_unknown_fields)
)]
pub struct RagdollBone {
    /// Whether this bone gets a body, merges into its parent body, or is
    /// skipped together with its descendants; see [`BoneBody`].
    pub body: BoneBody,
    /// Role used by hit reactions and recovery order. `None` keeps the role
    /// that the generator derives from the bone name and the skeleton topology.
    pub role: Option<BodyRole>,
    /// Body mass in kilograms. The generator keeps this mass when it scales
    /// the other bodies to reach the total ragdoll mass.
    pub mass: Option<f32>,
    /// Capsule radius in metres. `None` keeps the radius that the generator
    /// derives from the body role, the segment length and the skeleton size.
    pub radius: Option<f32>,
    /// Limits of the joint between this body and its parent body, in
    /// radians. `None` keeps the generated limits for the body role.
    pub limits: Option<JointLimits>,
    /// Maximum motor torque of that joint in newton metres. `None` keeps the
    /// template torque for the body role, scaled by the total ragdoll mass.
    pub max_torque: Option<f32>,
}

/// Overrides for several bones, usually loaded from a `.ragdoll.ron` file.
///
/// Keys are exact bone names or a prefix ending in one `*`. An exact key
/// beats a prefix, and a longer prefix beats a shorter one. A
/// [`RagdollBone`] component on the bone entity beats this asset.
///
/// ```ron
/// (
///     mass: Some(12.0),
///     bones: {
///         "tail_*": (limits: Some((x: (min: -0.3, max: 0.3), twist: (min: 0.0, max: 0.0), z: (min: -0.3, max: 0.3)))),
///         "ear_*": (body: Skip),
///     },
/// )
/// ```
#[derive(bevy::asset::Asset, Clone, Debug, Default, PartialEq, bevy::prelude::Reflect)]
#[cfg_attr(
    feature = "serialize",
    derive(serde::Deserialize, serde::Serialize),
    serde(default, deny_unknown_fields)
)]
pub struct RagdollOverrides {
    /// Total ragdoll mass in kilograms. `None` keeps the skeleton's mass,
    /// which defaults to the sum of the volume-derived body masses.
    pub mass: Option<f32>,
    /// Overrides keyed by exact bone name or by a `prefix*` pattern. The
    /// [`RagdollOverrides::get`] method resolves which key applies to each bone.
    pub bones: BTreeMap<String, RagdollBone>,
}

impl RagdollOverrides {
    /// Returns the override that applies to `bone`, if any key matches.
    ///
    /// An exact key wins. Otherwise the longest matching `prefix*` key wins.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BoneBody, RagdollBone, RagdollOverrides};
    ///
    /// let mut overrides = RagdollOverrides::default();
    /// let skip = RagdollBone { body: BoneBody::Skip, ..RagdollBone::default() };
    /// overrides.bones.insert("tail_*".to_owned(), skip);
    /// assert_eq!(overrides.get("tail_03"), Some(&skip));
    /// assert_eq!(overrides.get("head"), None);
    /// ```
    #[must_use]
    pub fn get(&self, bone: &str) -> Option<&RagdollBone> {
        if let Some(exact) = self.bones.get(bone) {
            return Some(exact);
        }
        // Pick the longest matching prefix pattern.
        self.bones
            .iter()
            .filter_map(|(key, value)| {
                let prefix = key.strip_suffix('*')?;
                bone.starts_with(prefix).then_some((prefix.len(), value))
            })
            .max_by_key(|(length, _)| *length)
            .map(|(_, value)| value)
    }
}

/// Loads [`RagdollOverrides`] from `.ragdoll.ron` files.
///
/// [`crate::RagdollPlugin`] registers this loader, so `AssetServer::load`
/// returns a [`RagdollOverrides`] handle for any asset path that ends in
/// `.ragdoll.ron`.
#[cfg(feature = "serialize")]
#[derive(Clone, Copy, Debug, Default, bevy::reflect::TypePath)]
pub struct RagdollOverridesLoader;

/// A `.ragdoll.ron` file could not be read or parsed.
///
/// The asset server reports this error and leaves the overrides handle
/// without a loaded asset.
#[cfg(feature = "serialize")]
#[derive(Debug, thiserror::Error)]
pub enum RagdollOverridesLoaderError {
    /// The asset reader failed before all bytes of the overrides file were
    /// read; the error source carries the underlying I/O error.
    #[error("could not read ragdoll overrides: {0}")]
    Io(#[from] std::io::Error),
    /// The bytes are not valid overrides RON; the error source carries the
    /// position and reason reported by the RON parser.
    #[error("could not parse ragdoll overrides: {0}")]
    Ron(#[from] ron::error::SpannedError),
}

#[cfg(feature = "serialize")]
impl bevy::asset::AssetLoader for RagdollOverridesLoader {
    type Asset = RagdollOverrides;
    type Settings = ();
    type Error = RagdollOverridesLoaderError;

    async fn load(
        &self,
        reader: &mut dyn bevy::asset::io::Reader,
        _settings: &(),
        _context: &mut bevy::asset::LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        Ok(ron::de::from_bytes(&bytes)?)
    }

    fn extensions(&self) -> &[&str] {
        &["ragdoll.ron"]
    }
}
