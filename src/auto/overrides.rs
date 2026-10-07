//! Sparse per-bone overrides from components, code or RON files.

use std::collections::BTreeMap;

use bevy::prelude::{Component, ReflectComponent, ReflectDefault};

use crate::profile::{BodyRole, JointLimits};

/// How the generator treats one bone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
#[reflect(Default)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
pub enum BoneBody {
    /// The generator decides from names, length and topology.
    #[default]
    Auto,
    /// The bone always gets its own body.
    Body,
    /// The bone never gets a body; its geometry joins the parent body.
    Merge,
    /// The bone and all its descendants are ignored.
    Skip,
}

/// Optional ragdoll overrides for one bone; every `None` keeps the generated value.
///
/// Insert it on a bone entity from Rust, or add it to a bone in Blender with
/// Skein: the plugin registers this type, so no other setup is needed.
#[derive(Component, Clone, Debug, Default, PartialEq, bevy::prelude::Reflect)]
#[reflect(Component, Default)]
#[cfg_attr(
    feature = "serialize",
    derive(serde::Deserialize, serde::Serialize),
    serde(default, deny_unknown_fields)
)]
pub struct RagdollBone {
    /// Whether this bone gets a body.
    pub body: BoneBody,
    /// Role used by hit reactions and recovery order.
    pub role: Option<BodyRole>,
    /// Body mass in kilograms; it is kept when the total mass is normalized.
    pub mass: Option<f32>,
    /// Capsule radius in metres.
    pub radius: Option<f32>,
    /// Limits of the joint between this body and its parent, in radians.
    pub limits: Option<JointLimits>,
    /// Maximum motor torque of that joint in newton metres.
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
    /// Total ragdoll mass in kilograms.
    pub mass: Option<f32>,
    /// Overrides keyed by bone name or `prefix*` pattern.
    pub bones: BTreeMap<String, RagdollBone>,
}

impl RagdollOverrides {
    /// Returns the override that applies to `bone`, if any key matches.
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
#[cfg(feature = "serialize")]
#[derive(Default, bevy::reflect::TypePath)]
pub struct RagdollOverridesLoader;

/// A `.ragdoll.ron` file could not be read or parsed.
#[cfg(feature = "serialize")]
#[derive(Debug, thiserror::Error)]
pub enum RagdollOverridesLoaderError {
    /// The asset bytes could not be read.
    #[error("could not read ragdoll overrides: {0}")]
    Io(#[from] std::io::Error),
    /// The bytes are not valid overrides RON.
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
