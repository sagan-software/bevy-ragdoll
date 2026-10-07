//! Ragdoll profiles generated from any skeleton without authored files.
//!
//! [`RagdollProfile::from_skeleton`] turns bone names, parents and rest poses
//! into bodies, capsules, masses and joint limits. Bone names select a
//! humanoid layout when they follow a standard convention (UE4 and UE5
//! mannequin, Mixamo, Unity, Godot and VRM humanoid, Rigify). Other skeletons
//! are classified by topology, so quadrupeds and many-legged creatures work
//! too. [`RagdollBone`] holds optional sparse overrides per bone.

mod generate;
mod humanoid;
mod overrides;
mod scene;

#[cfg(test)]
mod tests;

use bevy::math::{Isometry3d, Quat, Vec3};

pub(crate) use self::scene::skeleton_from_world;
pub use self::overrides::{BoneBody, RagdollBone, RagdollOverrides};
#[cfg(feature = "serialize")]
pub use self::overrides::{RagdollOverridesLoader, RagdollOverridesLoaderError};

use crate::profile::{Mass, ProfileError, ProfileSpec, RagdollProfile};

/// Generator input: bones in parent-first order, in one skeleton space.
///
/// The runtime builds this from the descendants of a [`crate::runtime::components::Ragdoll`]
/// entity. Code can build it with [`Skeleton::from_positions`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Skeleton {
    /// Bones in parent-first order; a parent index always precedes its child.
    pub bones: Vec<SkeletonBone>,
    /// Total ragdoll mass; `None` keeps the mass derived from body volume.
    pub mass: Option<Mass>,
}

/// One skeleton bone with its rest pose and optional overrides.
#[derive(Clone, Debug, PartialEq)]
pub struct SkeletonBone {
    /// Bone name used to bind the generated body to the skeleton entity.
    pub name: String,
    /// Index of the parent bone in [`Skeleton::bones`], absent for roots.
    pub parent: Option<usize>,
    /// Bone rest pose in skeleton space; the bone's local Y axis is expected to
    /// point along the bone, as in Blender exports.
    pub rest: Isometry3d,
    /// Sparse per-bone overrides; the default keeps every generated value.
    pub overrides: RagdollBone,
}

impl Skeleton {
    /// Builds a skeleton from `(name, parent name, head position)` triples.
    ///
    /// Entries must list each parent before its children; an unknown parent
    /// name makes the bone a root. Each bone's local Y axis is rotated to point
    /// at its first child (leaves follow their parent), which is the bone
    /// orientation the generator expects.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::math::Vec3;
    /// use bevy_ragdoll::{RagdollProfile, Skeleton};
    ///
    /// let skeleton = Skeleton::from_positions([
    ///     ("hips", None, Vec3::new(0.0, 1.0, 0.0)),
    ///     ("chest", Some("hips"), Vec3::new(0.0, 1.4, 0.0)),
    ///     ("head", Some("chest"), Vec3::new(0.0, 1.7, 0.0)),
    /// ]);
    /// let profile = RagdollProfile::from_skeleton(&skeleton)?;
    /// assert_eq!(profile.bodies().len(), 3);
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    pub fn from_positions<'a>(
        bones: impl IntoIterator<Item = (&'a str, Option<&'a str>, Vec3)>,
    ) -> Self {
        let mut skeleton = Self::default();
        for (name, parent, position) in bones {
            // Resolve the parent among bones already pushed, keeping parent-first order.
            let parent =
                parent.and_then(|parent| skeleton.bones.iter().position(|b| b.name == parent));
            skeleton.bones.push(SkeletonBone {
                name: name.to_owned(),
                parent,
                rest: Isometry3d::from_translation(position),
                overrides: RagdollBone::default(),
            });
        }
        skeleton.orient_bones();
        skeleton
    }

    /// Returns the index of the bone named `name`.
    pub fn bone_index(&self, name: &str) -> Option<usize> {
        self.bones.iter().position(|bone| bone.name == name)
    }

    /// An 80 kg, 1.8 m reference humanoid in T-pose with UE mannequin bone names.
    ///
    /// It faces +Z with +Y up and its feet on `y = 0`. Tests, benches and
    /// examples use it when no glTF asset is loaded.
    pub fn humanoid() -> Self {
        const BONES: &[(&str, Option<&str>, [f32; 3])] = &[
            ("pelvis", None, [0.0, 0.95, 0.0]),
            ("spine_01", Some("pelvis"), [0.0, 1.05, 0.0]),
            ("spine_02", Some("spine_01"), [0.0, 1.18, 0.0]),
            ("spine_03", Some("spine_02"), [0.0, 1.32, 0.0]),
            ("neck_01", Some("spine_03"), [0.0, 1.50, 0.0]),
            ("head", Some("neck_01"), [0.0, 1.58, 0.0]),
            ("clavicle_l", Some("spine_03"), [0.05, 1.45, 0.0]),
            ("upperarm_l", Some("clavicle_l"), [0.19, 1.45, 0.0]),
            ("lowerarm_l", Some("upperarm_l"), [0.47, 1.45, 0.0]),
            ("hand_l", Some("lowerarm_l"), [0.73, 1.45, 0.0]),
            ("clavicle_r", Some("spine_03"), [-0.05, 1.45, 0.0]),
            ("upperarm_r", Some("clavicle_r"), [-0.19, 1.45, 0.0]),
            ("lowerarm_r", Some("upperarm_r"), [-0.47, 1.45, 0.0]),
            ("hand_r", Some("lowerarm_r"), [-0.73, 1.45, 0.0]),
            ("thigh_l", Some("pelvis"), [0.10, 0.92, 0.0]),
            ("calf_l", Some("thigh_l"), [0.10, 0.50, 0.0]),
            ("foot_l", Some("calf_l"), [0.10, 0.08, 0.0]),
            ("ball_l", Some("foot_l"), [0.10, 0.02, 0.14]),
            ("thigh_r", Some("pelvis"), [-0.10, 0.92, 0.0]),
            ("calf_r", Some("thigh_r"), [-0.10, 0.50, 0.0]),
            ("foot_r", Some("calf_r"), [-0.10, 0.08, 0.0]),
            ("ball_r", Some("foot_r"), [-0.10, 0.02, 0.14]),
        ];
        let mut skeleton = Self::from_positions(
            BONES
                .iter()
                .map(|(name, parent, at)| (*name, *parent, Vec3::from_array(*at))),
        );
        skeleton.mass = Mass::try_from(80.0).ok();
        skeleton
    }

    /// Spawns the bones under `character` at their rest pose and returns them.
    ///
    /// Each bone entity gets a `Name` and a local `Transform`. A runtime
    /// [`crate::Ragdoll`] on `character` binds to these bones by name.
    pub fn spawn(
        &self,
        commands: &mut bevy::prelude::Commands,
        character: bevy::prelude::Entity,
    ) -> Vec<bevy::prelude::Entity> {
        use bevy::prelude::{ChildOf, Name, Transform};
        let mut entities: Vec<bevy::prelude::Entity> = Vec::with_capacity(self.bones.len());
        for bone in &self.bones {
            let parent = bone.parent.filter(|parent| *parent < entities.len());
            let (parent_entity, local) = match parent {
                Some(parent) => (entities[parent], self.bones[parent].rest.inverse() * bone.rest),
                None => (character, bone.rest),
            };
            let transform =
                Transform::from_translation(local.translation.into()).with_rotation(local.rotation);
            let mut entity = commands.spawn((Name::new(bone.name.clone()), transform, ChildOf(parent_entity)));
            if bone.overrides != RagdollBone::default() {
                entity.insert(bone.overrides.clone());
            }
            entities.push(entity.id());
        }
        entities
    }

    /// Rotates every bone so its local Y axis points along the bone.
    fn orient_bones(&mut self) {
        // The first child gives each bone's direction; leaves reuse their parent link.
        let directions = (0..self.bones.len())
            .map(|index| {
                let head = self.bones[index].rest.translation;
                let child = self.bones.iter().find(|bone| bone.parent == Some(index));
                let toward = match (child, self.bones[index].parent) {
                    (Some(child), _) => Vec3::from(child.rest.translation - head),
                    (None, Some(parent)) => {
                        Vec3::from(head - self.bones[parent].rest.translation)
                    }
                    (None, None) => Vec3::Y,
                };
                toward.try_normalize().unwrap_or(Vec3::Y)
            })
            .collect::<Vec<_>>();
        for (bone, direction) in self.bones.iter_mut().zip(directions) {
            bone.rest.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
        }
    }
}

impl From<&Skeleton> for ProfileSpec {
    /// Generates the unvalidated profile data for `skeleton`.
    fn from(skeleton: &Skeleton) -> Self {
        generate::generate(skeleton)
    }
}

impl RagdollProfile {
    /// Generates and validates a profile from any skeleton.
    ///
    /// The generator keeps deform bones, merges short and helper bones into
    /// their parents, fits capsules, assigns roles and limits, and derives
    /// masses from volume. Use [`ProfileSpec::from`] instead to inspect or edit
    /// the generated data before validation.
    ///
    /// # Errors
    ///
    /// Returns [`ProfileError::Empty`] when no bone can carry a body, or any
    /// other validation error caused by an override value.
    pub fn from_skeleton(skeleton: &Skeleton) -> Result<Self, ProfileError> {
        Self::new(ProfileSpec::from(skeleton))
    }
}
