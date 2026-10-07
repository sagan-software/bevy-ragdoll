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

use bevy::math::{Isometry3d, Mat3, Quat, Vec3};

pub use self::overrides::{BoneBody, RagdollBone, RagdollOverrides};
#[cfg(feature = "serialize")]
pub use self::overrides::{RagdollOverridesLoader, RagdollOverridesLoaderError};
pub(crate) use self::scene::skeleton_from_world;

use crate::profile::{Mass, ProfileError, ProfileSpec, RagdollProfile};

/// Generator input: bones in parent-first order, in one skeleton space.
///
/// The runtime builds this from the descendants of a [`crate::runtime::components::Ragdoll`]
/// entity. Code can build it with [`Skeleton::from_positions`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Skeleton {
    /// Bones in parent-first order. A parent index always precedes its child,
    /// so the generator can resolve every parent in one forward pass.
    pub bones: Vec<SkeletonBone>,
    /// Total ragdoll mass. `None` keeps the mass that the generator derives
    /// from body volume at the density of the human body.
    pub mass: Option<Mass>,
}

/// One skeleton bone with its rest pose and optional overrides.
///
/// The generator reads the name for roles, the parent for topology, and the
/// rest pose for segment lengths and joint frames.
#[derive(Clone, Debug, PartialEq)]
pub struct SkeletonBone {
    /// Bone name used to bind the generated body to the skeleton entity. The
    /// generator also matches it against known names to assign body roles.
    pub name: String,
    /// Index of the parent bone in [`Skeleton::bones`], or `None` for a root
    /// bone. The index must be smaller than this bone's own index.
    pub parent: Option<usize>,
    /// Bone rest pose in skeleton space; the bone's local Y axis is expected to
    /// point along the bone, as in Blender exports.
    pub rest: Isometry3d,
    /// Sparse per-bone overrides from a [`RagdollBone`] component or a RON
    /// file. The default value keeps every generated value for this bone.
    pub overrides: RagdollBone,
}

#[expect(
    clippy::indexing_slicing,
    reason = "bone indexes come from the same parent-first skeleton vectors"
)]
impl Skeleton {
    /// Builds a skeleton from `(name, parent name, head position)` triples.
    ///
    /// Entries must list each parent before its children; an unknown parent
    /// name makes the bone a root. Each bone's local Y axis points at the child
    /// that best continues its parent link (leaves follow their parent), and
    /// local Z stays closest to
    /// +Z, matching Blender's glTF export.
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
        // Bones start unrotated; orient_bones aims them once all heads are known.
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

    /// Returns the index of the bone named `name` in [`Skeleton::bones`], or
    /// `None` when no bone in this skeleton has that exact, case-sensitive name.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::Skeleton;
    ///
    /// let skeleton = Skeleton::humanoid();
    /// assert_eq!(skeleton.bone_index("pelvis"), Some(0));
    /// assert_eq!(skeleton.bone_index("tail"), None);
    /// ```
    #[must_use]
    pub fn bone_index(&self, name: &str) -> Option<usize> {
        self.bones.iter().position(|bone| bone.name == name)
    }

    /// An 80 kg, 1.8 m reference humanoid in A-pose with UE4 mannequin names.
    ///
    /// It faces +Z with +Y up and its feet on `y = 0`. Tests, benches and
    /// examples use it when no glTF asset is loaded.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{RagdollProfile, Skeleton};
    ///
    /// let profile = RagdollProfile::from_skeleton(&Skeleton::humanoid())?;
    /// assert_eq!(profile.bodies().len(), 16);
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    #[must_use]
    pub fn humanoid() -> Self {
        const BONES: &[(&str, Option<&str>, [f32; 3])] = &[
            ("pelvis", None, [0.0, 0.96, 0.0]),
            ("spine_01", Some("pelvis"), [0.0, 1.11, 0.0]),
            ("spine_02", Some("spine_01"), [0.0, 1.28, 0.0]),
            ("clavicle_l", Some("spine_02"), [0.04, 1.42, 0.0]),
            ("upperarm_l", Some("clavicle_l"), [0.17, 1.44, 0.0]),
            ("lowerarm_l", Some("upperarm_l"), [0.368, 1.242, 0.0]),
            ("hand_l", Some("lowerarm_l"), [0.552, 1.058, 0.0]),
            ("clavicle_r", Some("spine_02"), [-0.04, 1.42, 0.0]),
            ("upperarm_r", Some("clavicle_r"), [-0.17, 1.44, 0.0]),
            ("lowerarm_r", Some("upperarm_r"), [-0.368, 1.242, 0.0]),
            ("hand_r", Some("lowerarm_r"), [-0.552, 1.058, 0.0]),
            ("neck_01", Some("spine_02"), [0.0, 1.47, 0.0]),
            ("head", Some("neck_01"), [0.0, 1.58, 0.0]),
            ("thigh_l", Some("pelvis"), [0.09, 0.93, 0.0]),
            ("calf_l", Some("thigh_l"), [0.10, 0.51, 0.0]),
            ("foot_l", Some("calf_l"), [0.11, 0.09, -0.03]),
            ("ball_l", Some("foot_l"), [0.12, 0.02, 0.10]),
            ("thigh_r", Some("pelvis"), [-0.09, 0.93, 0.0]),
            ("calf_r", Some("thigh_r"), [-0.10, 0.51, 0.0]),
            ("foot_r", Some("calf_r"), [-0.11, 0.09, -0.03]),
            ("ball_r", Some("foot_r"), [-0.12, 0.02, 0.10]),
        ];
        let mut skeleton = Self::from_positions(
            BONES
                .iter()
                .map(|(name, parent, at)| (*name, *parent, Vec3::from_array(*at))),
        );
        // The segment-mass table sums to 80.02 kg.
        skeleton.mass = Mass::try_from(80.02).ok();
        skeleton
    }

    /// Spawns the bones under `character` at their rest pose and returns them.
    ///
    /// Each bone entity gets a `Name` and a local `Transform`. A runtime
    /// [`crate::Ragdoll`] on `character` binds to these bones by name.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::prelude::{Commands, Transform};
    /// use bevy_ragdoll::{Ragdoll, Skeleton};
    ///
    /// fn spawn_character(mut commands: Commands) {
    ///     let character = commands.spawn((Transform::default(), Ragdoll::default())).id();
    ///     Skeleton::humanoid().spawn(&mut commands, character);
    /// }
    /// # bevy::ecs::system::assert_is_system(spawn_character);
    /// ```
    pub fn spawn(
        &self,
        commands: &mut bevy::prelude::Commands<'_, '_>,
        character: bevy::prelude::Entity,
    ) -> Vec<bevy::prelude::Entity> {
        use bevy::prelude::{ChildOf, Name, Transform};
        // Bones are parent-first, so each parent entity exists before its children.
        let mut entities: Vec<bevy::prelude::Entity> = Vec::with_capacity(self.bones.len());
        for bone in &self.bones {
            let parent = bone.parent.filter(|parent| *parent < entities.len());
            let (parent_entity, local) = parent.map_or((character, bone.rest), |parent| {
                (
                    entities[parent],
                    self.bones[parent].rest.inverse() * bone.rest,
                )
            });
            // Children store their rest pose relative to the parent bone.
            let transform =
                Transform::from_translation(local.translation.into()).with_rotation(local.rotation);
            let mut entity = commands.spawn((
                Name::new(bone.name.clone()),
                transform,
                ChildOf(parent_entity),
            ));
            // Only non-default overrides need a component; the generator treats absence as default.
            if bone.overrides != RagdollBone::default() {
                entity.insert(bone.overrides);
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
                let parent = self.bones[index].parent;
                let incoming = parent
                    .and_then(|parent| {
                        Vec3::from(head - self.bones[parent].rest.translation).try_normalize()
                    })
                    .unwrap_or(Vec3::ZERO);
                // Follow the child that best continues the incoming direction.
                let child = self
                    .bones
                    .iter()
                    .filter(|bone| bone.parent == Some(index))
                    .map(|bone| Vec3::from(bone.rest.translation - head))
                    // Ties keep the earlier child, so a root follows its first child.
                    .reduce(|best, next| {
                        let score = |v: Vec3| v.normalize_or_zero().dot(incoming);
                        if score(next) > score(best) {
                            next
                        } else {
                            best
                        }
                    });
                let toward = match (child, parent) {
                    (Some(child), _) => child,
                    (None, Some(parent)) => Vec3::from(head - self.bones[parent].rest.translation),
                    (None, None) => Vec3::Y,
                };
                toward.try_normalize().unwrap_or(Vec3::Y)
            })
            .collect::<Vec<_>>();
        for (bone, direction) in self.bones.iter_mut().zip(directions) {
            // Local Z stays as close to the +Z front as possible; bones that
            // point mostly forward or backward keep local X horizontal instead.
            let reference = if direction.z.abs() > 0.7 {
                Vec3::Y
            } else {
                Vec3::Z
            };
            let x = direction
                .cross(reference)
                .try_normalize()
                .unwrap_or_else(|| Vec3::Y.cross(direction).normalize());
            let z = x.cross(direction);
            bone.rest.rotation = Quat::from_mat3(&Mat3::from_cols(x, direction, z)).normalize();
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
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{RagdollProfile, Skeleton};
    ///
    /// let profile = RagdollProfile::from_skeleton(&Skeleton::humanoid())?;
    /// assert_eq!(profile.bodies()[0].bone(), "pelvis");
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    pub fn from_skeleton(skeleton: &Skeleton) -> Result<Self, ProfileError> {
        Self::new(ProfileSpec::from(skeleton))
    }
}
