//! Validated ragdoll profiles and their authoring data.
//!
//! A [`RagdollProfile`] stores parent-first bodies, validated joints, derived
//! contact exclusions, direct-child masks, and total mass. Construct profiles
//! from [`ProfileSpec`] or [`ProfileBuilder`] so the public boundary checks
//! topology, geometry, transforms, names, mass, and limits before a runtime or
//! backend reads the data. `BodyIndex` preserves the checked profile order
//! across crates.

use std::collections::HashSet;
use std::f32::consts::PI;

use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::Component;

mod body;
mod builder;
mod error;
mod geometry;
mod joint;
mod limits;
mod mass;
mod role;
mod spec;

pub use self::body::Body;
pub use self::builder::ProfileBuilder;
pub use self::error::{JointAxis, ProfileError};
pub use self::joint::Joint;
pub use self::limits::{AngleRange, JointLimits};
pub use self::mass::{Mass, MassError};
pub use self::role::BodyRole;
pub use self::spec::{BodySpec, JointSpec, ProfileSpec, ShapeSpec};

/// Maximum accepted body count because relationship masks reserve one bit per
/// body.
///
/// Profiles can contain body indexes from zero through sixty-three. The value
/// bounds contact-mask storage and makes every body relationship fit in `u64`.
pub const MAX_BODIES: usize = 64;

/// A validated ragdoll configuration with derived body relationships.
///
/// This profile stores parent-first bodies, their joints, contact exclusions,
/// direct-child masks, and total mass. Construct it from [`ProfileSpec`] to
/// validate the tree, geometry, mass, transforms, joint limits, and bone names.
#[derive(bevy::asset::Asset, Clone, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollProfile {
    /// Bodies preserve the profile's parent-first order for stable body
    /// indexes.
    bodies: Vec<Body>,
    /// Joints are stored in child-body order after tree validation succeeds.
    joints: Vec<Joint>,
    /// Each entry marks bodies excluded from contact with the indexed body.
    no_contact: Vec<u64>,
    /// The finite positive sum of every validated body mass, measured in
    /// kilograms.
    total_mass: Mass,
    /// Each entry marks the direct child bodies of the indexed body.
    children: Vec<u64>,
}

/// A checked profile position that fits the relationship masks.
///
/// The value represents only body indexes in `0..MAX_BODIES`, so callers cannot
/// construct an out-of-range mask position through the checked conversion.
#[derive(
    Component, Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, bevy::prelude::Reflect,
)]
pub struct BodyIndex(u8);

impl BodyIndex {
    /// Returns the validated body position as a `usize` for slice lookups.
    ///
    /// The conversion preserves the index exactly because the stored value is
    /// an unsigned byte and was checked against the profile's 64-body limit.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::BodyIndex;
    ///
    /// let index = BodyIndex::try_from(3_usize).map_err(|_| "out of range")?;
    /// assert_eq!(index.get(), 3);
    /// # Ok::<(), &str>(())
    /// ```
    pub const fn get(self) -> usize {
        self.0 as usize
    }
}

impl TryFrom<usize> for BodyIndex {
    type Error = usize;

    /// Converts a mask position when it falls inside the supported body range.
    fn try_from(index: usize) -> Result<Self, Self::Error> {
        if index < MAX_BODIES {
            Ok(Self(index as u8))
        } else {
            Err(index)
        }
    }
}

/// Validated profile data carried between checks and runtime construction.
struct ValidatedProfile {
    /// Body entries with checked indexes and finite positive masses.
    bodies: Vec<Body>,
    /// Joint entries ordered by child body after all validation succeeds.
    joints: Vec<Joint>,
    /// The validated sum of body masses in kilograms.
    total_mass: Mass,
}

/// Data validated before the later joint-limit and duplicate-name checks.
struct ProfilePrerequisites {
    /// Each non-root body has its unique parent joint at the matching slot.
    joints_by_child: Vec<Option<JointSpec>>,
    /// The checked mass corresponding to each source body entry.
    masses: Vec<Mass>,
    /// The checked sum of all source body masses, measured in kilograms.
    total_mass: Mass,
}

impl RagdollProfile {
    /// Validates profile data, then derives masks used by runtime contact
    /// systems.
    ///
    /// Validation checks the body tree, masses, shapes, transforms, joint
    /// limits, torque bounds, and duplicate bone names in a stable error order.
    /// Contact derivation takes O(n²) time and O(n) auxiliary space for at most
    /// 64 bodies.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::math::{Isometry3d, Vec3}; use bevy_ragdoll::{BodySpec,
    /// ProfileSpec, RagdollProfile, ShapeSpec};
    ///
    /// let spec = ProfileSpec { bodies: vec![BodySpec { bone:
    /// "pelvis".to_owned(), shape: ShapeSpec::Sphere { center: Vec3::ZERO,
    /// radius: 0.2 }, mass: 8.0, rest: Isometry3d::IDENTITY, role: None, }], joints:
    /// Vec::new(), }; let profile = RagdollProfile::new(spec)?;
    /// assert_eq!(profile.total_mass().kilograms(), 8.0);
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    pub fn new(spec: ProfileSpec) -> Result<Self, ProfileError> {
        // Validate authoring data before constructing runtime body and joint types.
        let validated = ValidatedProfile::try_from(spec)?;
        // Derive contact and child masks from the validated rest geometry and tree.
        let (no_contact, children) = derive_masks(&validated.bodies, &validated.joints)?;
        Ok(Self {
            bodies: validated.bodies,
            joints: validated.joints,
            no_contact,
            total_mass: validated.total_mass,
            children,
        })
    }
}

impl RagdollProfile {
    /// Returns bodies in parent-first profile order for stable skeleton
    /// binding.
    ///
    /// Every body has a checked index, validated mass, valid collision shape,
    /// and rigid rest transform. The returned slice borrows the profile data.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::RagdollProfile;
    /// # fn inspect(profile: &RagdollProfile) {
    /// let bodies = profile.bodies(); assert!(bodies.len() <=
    /// bevy_ragdoll::MAX_BODIES);
    /// # }
    /// ```
    pub fn bodies(&self) -> &[Body] {
        &self.bodies
    }

    /// Returns joints in child-body order after validation of the parent-first
    /// tree.
    ///
    /// Each joint connects one checked child index to an earlier parent index,
    /// and its frame, limits, and maximum torque passed profile validation.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::RagdollProfile;
    /// # fn inspect(profile: &RagdollProfile) {
    /// let joints = profile.joints(); assert!(joints.len() <
    /// profile.bodies().len());
    /// # }
    /// ```
    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    /// Returns the contact-exclusion bit mask for each body in profile order.
    ///
    /// Bit `j` in entry `i` excludes body `j` from contact with body `i`; joint
    /// neighbors and resting shapes within 0.01 metres set symmetric bits.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::RagdollProfile;
    /// # fn inspect(profile: &RagdollProfile) {
    /// let masks = profile.no_contact_masks(); assert_eq!(masks.len(),
    /// profile.bodies().len());
    /// # }
    /// ```
    pub fn no_contact_masks(&self) -> &[u64] {
        &self.no_contact
    }

    /// Returns a bit mask for each body's direct children in profile order.
    ///
    /// Bit `j` in entry `i` is set exactly when the validated joint tree names
    /// body `j` as a direct child of body `i`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::RagdollProfile;
    /// # fn inspect(profile: &RagdollProfile) {
    /// let masks = profile.children_masks();
    /// assert_eq!(masks.len(), profile.bodies().len());
    /// # }
    /// ```
    pub fn children_masks(&self) -> &[u64] {
        &self.children
    }

    /// Returns the sum of validated body masses without changing kilograms.
    ///
    /// Profile construction rejects non-finite totals, so the returned value
    /// remains a finite positive mass suitable for reporting or normalization.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::RagdollProfile;
    /// # fn total(profile: &RagdollProfile) -> f32 {
    /// profile.total_mass().kilograms()
    /// # }
    /// ```
    pub const fn total_mass(&self) -> Mass {
        self.total_mass
    }

    /// Finds a checked body index by its exact skeleton bone name.
    ///
    /// The lookup scans profile order and returns `None` when no body uses the
    /// supplied name; duplicate names cannot survive profile validation.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::{BodyIndex, RagdollProfile};
    /// # fn find(profile: &RagdollProfile) -> Option<BodyIndex> {
    /// profile.body_index("pelvis")
    /// # }
    /// ```
    pub fn body_index(&self, bone: &str) -> Option<BodyIndex> {
        self.bodies
            .iter()
            .find(|body| body.bone() == bone)
            .map(Body::index)
    }

    /// Finds the validated joint whose child is `body`; the root has no joint.
    ///
    /// The result borrows stored joint data and preserves the parent and child
    /// indexes, frame, angular limits, and torque limit from the profile.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::{BodyIndex, RagdollProfile};
    /// # fn find(profile: &RagdollProfile, body: BodyIndex) {
    /// let _joint = profile.joint_of(body);
    /// # }
    /// ```
    pub fn joint_of(&self, body: BodyIndex) -> Option<&Joint> {
        self.joints.iter().find(|joint| joint.child() == body)
    }

    /// Finds the first body with `role` in profile order.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyRole, RagdollProfile, Skeleton};
    ///
    /// let profile = RagdollProfile::from_skeleton(&Skeleton::humanoid())?;
    /// let head = profile.body_with_role(BodyRole::Head).map(|body| profile.bodies()[body.get()].bone());
    /// assert_eq!(head, Some("head"));
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    pub fn body_with_role(&self, role: BodyRole) -> Option<BodyIndex> {
        self.bodies
            .iter()
            .find(|body| body.role() == role)
            .map(Body::index)
    }
}

impl RagdollProfile {
    /// Lazily places rest transforms under `root` in parent-first profile
    /// order.
    ///
    /// Each yielded isometry composes the root with one body's validated rest
    /// frame. The iterator performs O(n) total work and uses O(1) auxiliary
    /// space.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy::math::Isometry3d;
    /// # use bevy_ragdoll::RagdollProfile;
    /// # fn rest(profile: &RagdollProfile) {
    /// let poses =
    /// profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    /// assert_eq!(poses.len(), profile.bodies().len());
    /// # }
    /// ```
    pub fn rest_poses(&self, root: Isometry3d) -> impl Iterator<Item = Isometry3d> + '_ {
        self.bodies.iter().map(move |body| root * body.rest())
    }

    /// Measures a child's rotation around joint X, twist, and Z axes in
    /// radians.
    ///
    /// The method returns `None` for the root, a missing child or parent pose,
    /// or any non-finite, degenerate, or non-unit pose rotation.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy::math::Isometry3d;
    /// # use bevy_ragdoll::{BodyIndex, RagdollProfile};
    /// # fn angles(profile: &RagdollProfile, child: BodyIndex) {
    /// let poses =
    /// profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>(); let
    /// _angles = profile.joint_angles(child, &poses);
    /// # }
    /// ```
    pub fn joint_angles(&self, child: BodyIndex, poses: &[Isometry3d]) -> Option<Vec3> {
        // A root body has no joint, and both related poses must be present.
        let joint = self.joint_of(child)?;
        let child_pose = *poses.get(child.get())?;
        let parent_pose = *poses.get(joint.parent().get())?;
        // Reject malformed rotations before quaternion operations.
        if !is_valid_rotation(child_pose.rotation) || !is_valid_rotation(parent_pose.rotation) {
            return None;
        }
        joint_angles(joint, parent_pose, child_pose)
    }
}

impl TryFrom<&ProfileSpec> for ProfilePrerequisites {
    type Error = ProfileError;

    /// Checks tree structure, mass, and shape before later joint validation
    /// stages.
    fn try_from(spec: &ProfileSpec) -> Result<Self, Self::Error> {
        // Body count and joint references establish the legal profile index range.
        let joints_by_child = arrange_joints(spec.bodies.len(), &spec.joints)?;
        // Validate each mass and its running total in parent-first profile order.
        let (masses, total_mass) = validate_masses(&spec.bodies)?;
        // Reject invalid rest transforms and collision shapes before joint limits.
        validate_body_shapes(&spec.bodies)?;
        Ok(Self {
            joints_by_child,
            masses,
            total_mass,
        })
    }
}

impl TryFrom<ProfileSpec> for ValidatedProfile {
    type Error = ProfileError;

    /// Converts valid authoring data into checked runtime body and joint
    /// entries.
    fn try_from(spec: ProfileSpec) -> Result<Self, Self::Error> {
        // Preserve validation order: tree structure, masses, and body shapes come first.
        let prerequisites = ProfilePrerequisites::try_from(&spec)?;
        // Joint limits and frames precede torque failures in the public error order.
        validate_joint_data(&prerequisites.joints_by_child)?;
        // Duplicate names are checked last so earlier profile errors retain precedence.
        validate_unique_bone_names(&spec.bodies)?;
        // Move each source body beside its checked mass without cloning retained data.
        let bodies = spec
            .bodies
            .into_iter()
            .zip(prerequisites.masses)
            .enumerate()
            .map(|(index, (body, mass))| {
                // Resolve optional authoring roles once before moving profile data into storage.
                let role = body
                    .role
                    .unwrap_or_else(|| BodyRole::from(body.bone.as_str()));
                Body::new(
                    BodyIndex(index as u8),
                    body.bone,
                    body.shape,
                    mass,
                    body.rest,
                    role,
                )
            })
            .collect();
        // The validated slot vector has one entry for each non-root body.
        let joints = prerequisites
            .joints_by_child
            .into_iter()
            .enumerate()
            .skip(1)
            .filter_map(|(child, joint)| joint.map(|joint| (child, joint)))
            .map(|(child, joint)| {
                Joint::new(
                    BodyIndex(child as u8),
                    BodyIndex(joint.parent),
                    joint.frame,
                    joint.limits,
                    joint.max_torque,
                )
            })
            .collect();
        Ok(Self {
            bodies,
            joints,
            total_mass: prerequisites.total_mass,
        })
    }
}

/// Places each joint in its child slot and enforces a parent-first tree.
fn arrange_joints(
    body_count: usize,
    joint_specs: &[JointSpec],
) -> Result<Vec<Option<JointSpec>>, ProfileError> {
    // Reject impossible profile sizes before allocating relationship slots.
    validate_body_count(body_count)?;
    let mut joints_by_child = vec![None; body_count];
    // Place each validated parent-first joint in the slot named by its child.
    for joint in joint_specs.iter().copied() {
        insert_joint(&mut joints_by_child, joint)?;
    }
    // Every non-root body must have exactly one incoming joint.
    if joints_by_child.iter().skip(1).any(Option::is_none) {
        return Err(ProfileError::NotATree);
    }
    Ok(joints_by_child)
}

/// Checks the closed body-count range before profile-sized storage is
/// allocated.
fn validate_body_count(body_count: usize) -> Result<(), ProfileError> {
    // Test the lower boundary before the upper bound so an empty profile is distinct.
    if body_count == 0 {
        return Err(ProfileError::Empty);
    }
    if body_count > MAX_BODIES {
        return Err(ProfileError::TooManyBodies(body_count));
    }
    Ok(())
}

/// Inserts one unique joint whose child follows its parent in profile order.
fn insert_joint(
    joints_by_child: &mut [Option<JointSpec>],
    joint: JointSpec,
) -> Result<(), ProfileError> {
    let child = usize::from(joint.child);
    let parent = usize::from(joint.parent);
    // The root has no incoming joint, and every parent must precede its child.
    if child == 0 || parent >= child {
        return Err(ProfileError::NotATree);
    }
    // The array bound and parent order are checked before writing the child slot.
    let child_slot = joints_by_child
        .get_mut(child)
        .ok_or(ProfileError::NotATree)?;
    if child_slot.replace(joint).is_some() {
        return Err(ProfileError::NotATree);
    }
    Ok(())
}

/// Checks each positive body mass and the finite total accumulated in profile
/// order.
fn validate_masses(body_specs: &[BodySpec]) -> Result<(Vec<Mass>, Mass), ProfileError> {
    let mut masses = Vec::with_capacity(body_specs.len());
    let mut total_mass = 0.0_f32;
    // Validate each body mass and reject overflow at the body that caused it.
    for (index, body) in body_specs.iter().enumerate() {
        // The earlier body-count check bounds every index to the stored u8 range.
        let checked_index = BodyIndex(index as u8);
        let mass = Mass::try_from(body.mass).map_err(|error| match error {
            MassError => ProfileError::BadMass {
                body: checked_index,
            },
        })?;
        total_mass += mass.kilograms();
        if !total_mass.is_finite() {
            return Err(ProfileError::BadMass {
                body: checked_index,
            });
        }
        masses.push(mass);
    }
    // The body-count check guarantees at least one body before selecting its error index.
    let last_body = BodyIndex((body_specs.len().saturating_sub(1)) as u8);
    let total_mass = validate_total_mass(total_mass, last_body)?;
    Ok((masses, total_mass))
}

/// Converts the accumulated kilogram value while retaining its final body
/// context.
fn validate_total_mass(kilograms: f32, last_body: BodyIndex) -> Result<Mass, ProfileError> {
    Mass::try_from(kilograms).map_err(|MassError| ProfileError::BadMass { body: last_body })
}

/// Rejects body entries with empty bone names, invalid rests, or invalid
/// shapes.
fn validate_body_shapes(body_specs: &[BodySpec]) -> Result<(), ProfileError> {
    // Names and rest frames fail through BadShape before any joint-data checks.
    for (index, body) in body_specs.iter().enumerate() {
        if body.bone.is_empty() || !is_valid_isometry(body.rest) || !is_valid_shape(&body.shape) {
            return Err(ProfileError::BadShape {
                body: body_index(index)?,
            });
        }
    }
    Ok(())
}

/// Checks all joint limits and frames before checking motor torque values.
fn validate_joint_data(joints_by_child: &[Option<JointSpec>]) -> Result<(), ProfileError> {
    // Preserve public error order by completing all geometry checks first.
    validate_joint_limits_and_frames(joints_by_child)?;
    // Torque diagnostics follow every range and frame diagnostic.
    validate_joint_torques(joints_by_child)
}

/// Checks each angular range and rigid joint frame in child-body order.
fn validate_joint_limits_and_frames(
    joints_by_child: &[Option<JointSpec>],
) -> Result<(), ProfileError> {
    // Angular limits and frames precede torque checks to preserve public error order.
    for joint in joints_by_child.iter().skip(1).flatten() {
        // Report the first invalid axis using the profile's X, twist, Z order.
        for (axis, range) in [
            (JointAxis::X, joint.limits.x),
            (JointAxis::Twist, joint.limits.twist),
            (JointAxis::Z, joint.limits.z),
        ] {
            if !is_valid_angle_range(range) {
                return Err(ProfileError::BadLimit {
                    joint: BodyIndex(joint.child),
                    axis,
                });
            }
        }
        // A finite unit frame rotation and finite translation define the constraint anchor.
        if !is_valid_isometry(joint.frame) {
            return Err(ProfileError::BadLimit {
                joint: BodyIndex(joint.child),
                axis: JointAxis::Frame,
            });
        }
    }
    Ok(())
}

/// Checks finite nonnegative motor torque after every joint geometry check
/// passes.
fn validate_joint_torques(joints_by_child: &[Option<JointSpec>]) -> Result<(), ProfileError> {
    // Preserve child order so the first bad motor reports deterministically.
    for joint in joints_by_child.iter().skip(1).flatten() {
        if !(joint.max_torque.is_finite() && joint.max_torque >= 0.0) {
            return Err(ProfileError::BadTorque {
                joint: BodyIndex(joint.child),
            });
        }
    }
    Ok(())
}

/// Rejects repeated skeleton bone names after the earlier validation stages.
fn validate_unique_bone_names(body_specs: &[BodySpec]) -> Result<(), ProfileError> {
    let mut bone_names = HashSet::with_capacity(body_specs.len());
    // Use borrowed names so validation does not allocate a second copy of each string.
    for body in body_specs {
        // Stop at the first repeat so the duplicate name stays deterministic.
        if !bone_names.insert(body.bone.as_str()) {
            return Err(ProfileError::DuplicateBone {
                bone: body.bone.clone(),
            });
        }
    }
    Ok(())
}

/// Converts an internal slice position after enforcing the 64-body boundary.
fn body_index(index: usize) -> Result<BodyIndex, ProfileError> {
    BodyIndex::try_from(index).map_err(|count| ProfileError::TooManyBodies(count.saturating_add(1)))
}

/// Derives contact exclusions and direct-child masks from validated
/// relationships.
///
/// This scans every unordered body pair for resting contact, using O(n²) time
/// and O(n) auxiliary mask storage with `n <= MAX_BODIES`.
fn derive_masks(bodies: &[Body], joints: &[Joint]) -> Result<(Vec<u64>, Vec<u64>), ProfileError> {
    let mut no_contact = vec![0_u64; bodies.len()];
    let mut children = vec![0_u64; bodies.len()];
    // Joint relationships populate symmetric contact exclusions and child masks.
    add_joint_relationship_masks(joints, &mut no_contact, &mut children)?;
    // Resting geometry adds symmetric exclusions independently of tree adjacency.
    add_rest_contact_masks(bodies, &mut no_contact)?;
    Ok((no_contact, children))
}

/// Writes contact exclusions and direct-child relationships for every joint.
fn add_joint_relationship_masks(
    joints: &[Joint],
    no_contact: &mut [u64],
    children: &mut [u64],
) -> Result<(), ProfileError> {
    // Each validated joint excludes its neighboring bodies from contact.
    for joint in joints {
        // Store both symmetric collision directions for the pair.
        set_mask_bit(no_contact, joint.child(), joint.parent())?;
        set_mask_bit(no_contact, joint.parent(), joint.child())?;
        set_mask_bit(children, joint.parent(), joint.child())?;
    }
    Ok(())
}

/// Adds symmetric exclusions for every pair inside the rest-contact margin.
fn add_rest_contact_masks(bodies: &[Body], no_contact: &mut [u64]) -> Result<(), ProfileError> {
    // Compare each unordered pair once, retaining deterministic profile order.
    for (first_index, first) in bodies.iter().enumerate() {
        // Skip the same body and every earlier pair already checked.
        for second in bodies.iter().skip(first_index + 1) {
            let first_body = first.index();
            let second_body = second.index();
            // Shapes closer than the margin need a symmetric contact exclusion.
            if geometry::shapes_are_within_rest_contact_margin(
                first.shape(),
                first.rest(),
                second.shape(),
                second.rest(),
            ) {
                set_mask_bit(no_contact, first_body, second_body)?;
                set_mask_bit(no_contact, second_body, first_body)?;
            }
        }
    }
    Ok(())
}

/// Sets one checked relationship bit in the mask for `target`.
fn set_mask_bit(
    masks: &mut [u64],
    target: BodyIndex,
    related: BodyIndex,
) -> Result<(), ProfileError> {
    let mask = masks.get_mut(target.get()).ok_or(ProfileError::NotATree)?;
    *mask |= 1_u64 << related.get();
    Ok(())
}

/// Measures the child's joint angles after validating both pose rotations.
fn joint_angles(joint: &Joint, parent_pose: Isometry3d, child_pose: Isometry3d) -> Option<Vec3> {
    // Compose the parent pose with the joint frame before measuring child rotation.
    let frame_rotation = parent_pose.rotation * joint.frame().rotation;
    let relative = frame_rotation.inverse() * child_pose.rotation;
    if !is_valid_rotation(relative) {
        return None;
    }
    // Normalize equivalent quaternion signs so angles use the shortest representation.
    let relative = relative.normalize();
    let shortest = if relative.w < 0.0 {
        -relative
    } else {
        relative
    };
    Some(Vec3::new(
        2.0 * shortest.x.atan2(shortest.w),
        2.0 * shortest.y.atan2(shortest.w),
        2.0 * shortest.z.atan2(shortest.w),
    ))
}

/// Returns whether a quaternion is finite, nondegenerate, and unit length.
fn is_valid_rotation(rotation: Quat) -> bool {
    let length_squared = rotation.length_squared();
    rotation.is_finite() && length_squared.is_finite() && (length_squared - 1.0).abs() <= 1.0e-4
}

/// Returns whether all transform components form a rigid isometry.
fn is_valid_isometry(isometry: Isometry3d) -> bool {
    isometry.translation.is_finite() && is_valid_rotation(isometry.rotation)
}

/// Returns whether a body shape has finite geometry and valid dimensions.
fn is_valid_shape(shape: &ShapeSpec) -> bool {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => is_valid_capsule(*a, *b, *radius),
        ShapeSpec::Sphere { center, radius } => is_valid_sphere(*center, *radius),
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => is_valid_cuboid(*center, *rotation, *half_extents),
    }
}

/// Validates finite capsule endpoints and a positive finite radius in metres.
fn is_valid_capsule(a: Vec3, b: Vec3, radius: f32) -> bool {
    a.is_finite() && b.is_finite() && radius.is_finite() && radius > 0.0
}

/// Validates a finite sphere centre and a positive finite radius in metres.
fn is_valid_sphere(center: Vec3, radius: f32) -> bool {
    center.is_finite() && radius.is_finite() && radius > 0.0
}

/// Validates a finite cuboid centre, rigid rotation, and positive half extents.
fn is_valid_cuboid(center: Vec3, rotation: Quat, half_extents: Vec3) -> bool {
    center.is_finite()
        && half_extents.is_finite()
        && half_extents.min_element() > 0.0
        && is_valid_rotation(rotation)
}

/// Returns whether a joint interval contains zero and stays inside `-PI..=PI`.
fn is_valid_angle_range(range: AngleRange) -> bool {
    range.min.is_finite()
        && range.max.is_finite()
        && range.min <= 0.0
        && range.max >= 0.0
        && range.min >= -PI
        && range.max <= PI
}

#[cfg(test)]
mod tests;
