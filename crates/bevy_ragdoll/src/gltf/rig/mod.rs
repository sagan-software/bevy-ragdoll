//! A bounded glTF 2.0 GLB reader for skeletal ragdoll profiles.
//!
//! The reader adopts the Khronos glTF 2.0.1 specification, sections 3.2 and 4.4:
//! <https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html>.
//! It requires the JSON chunk first, allows at most one BIN chunk second, checks
//! chunk bounds and four-byte alignment, and ignores unknown chunk payloads.
//! It accepts GLB container version 2, exact JSON `asset.version` `2.0`, and an
//! absent or exact `asset.minVersion` `2.0`. It rejects a valid lower
//! `minVersion` because this local profile requires exact `2.0` metadata.
//! It rejects malformed `extensionsRequired` members and every nonempty list
//! because the importer implements no glTF extensions; it ignores
//! `extensionsUsed` and optional extension members. It reads node hierarchy
//! and transforms, skin joints, mesh POSITION bounds, and application extras;
//! other glTF fields are ignored.
//! Rig bones require a positive `tgf_length`; capsule nodes require Skein body
//! data and a round local mesh bound. Accepted unit quaternions are normalized
//! after parsing, and bone scale is dropped from the profile rest frame after
//! it places capsule endpoints in bone space.

use std::collections::HashSet;

use bevy::math::{Isometry3d, Mat4, Quat, Vec3};
use serde::de::DeserializeOwned;
use serde_json::Value;

use crate::profile::{BodySpec, JointLimits, JointSpec, ProfileSpec, ShapeSpec};
use crate::skein::{AngleRange, RagdollBody, RagdollJoint};

mod container;

#[cfg(test)]
use self::container::{BIN_CHUNK, JSON_CHUNK, json_chunk, read_u32, validate_remaining_chunks};
use self::container::{parse_document, validate_asset_profile};

/// Maximum difference between imported capsule radii and extents.
const ROUND_TOLERANCE: f32 = 0.05;
/// Reflected path used by TGF's earlier Skein body component.
const OLD_BODY_PATH: &str = "tgf_rig::ragdoll::RagdollBody";
/// Reflected path used by TGF's earlier Skein joint component.
const OLD_JOINT_PATH: &str = "tgf_rig::ragdoll::RagdollJoint";
/// Reflected path used by this crate's Skein body component.
const NEW_BODY_PATH: &str = "bevy_ragdoll::skein::RagdollBody";
/// Reflected path used by this crate's Skein joint component.
const NEW_JOINT_PATH: &str = "bevy_ragdoll::skein::RagdollJoint";

/// A GLB container, JSON document, or skeletal rig import failure.
///
/// The variants separate malformed glTF JSON from an invalid or unsupported
/// ragdoll rig. Display text adds the parser context needed to locate bad data.
/// Callers can match the variant before reporting the contained diagnostic.
///
/// # Examples
///
/// ```no_run
/// use bevy_ragdoll::gltf::GltfRigError;
///
/// fn report(error: GltfRigError) {
///     match error {
///         GltfRigError::Invalid(_) | GltfRigError::Json(_) => eprintln!("{error}"),
///     }
/// }
/// ```
#[derive(Debug, thiserror::Error)]
pub enum GltfRigError {
    /// The container or skeletal rig violates the importer's supported profile.
    ///
    /// This variant retains the specific structural or metadata problem while
    /// providing a stable machine-readable failure category to asset callers.
    #[error("invalid GLB ragdoll rig: {0}")]
    Invalid(#[source] GltfRigValidationError),
    /// The JSON chunk contains bytes that serde_json cannot parse as a document.
    ///
    /// This variant preserves serde_json's source error for callers that need
    /// the original line, column, or token-level parsing diagnostic.
    #[error("invalid GLB JSON: {0}")]
    Json(#[from] serde_json::Error),
}

/// A detailed validation diagnostic for malformed or unsupported rig data.
///
/// This error carries contextual text for failed container, hierarchy,
/// transform, Skein annotation, or capsule-bound checks. Use its `Display`
/// implementation when reporting the specific failure to an asset author.
#[derive(Debug, thiserror::Error)]
#[error("{message}")]
pub struct GltfRigValidationError {
    /// The specific validation failure produced by the importer.
    message: String,
}

impl From<String> for GltfRigValidationError {
    fn from(message: String) -> Self {
        Self { message }
    }
}

impl From<&str> for GltfRigValidationError {
    fn from(message: &str) -> Self {
        Self::new(message)
    }
}

impl GltfRigValidationError {
    /// Creates a validation diagnostic from a parser-stage message.
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl ProfileSpec {
    /// Reads bones, rest transforms, capsule bounds, and Skein extras from GLB bytes.
    ///
    /// The parser accepts a GLB 2.0 container and exact glTF asset version 2.0.
    /// It rejects required extensions and applies the module's documented
    /// `minVersion` policy before reading supported rig metadata.
    /// It returns unvalidated profile data for [`crate::RagdollProfile::new`] to check.
    /// The `gltf` Cargo feature enables this method and its typed error module.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use bevy_ragdoll::ProfileSpec;
    ///
    /// let bytes = std::fs::read("assets/rigs/human.glb")?;
    /// let spec = ProfileSpec::from_glb(&bytes)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn from_glb(bytes: &[u8]) -> Result<Self, GltfRigError> {
        from_glb(bytes)
    }
}

/// Reads the GLB's rig nodes, capsule bounds, and Skein extras.
pub(crate) fn from_glb(bytes: &[u8]) -> Result<ProfileSpec, GltfRigError> {
    // Decode the container and JSON before interpreting glTF semantics.
    let document = parse_document(bytes)?;
    // Keep profile construction separate from the JSON byte boundary.
    import_document(&document).map_err(GltfRigError::Invalid)
}

/// Imports a supported glTF rig into unvalidated profile authoring data.
fn import_document(document: &Value) -> Result<ProfileSpec, GltfRigValidationError> {
    // Validate version and required extensions before following document references.
    validate_asset_profile(document)?;
    // Resolve parent links and world transforms before reading skin data.
    let nodes = NodeTree::new(document)?;
    // Preserve skeleton order when mapping bones, bodies, and joints.
    let (bones, bone_for_node) = read_bones(document, &nodes)?;
    let (bodies, joints) = read_bodies(document, &nodes, &bones, &bone_for_node)?;
    Ok(ProfileSpec { bodies, joints })
}

/// Node records with parent indices and global transforms.
struct NodeTree<'a> {
    /// The source glTF node array.
    nodes: &'a [Value],
    /// Each node's optional direct parent.
    parents: Vec<Option<usize>>,
    /// Each node's global transform from the root.
    world: Vec<Mat4>,
}

/// A node's state during parent-first transform traversal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum VisitState {
    /// The traversal has not reached this node.
    Unseen,
    /// The node is on the current ancestor chain.
    Visiting,
    /// The node's world transform has been resolved.
    Complete,
}

impl<'a> NodeTree<'a> {
    /// Resolves one-parent trees and composes every node transform.
    ///
    /// Node order comes from the document, and every parent index must point
    /// within that array. The traversal rejects cycles before storing transforms.
    fn new(document: &'a Value) -> Result<Self, GltfRigValidationError> {
        // Read the node array before validating references into it.
        let nodes = document
            .get("nodes")
            .and_then(Value::as_array)
            .ok_or_else(|| GltfRigValidationError::new("the glTF nodes member is not an array"))?;
        // Resolve one direct parent per node and reject invalid child references.
        let parents = parent_links(nodes)?;
        // Parse local transforms before composing their parent transforms.
        let local = nodes
            .iter()
            .enumerate()
            .map(|(index, node)| {
                local_transform(node).map_err(|problem| {
                    GltfRigValidationError::new(format!("node {index}: {problem}"))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        // Resolve each node once, rejecting any parent cycle.
        let world = world_transforms(&parents, &local)?;
        Ok(Self {
            nodes,
            parents,
            world,
        })
    }

    /// Returns a node's optional name.
    fn name(&self, index: usize) -> Option<&'a str> {
        self.nodes.get(index)?.get("name").and_then(Value::as_str)
    }

    /// Returns a node's optional extras object.
    fn extras(&self, index: usize) -> Option<&'a serde_json::Map<String, Value>> {
        self.nodes
            .get(index)?
            .get("extras")
            .and_then(Value::as_object)
    }
}

/// Builds the direct-parent table from each node's optional `children` member.
fn parent_links(nodes: &[Value]) -> Result<Vec<Option<usize>>, GltfRigValidationError> {
    let mut parents = vec![None; nodes.len()];
    // Process source children in node order to preserve deterministic parent validation.
    for (parent, node) in nodes.iter().enumerate() {
        let Some(children_value) = node.get("children") else {
            continue;
        };
        // Child references must be integer indices into the node array.
        let children = children_value
            .as_array()
            .ok_or_else(|| format!("node {parent}: children is not an array"))?;
        for child_value in children {
            let child = child_value
                .as_u64()
                .and_then(|child| usize::try_from(child).ok())
                .filter(|child| *child < nodes.len() && *child != parent)
                .ok_or_else(|| format!("node {parent}: child index is invalid"))?;
            // A child may have only one parent in the imported node tree.
            let parent_slot = slot_mut(&mut parents, child, "node parent")?;
            if parent_slot.replace(parent).is_some() {
                return Err(format!("node {child} has more than one parent").into());
            }
        }
    }
    Ok(parents)
}

/// Resolves transforms with an iterative traversal that detects parent cycles.
fn world_transforms(
    parents: &[Option<usize>],
    local: &[Mat4],
) -> Result<Vec<Mat4>, GltfRigValidationError> {
    // Allocate one traversal state and output transform per source node.
    let mut states = vec![VisitState::Unseen; parents.len()];
    let mut world = vec![Mat4::IDENTITY; parents.len()];
    // Resolve each chain parent-first and reuse completed ancestor transforms.
    for start in 0..parents.len() {
        resolve_world_transform(start, parents, local, &mut states, &mut world)?;
    }
    Ok(world)
}

/// Resolves one node's parent chain and stores each newly completed world transform.
fn resolve_world_transform(
    start: usize,
    parents: &[Option<usize>],
    local: &[Mat4],
    states: &mut [VisitState],
    world: &mut [Mat4],
) -> Result<(), GltfRigValidationError> {
    // Build the child-to-root chain without recomputing completed ancestors.
    let chain = world_parent_chain(start, parents, states)?;
    // Store transforms while unwinding so each parent is ready before its child.
    store_world_chain(chain, parents, local, states, world)
}

/// Marks unseen nodes while walking to a root or an already completed ancestor.
fn world_parent_chain(
    start: usize,
    parents: &[Option<usize>],
    states: &mut [VisitState],
) -> Result<Vec<usize>, GltfRigValidationError> {
    // Walk toward the root until a completed ancestor or an invalid cycle appears.
    let mut chain = Vec::new();
    let mut cursor = Some(start);
    // Stop at a cached parent transform so its resolved prefix can be reused.
    while let Some(index) = cursor {
        match *slot(states, index, "node visit state")? {
            VisitState::Complete => break,
            VisitState::Visiting => return Err("the node hierarchy contains a cycle".into()),
            VisitState::Unseen => {}
        }
        *slot_mut(states, index, "node visit state")? = VisitState::Visiting;
        chain.push(index);
        cursor = *slot(parents, index, "node parent")?;
    }
    Ok(chain)
}

/// Stores a parent chain in reverse traversal order after its references validate.
fn store_world_chain(
    mut chain: Vec<usize>,
    parents: &[Option<usize>],
    local: &[Mat4],
    states: &mut [VisitState],
    world: &mut [Mat4],
) -> Result<(), GltfRigValidationError> {
    // Unwind the chain so every parent world transform exists before its child.
    while let Some(index) = chain.pop() {
        store_world_transform(index, parents, local, states, world)?;
    }
    Ok(())
}

/// Composes and validates one node's world transform before marking it complete.
fn store_world_transform(
    index: usize,
    parents: &[Option<usize>],
    local: &[Mat4],
    states: &mut [VisitState],
    world: &mut [Mat4],
) -> Result<(), GltfRigValidationError> {
    // Compose the local transform with its already-resolved parent transform.
    let transform = compose_world_transform(index, parents, local, world)?;
    // Reject overflow or invalid inputs before committing the transform state.
    if !is_finite_matrix(transform) {
        return Err(format!("node {index}: global transform is not finite").into());
    }
    // Write the transform before marking its traversal state complete.
    *slot_mut(world, index, "node world transform")? = transform;
    *slot_mut(states, index, "node visit state")? = VisitState::Complete;
    Ok(())
}

/// Composes one local transform with its parent when a parent exists.
fn compose_world_transform(
    index: usize,
    parents: &[Option<usize>],
    local: &[Mat4],
    world: &[Mat4],
) -> Result<Mat4, GltfRigValidationError> {
    let local_transform = *slot(local, index, "node local transform")?;
    match *slot(parents, index, "node parent")? {
        Some(parent) => Ok(*slot(world, parent, "parent world transform")? * local_transform),
        None => Ok(local_transform),
    }
}

/// Reads an indexed slice entry or reports the violated internal bound.
fn slot<'a, T>(
    values: &'a [T],
    index: usize,
    description: &str,
) -> Result<&'a T, GltfRigValidationError> {
    values.get(index).ok_or_else(|| {
        GltfRigValidationError::new(format!("{description} index {index} is out of bounds"))
    })
}

/// Reads an indexed mutable slice entry or reports the violated internal bound.
fn slot_mut<'a, T>(
    values: &'a mut [T],
    index: usize,
    description: &str,
) -> Result<&'a mut T, GltfRigValidationError> {
    values.get_mut(index).ok_or_else(|| {
        GltfRigValidationError::new(format!("{description} index {index} is out of bounds"))
    })
}

/// Imports bone data in depth-first, parent-first order.
fn read_bones(
    document: &Value,
    nodes: &NodeTree<'_>,
) -> Result<(Vec<Bone>, Vec<Option<usize>>), GltfRigValidationError> {
    // Resolve the unique skin's indexed joints before following hierarchy links.
    let joint_nodes = skin_joint_nodes(document, nodes.nodes.len())?;
    // Map each skin joint to its nearest joint ancestor and stable tree order.
    let parent_joint = nearest_joint_parents(&joint_nodes, nodes)?;
    read_ordered_bones(joint_nodes, parent_joint, nodes)
}

/// Reads the sole skin's nonempty joint list and rejects duplicate or invalid nodes.
fn skin_joint_nodes(
    document: &Value,
    node_count: usize,
) -> Result<Vec<usize>, GltfRigValidationError> {
    // Read the single accepted skin before allocating node-indexed markers.
    let joint_values = single_skin_joint_values(document)?;
    let mut is_joint = vec![false; node_count];
    let mut joint_nodes = Vec::with_capacity(joint_values.len());
    // Validate each node reference before preserving source skin order.
    for value in joint_values {
        let node = value
            .as_u64()
            .and_then(|node| usize::try_from(node).ok())
            .filter(|node| *node < node_count)
            .ok_or_else(|| GltfRigValidationError::new("a skin joint index is not a node"))?;
        let seen = slot_mut(&mut is_joint, node, "skin joint marker")?;
        // A skin cannot list the same joint node more than once.
        if std::mem::replace(seen, true) {
            return Err(format!("skin joint node {node} is listed twice").into());
        }
        // Preserve source skin order until the deterministic tree walk begins.
        joint_nodes.push(node);
    }
    Ok(joint_nodes)
}

/// Returns the nonempty joint array from exactly one glTF skin.
fn single_skin_joint_values(document: &Value) -> Result<&[Value], GltfRigValidationError> {
    // The importer supports one skin so each source node has one skeleton role.
    let skins = document
        .get("skins")
        .and_then(Value::as_array)
        .ok_or_else(|| GltfRigValidationError::new("the glTF skins member is not an array"))?;
    if skins.len() != 1 {
        return Err(format!("the GLB has {} skins; a rig has one", skins.len()).into());
    }
    let skin = skins
        .first()
        .ok_or_else(|| GltfRigValidationError::new("the GLB has no skin"))?;
    // Require a nonempty joint list before any node indices enter the tree.
    let joint_values = skin
        .get("joints")
        .and_then(Value::as_array)
        .filter(|joints| !joints.is_empty())
        .ok_or_else(|| GltfRigValidationError::new("the skin has no joint node array"))?;
    Ok(joint_values)
}

/// Sorts, validates, and imports bones in deterministic parent-first order.
fn read_ordered_bones(
    mut joint_nodes: Vec<usize>,
    parent_joint: Vec<Option<usize>>,
    nodes: &NodeTree<'_>,
) -> Result<(Vec<Bone>, Vec<Option<usize>>), GltfRigValidationError> {
    // Sort by source node before collecting roots and traversing children.
    joint_nodes.sort_unstable();
    // Require exactly one root and preserve deterministic depth-first order.
    let order = bone_depth_first_order(&joint_nodes, &parent_joint, nodes.nodes.len())?;
    // Reject duplicate source names before constructing any owned bone data.
    let named_order = validate_bone_names(order, nodes)?;
    // Store imported bones by source node index for later body attachment lookup.
    import_ordered_bones(named_order, &parent_joint, nodes)
}

/// Requires every joint name to be nonempty and unique in the source document.
fn validate_bone_names<'a>(
    order: Vec<usize>,
    nodes: &NodeTree<'a>,
) -> Result<Vec<(usize, &'a str)>, GltfRigValidationError> {
    let mut names = HashSet::with_capacity(order.len());
    let mut named_order = Vec::with_capacity(order.len());
    // Borrow names from the JSON document so this check creates no owned copies.
    for node in order {
        let name = nodes
            .name(node)
            .filter(|name| !name.is_empty())
            .ok_or_else(|| {
                GltfRigValidationError::new(format!("joint node {node} has no non-empty name"))
            })?;
        // Distinct names make profile bone lookup unambiguous after import.
        if !names.insert(name) {
            return Err(format!("bone {name} is listed twice").into());
        }
        // Carry the validated name into construction without repeating a failing lookup.
        named_order.push((node, name));
    }
    Ok(named_order)
}

/// Imports each ordered bone and records its node-to-bone mapping.
fn import_ordered_bones(
    order: Vec<(usize, &str)>,
    parent_joint: &[Option<usize>],
    nodes: &NodeTree<'_>,
) -> Result<(Vec<Bone>, Vec<Option<usize>>), GltfRigValidationError> {
    // Allocate outputs in the validated parent-first order.
    let mut bone_for_node = vec![None; nodes.nodes.len()];
    let mut bones = Vec::with_capacity(order.len());
    // Parent-first order guarantees the parent mapping exists before each child.
    for (node, name) in order {
        let parent = parent_bone_index(node, parent_joint, &bone_for_node)?;
        // Validate the complete source rest frame before committing its mapping.
        let bone = read_bone(node, name, parent, nodes)?;
        let index = bones.len();
        *slot_mut(&mut bone_for_node, node, "bone mapping")? = Some(index);
        bones.push(bone);
    }
    Ok((bones, bone_for_node))
}

/// Resolves one source joint's parent node to its already imported bone index.
fn parent_bone_index(
    node: usize,
    parent_joint: &[Option<usize>],
    bone_for_node: &[Option<usize>],
) -> Result<Option<usize>, GltfRigValidationError> {
    match *slot(parent_joint, node, "joint parent")? {
        Some(parent_node) => slot(bone_for_node, parent_node, "parent bone mapping")?
            .map(Some)
            .ok_or_else(|| GltfRigValidationError::new("parent bone was not imported")),
        None => Ok(None),
    }
}

/// Finds each joint's nearest joint ancestor, skipping non-joint source nodes.
fn nearest_joint_parents(
    joint_nodes: &[usize],
    nodes: &NodeTree<'_>,
) -> Result<Vec<Option<usize>>, GltfRigValidationError> {
    let mut is_joint = vec![false; nodes.nodes.len()];
    // Mark all skin joints before searching source parent chains.
    for &node in joint_nodes {
        *slot_mut(&mut is_joint, node, "skin joint marker")? = true;
    }
    let mut parent_joint = vec![None; nodes.nodes.len()];
    // Parent links already reject multiple parents, so each ancestor chain is unique.
    for &node in joint_nodes {
        let parent = nearest_joint_parent(node, &is_joint, nodes)?;
        *slot_mut(&mut parent_joint, node, "joint parent")? = parent;
    }
    Ok(parent_joint)
}

/// Walks one parent chain until it finds the nearest skin joint ancestor.
fn nearest_joint_parent(
    node: usize,
    is_joint: &[bool],
    nodes: &NodeTree<'_>,
) -> Result<Option<usize>, GltfRigValidationError> {
    // Begin with the source node's direct parent and skip non-joint intermediates.
    let mut ancestor = *slot(&nodes.parents, node, "node parent")?;
    // Skip intermediate non-joint nodes because the profile stores skin joints only.
    while let Some(parent) = ancestor {
        if *slot(is_joint, parent, "skin joint marker")? {
            return Ok(Some(parent));
        }
        ancestor = *slot(&nodes.parents, parent, "node parent")?;
    }
    // A missing joint ancestor marks a skeleton root for this profile.
    Ok(None)
}

/// Produces deterministic parent-first bone order and requires one connected root.
fn bone_depth_first_order(
    joint_nodes: &[usize],
    parent_joint: &[Option<usize>],
    node_count: usize,
) -> Result<Vec<usize>, GltfRigValidationError> {
    // Select one skin-joint root before building the ordered child lists.
    let root = unique_bone_root(joint_nodes, parent_joint)?;
    let children = bone_children(joint_nodes, parent_joint, node_count)?;
    // Depth-first traversal preserves parent-first order and source sibling order.
    let order = depth_first_bone_nodes(root, &children, joint_nodes.len())?;
    if order.len() != joint_nodes.len() {
        return Err("the skin joints do not form one connected bone tree".into());
    }
    Ok(order)
}

/// Requires exactly one joint without another joint ancestor.
fn unique_bone_root(
    joint_nodes: &[usize],
    parent_joint: &[Option<usize>],
) -> Result<usize, GltfRigValidationError> {
    let mut roots = Vec::new();
    // Count roots in the already sorted source-node order.
    for &node in joint_nodes {
        if slot(parent_joint, node, "joint parent")?.is_none() {
            roots.push(node);
        }
    }
    if roots.len() != 1 {
        return Err(format!("the rig has {} root bones; it needs one", roots.len()).into());
    }
    // The single root is the first item because source nodes were sorted.
    let root = *roots
        .first()
        .ok_or_else(|| GltfRigValidationError::new("the rig has no root bone"))?;
    Ok(root)
}

/// Builds each joint node's direct skin-joint children in stable node order.
fn bone_children(
    joint_nodes: &[usize],
    parent_joint: &[Option<usize>],
    node_count: usize,
) -> Result<Vec<Vec<usize>>, GltfRigValidationError> {
    let mut children = vec![Vec::new(); node_count];
    // Append children in sorted source-node order for deterministic traversal.
    for &node in joint_nodes {
        if let Some(parent) = *slot(parent_joint, node, "joint parent")? {
            slot_mut(&mut children, parent, "joint children")?.push(node);
        }
    }
    // Keep every node slot so indexed traversal can use checked access.
    Ok(children)
}

/// Walks the joint tree depth-first while preserving each parent's child order.
fn depth_first_bone_nodes(
    root: usize,
    children: &[Vec<usize>],
    joint_count: usize,
) -> Result<Vec<usize>, GltfRigValidationError> {
    // Reverse-push sorted children so popping preserves ascending node order.
    let mut order = Vec::with_capacity(joint_count);
    let mut stack = vec![root];
    // Reverse pushes preserve the sorted child order when the stack is popped.
    while let Some(node) = stack.pop() {
        order.push(node);
        stack.extend(
            slot(children, node, "joint children")?
                .iter()
                .rev()
                .copied(),
        );
    }
    Ok(order)
}

/// Validates a joint's name, source length, rest transform, and parent index.
fn read_bone(
    node: usize,
    name: &str,
    parent: Option<usize>,
    nodes: &NodeTree<'_>,
) -> Result<Bone, GltfRigValidationError> {
    // Validate Skein metadata before retaining the source skeleton transform.
    validate_bone_metadata(node, name, nodes)?;
    // Keep the full matrix for capsule conversion and store its scale-free rest pose.
    let (rest_matrix, rest) = bone_rest_pose(node, name, nodes)?;
    Ok(Bone {
        name: name.to_owned(),
        parent,
        rest,
        rest_matrix,
    })
}

/// Requires the positive finite TGF length used to author capsule placement.
fn validate_bone_metadata(
    node: usize,
    name: &str,
    nodes: &NodeTree<'_>,
) -> Result<(), GltfRigValidationError> {
    // Skein capsule authoring uses this length to define each bone's local axis.
    let extras = nodes.extras(node).ok_or_else(|| {
        GltfRigValidationError::new(format!("bone {name}: extras are missing or not an object"))
    })?;
    extras
        .get("tgf_length")
        .and_then(Value::as_f64)
        .map(|length| length as f32)
        .filter(|length| length.is_finite() && *length > 0.0)
        .ok_or_else(|| {
            GltfRigValidationError::new(format!(
                "bone {name}: tgf_length is not positive and finite"
            ))
        })?;
    Ok(())
}

/// Validates a bone's full world matrix and returns its full and rigid rest poses.
fn bone_rest_pose(
    node: usize,
    name: &str,
    nodes: &NodeTree<'_>,
) -> Result<(Mat4, Isometry3d), GltfRigValidationError> {
    // Keep scale in the matrix because capsule endpoints are converted through bone space.
    let rest_matrix = *slot(&nodes.world, node, "bone rest transform")?;
    // Reject singular basis vectors before extracting the scale-free profile pose.
    validate_bone_basis(name, rest_matrix)?;
    let rest = pose_of(rest_matrix).ok_or_else(|| {
        GltfRigValidationError::new(format!(
            "bone {name}: rest transform cannot form an isometry"
        ))
    })?;
    Ok((rest_matrix, rest))
}

/// Rejects a singular linear basis before extracting a scale-free bone pose.
fn validate_bone_basis(name: &str, matrix: Mat4) -> Result<(), GltfRigValidationError> {
    // Translation does not change whether the affine matrix's basis is singular.
    let determinant = matrix
        .x_axis
        .truncate()
        .dot(matrix.y_axis.truncate().cross(matrix.z_axis.truncate()));
    if !determinant.is_finite() || determinant.abs() <= 1.0e-12 {
        return Err(format!("bone {name}: rest transform is singular").into());
    }
    Ok(())
}

/// One imported bone with its source node transform and validated length.
struct Bone {
    /// The source glTF node name.
    name: String,
    /// The parent bone index, or none for the root.
    parent: Option<usize>,
    /// The scale-free rest frame in skeleton space.
    rest: Isometry3d,
    /// The full source rest transform used to return points to bone space.
    rest_matrix: Mat4,
}

/// One imported ragdoll capsule before its body parent is resolved.
struct ImportedBody {
    /// The bone index that owns the capsule node.
    bone: usize,
    /// The body data from the Skein component.
    spec: BodySpec,
    /// The optional Skein joint data from the capsule node.
    joint: Option<RagdollJoint>,
}

/// Imports capsule nodes, orders bodies, and connects nearest body ancestors.
fn read_bodies(
    document: &Value,
    nodes: &NodeTree<'_>,
    bones: &[Bone],
    bone_for_node: &[Option<usize>],
) -> Result<(Vec<BodySpec>, Vec<JointSpec>), GltfRigValidationError> {
    // Import annotations in glTF node order and sort by the validated bone tree.
    let imported = import_body_nodes(document, nodes, bones, bone_for_node)?;
    // Map each bone to its ordered body slot for nearest-ancestor resolution.
    let body_for_bone = body_indices_for_bones(&imported, bones.len())?;
    let (bodies, joints, roots) = assemble_bodies(imported, bones, &body_for_bone)?;
    validate_body_root_count(roots)?;
    Ok((bodies, joints))
}

/// Requires a single ragdoll root after parent-first body assembly.
fn validate_body_root_count(roots: usize) -> Result<(), GltfRigValidationError> {
    if roots != 1 {
        return Err(format!("the ragdoll has {roots} root bodies; it needs one").into());
    }
    Ok(())
}

/// Imports annotated non-bone nodes whose direct parent is a skeleton bone.
fn import_body_nodes(
    document: &Value,
    nodes: &NodeTree<'_>,
    bones: &[Bone],
    bone_for_node: &[Option<usize>],
) -> Result<Vec<ImportedBody>, GltfRigValidationError> {
    let mut imported = Vec::new();
    // Visit source nodes in order and select only direct children of imported bones.
    for node in 0..nodes.nodes.len() {
        // Ignore nodes whose parent is absent or is not a skeleton bone.
        let Some(bone) = attached_body_bone(node, nodes, bone_for_node)? else {
            continue;
        };
        // Bone nodes cannot also own capsule shapes in the profile model.
        if let Some(body) = read_body(document, nodes, node, bone, bones)? {
            imported.push(body);
        }
    }
    // Sort by the already deterministic bone tree before validating uniqueness.
    imported.sort_by_key(|body| body.bone);
    validate_unique_body_bones(&imported, bones)?;
    Ok(imported)
}

/// Resolves the body owner when a non-bone node is directly attached to a bone.
fn attached_body_bone(
    node: usize,
    nodes: &NodeTree<'_>,
    bone_for_node: &[Option<usize>],
) -> Result<Option<usize>, GltfRigValidationError> {
    // A body annotation belongs only to the direct parent bone, not a distant ancestor.
    let parent_node = *slot(&nodes.parents, node, "node parent")?;
    let parent_bone = parent_node
        .map(|parent| slot(bone_for_node, parent, "parent bone mapping").copied())
        .transpose()?
        .flatten();
    // Keep bones exclusively in the skeleton hierarchy used by the profile tree.
    let node_is_bone = slot(bone_for_node, node, "bone mapping")?.is_some();
    Ok((!node_is_bone).then_some(parent_bone).flatten())
}

/// Rejects multiple capsule annotations attached to the same source bone.
fn validate_unique_body_bones(
    imported: &[ImportedBody],
    bones: &[Bone],
) -> Result<(), GltfRigValidationError> {
    // Sorting makes duplicate body annotations adjacent for one bounded scan.
    let duplicate_bone = imported.windows(2).find_map(|pair| match pair {
        [first, second] if first.bone == second.bone => Some(first.bone),
        _ => None,
    });
    if let Some(duplicate_bone) = duplicate_bone {
        // Resolve the source name so authors can locate the duplicate annotation.
        let bone = slot(bones, duplicate_bone, "body bone")?;
        return Err(format!("bone {} has two ragdoll body nodes", bone.name).into());
    }
    Ok(())
}

/// Assigns each source skeleton bone its ordered body index, when it has a body.
fn body_indices_for_bones(
    imported: &[ImportedBody],
    bone_count: usize,
) -> Result<Vec<Option<usize>>, GltfRigValidationError> {
    let mut body_for_bone = vec![None; bone_count];
    // Record each ordered body's slot at its source bone for ancestor lookup.
    for (index, body) in imported.iter().enumerate() {
        *slot_mut(&mut body_for_bone, body.bone, "body index by bone")? = Some(index);
    }
    Ok(body_for_bone)
}

/// Connects parent-first body specs and joints, returning the number of roots.
fn assemble_bodies(
    imported: Vec<ImportedBody>,
    bones: &[Bone],
    body_for_bone: &[Option<usize>],
) -> Result<(Vec<BodySpec>, Vec<JointSpec>, usize), GltfRigValidationError> {
    // Parent-first body order lets each joint resolve its parent body immediately.
    let mut bodies: Vec<BodySpec> = Vec::with_capacity(imported.len());
    let mut joints = Vec::with_capacity(imported.len().saturating_sub(1));
    let mut roots = 0;
    // The body index also becomes the child index in the checked profile domain.
    for (child, body) in imported.into_iter().enumerate() {
        let parent = nearest_body_parent(body.bone, bones, body_for_bone)?;
        if parent.is_none() {
            roots += 1;
        }
        if let Some(joint) = imported_joint(child, &body, parent, bones, &bodies)? {
            joints.push(joint);
        }
        // Store the body after its parent and optional joint frame validate.
        bodies.push(body.spec);
    }
    Ok((bodies, joints, roots))
}

/// Finds the nearest ancestor bone that has a body, preserving skeleton ancestry.
fn nearest_body_parent(
    body_bone: usize,
    bones: &[Bone],
    body_for_bone: &[Option<usize>],
) -> Result<Option<usize>, GltfRigValidationError> {
    // Begin above the capsule's owning bone so its parent body cannot resolve itself.
    let mut ancestor = slot(bones, body_bone, "body bone")?.parent;
    while let Some(index) = ancestor {
        if let Some(body) = *slot(body_for_bone, index, "body index by bone")? {
            return Ok(Some(body));
        }
        ancestor = slot(bones, index, "ancestor bone")?.parent;
    }
    // No body ancestor means this body contributes a root candidate.
    Ok(None)
}

/// Validates root and joint annotations, then derives one body-local joint frame.
fn imported_joint(
    child: usize,
    body: &ImportedBody,
    parent: Option<usize>,
    bones: &[Bone],
    bodies: &[BodySpec],
) -> Result<Option<JointSpec>, GltfRigValidationError> {
    let bone = slot(bones, body.bone, "body bone")?;
    // Root bodies have no joint parent and must not carry joint metadata.
    let Some(parent) = parent else {
        validate_root_joint_annotation(body, bone)?;
        return Ok(None);
    };
    // Every non-root body needs joint metadata before a frame can be derived.
    let joint = body.joint.ok_or_else(|| {
        GltfRigValidationError::new(format!("body on bone {} has no RagdollJoint", bone.name))
    })?;
    make_joint_spec(child, parent, body, joint, bodies).map(Some)
}

/// Rejects joint metadata on the unique root body.
fn validate_root_joint_annotation(
    body: &ImportedBody,
    bone: &Bone,
) -> Result<(), GltfRigValidationError> {
    if body.joint.is_some() {
        return Err(format!("root body on bone {} has a RagdollJoint", bone.name).into());
    }
    Ok(())
}

/// Creates a checked joint description between ordered parent and child bodies.
fn make_joint_spec(
    child: usize,
    parent: usize,
    body: &ImportedBody,
    joint: RagdollJoint,
    bodies: &[BodySpec],
) -> Result<JointSpec, GltfRigValidationError> {
    // The importer preserves parent-first order and uses profile-sized indices.
    let child_index = u8::try_from(child).map_err(|_conversion_error| {
        GltfRigValidationError::new(format!("body child index exceeds 255 (value {child})"))
    })?;
    let parent_index = u8::try_from(parent).map_err(|_conversion_error| {
        GltfRigValidationError::new(format!("body parent index exceeds 255 (value {parent})"))
    })?;
    // Express the joint frame relative to the selected parent body's rest frame.
    let parent_rest = slot(bodies, parent, "parent body")?.rest;
    let frame = parent_rest.inverse_mul(body.spec.rest);
    Ok(JointSpec {
        child: child_index,
        parent: parent_index,
        frame,
        limits: JointLimits {
            x: radians(joint.limit_x),
            twist: radians(joint.limit_y),
            z: radians(joint.limit_z),
        },
        max_torque: joint.torque_nm,
    })
}

/// Imports one Skein body node and derives its capsule from mesh bounds.
fn read_body(
    document: &Value,
    nodes: &NodeTree<'_>,
    node: usize,
    bone_index: usize,
    bones: &[Bone],
) -> Result<Option<ImportedBody>, GltfRigValidationError> {
    let node_name = nodes.name(node).unwrap_or("<unnamed>");
    // Validate both reflected components before using any mesh geometry.
    let Some((body, joint)) = read_body_components(node_name, nodes, node)? else {
        return Ok(None);
    };
    let bone = slot(bones, bone_index, "body bone")?;
    // Mesh bounds define a capsule in world space, which is converted into bone space.
    let shape = body_shape_from_mesh(document, nodes, node, node_name, bone.rest_matrix)?;
    Ok(Some(ImportedBody {
        bone: bone_index,
        spec: BodySpec {
            bone: bone.name.clone(),
            shape,
            mass: body.mass_kg,
            rest: bone.rest,
        },
        joint,
    }))
}

/// Reads one body's mesh bounds and converts its capsule into the owning bone frame.
fn body_shape_from_mesh(
    document: &Value,
    nodes: &NodeTree<'_>,
    node: usize,
    node_name: &str,
    bone_matrix: Mat4,
) -> Result<ShapeSpec, GltfRigValidationError> {
    let source_node = slot(nodes.nodes, node, "body node")?;
    let (minimum, maximum) = mesh_bounds(document, source_node)
        .map_err(|problem| GltfRigValidationError::new(format!("node {node_name}: {problem}")))?;
    let world = *slot(&nodes.world, node, "body world transform")?;
    capsule_shape(node_name, minimum, maximum, world, bone_matrix)
}

/// Reads and validates the body and optional joint annotations from a capsule node.
fn read_body_components(
    node_name: &str,
    nodes: &NodeTree<'_>,
    node: usize,
) -> Result<Option<(RagdollBody, Option<RagdollJoint>)>, GltfRigValidationError> {
    // Parse current and legacy paths with the same duplicate and shape rules.
    let components = skein_components(nodes, node)?;
    let body: Option<RagdollBody> =
        component(&components, [NEW_BODY_PATH, OLD_BODY_PATH], "RagdollBody")?;
    let joint: Option<RagdollJoint> = component(
        &components,
        [NEW_JOINT_PATH, OLD_JOINT_PATH],
        "RagdollJoint",
    )?;
    validate_body_annotation_pair(node_name, body, joint)
}

/// Validates body and joint annotations together, including the root-body rule.
fn validate_body_annotation_pair(
    node_name: &str,
    body: Option<RagdollBody>,
    joint: Option<RagdollJoint>,
) -> Result<Option<(RagdollBody, Option<RagdollJoint>)>, GltfRigValidationError> {
    let Some(body) = body else {
        // A joint annotation without its owning body cannot define a profile link.
        if joint.is_some() {
            return Err(format!("node {node_name}: RagdollJoint has no RagdollBody").into());
        }
        return Ok(None);
    };
    // Reject invalid authoring values before interpreting collision geometry.
    if let Some(problem) = body.problem() {
        return Err(format!("node {node_name}: {problem}").into());
    }
    if let Some(problem) = joint.as_ref().and_then(RagdollJoint::problem) {
        return Err(format!("node {node_name}: {problem}").into());
    }
    // Return only annotations that passed the shared Skein authoring rules.
    Ok(Some((body, joint)))
}

/// Derives a round capsule from mesh bounds and returns its endpoints in bone space.
fn capsule_shape(
    node_name: &str,
    minimum: Vec3,
    maximum: Vec3,
    world: Mat4,
    bone_matrix: Mat4,
) -> Result<ShapeSpec, GltfRigValidationError> {
    // Transform the local AABB's three half axes into world-space extents.
    let local_center = (minimum + maximum) * 0.5;
    let local_half_extents = (maximum - minimum) * 0.5;
    let axes = [
        world.x_axis.truncate() * local_half_extents.x,
        world.y_axis.truncate() * local_half_extents.y,
        world.z_axis.truncate() * local_half_extents.z,
    ];
    let lengths = axes.map(Vec3::length);
    // Select the long axis and validate the two remaining half extents as a radius.
    let (axis, length, radius) = capsule_dimensions(node_name, axes, lengths)?;
    // Orient endpoints toward the bone head before returning them to bone space.
    let center = world.transform_point3(local_center);
    let half_axis = axis.normalize() * (length - radius);
    let (near, far) = capsule_ends_toward_bone(center, half_axis, bone_matrix);
    let inverse_bone = bone_matrix.inverse();
    let a = inverse_bone.transform_point3(near);
    let b = inverse_bone.transform_point3(far);
    if !a.is_finite() || !b.is_finite() || !center.is_finite() {
        return Err(format!("node {node_name}: capsule transform is not finite").into());
    }
    Ok(ShapeSpec::Capsule { a, b, radius })
}

/// Resolves the stable longest-axis choice, its length, and validated capsule radius.
fn capsule_dimensions(
    node_name: &str,
    axes: [Vec3; 3],
    lengths: [f32; 3],
) -> Result<(Vec3, f32, f32), GltfRigValidationError> {
    // Ties select the later axis, matching TGF's capsule endpoint orientation.
    let longest = longest_capsule_axis(lengths);
    let axis = axes
        .iter()
        .copied()
        .nth(longest)
        .ok_or_else(|| GltfRigValidationError::new("capsule axis index is invalid"))?;
    let length = lengths
        .iter()
        .copied()
        .nth(longest)
        .ok_or_else(|| GltfRigValidationError::new("capsule length index is invalid"))?;
    // A capsule's two transverse extents must agree within the importer tolerance.
    let radius = capsule_radius(node_name, &lengths, longest)?;
    Ok((axis, length, radius))
}

/// Selects the longest capsule axis with stable last-axis tie breaking.
fn longest_capsule_axis(lengths: [f32; 3]) -> usize {
    let [first, second, third] = lengths;
    // The fixed three-axis input makes X the deterministic initial candidate.
    // Ignore axes shorter than X while folding remaining candidates in XYZ order.
    [(1, second), (2, third)]
        .into_iter()
        .filter(|(_, length)| *length >= first)
        .fold((0, first), |longest, candidate| {
            if candidate.1 >= longest.1 {
                candidate
            } else {
                longest
            }
        })
        .0
}

/// Validates transverse extents and returns their larger value as the capsule radius.
fn capsule_radius(
    node_name: &str,
    lengths: &[f32; 3],
    longest: usize,
) -> Result<f32, GltfRigValidationError> {
    // Exclude the long axis and preserve the original X, Y, Z transverse order.
    let mut transverse = lengths
        .iter()
        .copied()
        .enumerate()
        .filter(|(axis, _)| *axis != longest)
        .map(|(_, length)| length);
    let first = transverse
        .next()
        .ok_or_else(|| GltfRigValidationError::new("capsule has fewer than two transverse axes"))?;
    let second = transverse
        .next()
        .ok_or_else(|| GltfRigValidationError::new("capsule has fewer than two transverse axes"))?;
    // Radius is the larger transverse half extent; both must remain finite and round.
    let radius = first.max(second);
    if !radius.is_finite() || radius <= 0.0 || (first - second).abs() > radius * ROUND_TOLERANCE {
        return Err(format!("node {node_name}: mesh bounds are not a round capsule").into());
    }
    Ok(radius)
}

/// Orders the segment endpoints from the nearest bone-head end to the far end.
fn capsule_ends_toward_bone(center: Vec3, half_axis: Vec3, bone_matrix: Mat4) -> (Vec3, Vec3) {
    // Compare the two cap centres with the owning bone's skeleton-space origin.
    let first = center - half_axis;
    let second = center + half_axis;
    let bone_head = bone_matrix.w_axis.truncate();
    // Keep the nearer endpoint first to match body-local capsule authoring.
    if first.distance(bone_head) <= second.distance(bone_head) {
        (first, second)
    } else {
        (second, first)
    }
}

/// Extracts one-component Skein entries from a node's extras object.
fn skein_components<'a>(
    nodes: &NodeTree<'a>,
    node: usize,
) -> Result<Vec<(&'a str, &'a Value)>, GltfRigValidationError> {
    // Missing extras are allowed; a present extras value must be an object.
    let Some(entries) = skein_entries(nodes, node)? else {
        return Ok(Vec::new());
    };
    // Each authoring entry has one reflected type path and one serialized value.
    entries.iter().map(skein_component_entry).collect()
}

/// Reads a node's optional Skein entry array after validating its containing objects.
fn skein_entries<'a>(
    nodes: &NodeTree<'a>,
    node: usize,
) -> Result<Option<&'a [Value]>, GltfRigValidationError> {
    // Validate the node first because malformed extras differ from absent extras.
    let source_node = slot(nodes.nodes, node, "Skein node")?;
    let Some(extras) = nodes.extras(node) else {
        if source_node.get("extras").is_some() {
            return Err("node extras are not an object".into());
        }
        return Ok(None);
    };
    let Some(skein) = extras.get("skein") else {
        return Ok(None);
    };
    // Only the list form preserves distinct reflected components without key collisions.
    skein
        .as_array()
        .map(Vec::as_slice)
        .map(Some)
        .ok_or_else(|| "the skein extras are not a list".into())
}

/// Reads the single reflected component stored in a Skein list entry.
fn skein_component_entry(entry: &Value) -> Result<(&str, &Value), GltfRigValidationError> {
    // A one-key object is the serialized shape of one reflected component.
    let map = entry
        .as_object()
        .filter(|map| map.len() == 1)
        .ok_or_else(|| GltfRigValidationError::new("a skein entry is not one component"))?;
    let (path, value) = map
        .iter()
        .next()
        .ok_or_else(|| GltfRigValidationError::new("a skein entry is empty"))?;
    Ok((path.as_str(), value))
}

/// Deserializes one current or legacy Skein component and rejects duplicates.
fn component<T: DeserializeOwned>(
    components: &[(&str, &Value)],
    accepted_paths: [&str; 2],
    short_name: &str,
) -> Result<Option<T>, GltfRigValidationError> {
    let mut value = None;
    // Accept only the current crate path or the documented TGF compatibility path.
    for (path, component) in components {
        if accepted_paths.contains(path) {
            if value.is_some() {
                return Err(format!("{short_name} is attached more than once").into());
            }
            value = Some(*component);
        }
    }
    // Deserialize after uniqueness so duplicate paths cannot overwrite each other.
    value
        .map(|component| {
            serde_json::from_value(component.clone())
                .map_err(|error| GltfRigValidationError::new(format!("{short_name}: {error}")))
        })
        .transpose()
}

/// Unions the POSITION accessor bounds for every mesh primitive.
fn mesh_bounds(document: &Value, node: &Value) -> Result<(Vec3, Vec3), GltfRigValidationError> {
    // Resolve the indexed mesh primitives and accessor table before scanning bounds.
    let primitives = body_mesh_primitives(document, node)?;
    let accessors = validate_position_accessor_table(document)?;
    let mut bounds = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
    // Initialize the union as an empty interval that each primitive can extend.
    // Every primitive contributes to one conservative local-space mesh bound.
    for primitive in primitives {
        let (minimum, maximum) = primitive_position_bounds(primitive, accessors)?;
        bounds.0 = bounds.0.min(minimum);
        bounds.1 = bounds.1.max(maximum);
    }
    Ok(bounds)
}

/// Resolves a body node's nonempty mesh primitive list from the glTF document.
fn body_mesh_primitives<'a>(
    document: &'a Value,
    node: &Value,
) -> Result<&'a [Value], GltfRigValidationError> {
    // Resolve the node's mesh index against the document's mesh collection.
    let meshes = document
        .get("meshes")
        .and_then(Value::as_array)
        .ok_or_else(|| GltfRigValidationError::new("the glTF meshes member is not an array"))?;
    let mesh_index = json_index(node.get("mesh"), meshes.len(), "body mesh")?;
    let mesh = meshes
        .get(mesh_index)
        .ok_or_else(|| GltfRigValidationError::new("body mesh index is out of range"))?;
    // Empty primitives cannot define a collision bound for the body.
    mesh.get("primitives")
        .and_then(Value::as_array)
        .filter(|primitives| !primitives.is_empty())
        .map(Vec::as_slice)
        .ok_or_else(|| GltfRigValidationError::new("the body mesh has no primitives"))
}

/// Reads the glTF accessor array used by mesh POSITION attributes.
fn validate_position_accessor_table(document: &Value) -> Result<&[Value], GltfRigValidationError> {
    document
        .get("accessors")
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| GltfRigValidationError::new("the glTF accessors member is not an array"))
}

/// Reads and validates the POSITION bounds for one mesh primitive.
fn primitive_position_bounds(
    primitive: &Value,
    accessors: &[Value],
) -> Result<(Vec3, Vec3), GltfRigValidationError> {
    // Resolve the indexed accessor before interpreting its declared bounds.
    let accessor = position_accessor(primitive, accessors)?;
    // Both endpoints are mandatory because collision bounds cannot infer an extent.
    let minimum = vector3(accessor.get("min")).ok_or_else(|| {
        GltfRigValidationError::new("a POSITION accessor has no finite min bound")
    })?;
    let maximum = vector3(accessor.get("max")).ok_or_else(|| {
        GltfRigValidationError::new("a POSITION accessor has no finite max bound")
    })?;
    // Reject inverted data rather than swapping source-authored extrema.
    if (minimum.cmpgt(maximum)).any() {
        return Err("a POSITION accessor has a reversed bound".into());
    }
    Ok((minimum, maximum))
}

/// Resolves the accessor referenced by one mesh primitive's POSITION attribute.
fn position_accessor<'a>(
    primitive: &Value,
    accessors: &'a [Value],
) -> Result<&'a Value, GltfRigValidationError> {
    // POSITION must reference an accessor in the document-level accessor table.
    let accessor_index = json_index(
        primitive
            .get("attributes")
            .and_then(|attributes| attributes.get("POSITION")),
        accessors.len(),
        "POSITION accessor",
    )?;
    // Keep the checked accessor lookup adjacent to its error context.
    accessors
        .get(accessor_index)
        .ok_or_else(|| GltfRigValidationError::new("POSITION accessor index is out of range"))
}

/// Reads a JSON array index and checks its collection bound.
fn json_index(
    value: Option<&Value>,
    length: usize,
    label: &str,
) -> Result<usize, GltfRigValidationError> {
    let index = value
        .and_then(Value::as_u64)
        .and_then(|index| usize::try_from(index).ok())
        .filter(|index| *index < length)
        .ok_or_else(|| {
            GltfRigValidationError::new(format!("{label} index is missing or out of range"))
        })?;
    Ok(index)
}

/// Reads exactly three finite JSON numbers as a Bevy vector.
fn vector3(value: Option<&Value>) -> Option<Vec3> {
    let values = value?.as_array()?;
    if values.len() != 3 {
        return None;
    }
    // Convert JSON's numeric representation only at the glTF input boundary.
    let mut components = [0.0; 3];
    for (component, value) in components.iter_mut().zip(values) {
        *component = value.as_f64()? as f32;
    }
    // Bounds must stay finite after narrowing JSON numbers to profile precision.
    components
        .iter()
        .all(|component| component.is_finite())
        .then(|| Vec3::from_array(components))
}

/// Converts authoring degrees to the profile's radian range.
fn radians(range: AngleRange) -> crate::profile::AngleRange {
    crate::profile::AngleRange {
        min: range.min_deg.to_radians(),
        max: range.max_deg.to_radians(),
    }
}

/// Returns a scale-free rest isometry from a global node matrix.
fn pose_of(matrix: Mat4) -> Option<Isometry3d> {
    let (_, rotation, translation) = matrix.to_scale_rotation_translation();
    // Reject non-finite and degenerate rotation data before normalization.
    if !translation.is_finite() || !rotation.is_finite() || rotation.length_squared() <= 1.0e-12 {
        return None;
    }
    Some(Isometry3d::new(translation, rotation.normalize()))
}

/// Parses a node's matrix or TRS transform using glTF's column-major layout.
fn local_transform(node: &Value) -> Result<Mat4, GltfRigValidationError> {
    let has_matrix = node.get("matrix").is_some();
    let has_trs = ["translation", "rotation", "scale"]
        .iter()
        .any(|member| node.get(*member).is_some());
    if has_matrix && has_trs {
        return Err("matrix and TRS transform members are both present".into());
    }
    // Matrix and TRS forms have separate validation paths because glTF forbids both.
    match node.get("matrix") {
        Some(matrix) => matrix_transform(matrix),
        None => trs_transform(node),
    }
}

/// Parses and validates the glTF column-major matrix representation.
fn matrix_transform(value: &Value) -> Result<Mat4, GltfRigValidationError> {
    let values = number_array::<16>(value)
        .ok_or_else(|| GltfRigValidationError::new("matrix is not sixteen finite numbers"))?;
    let matrix = Mat4::from_cols_array(&values);
    is_valid_local_matrix(matrix)
        .then_some(matrix)
        .ok_or_else(|| GltfRigValidationError::new("matrix is not a finite affine TRS transform"))
}

/// Parses, normalizes, and validates the glTF translation-rotation-scale representation.
fn trs_transform(node: &Value) -> Result<Mat4, GltfRigValidationError> {
    // Apply glTF identity defaults before validating the supplied quaternion.
    let translation = optional_array::<3>(node, "translation", [0.0; 3])?;
    let rotation = optional_array::<4>(node, "rotation", [0.0, 0.0, 0.0, 1.0])?;
    let scale = optional_array::<3>(node, "scale", [1.0; 3])?;
    let rotation = Quat::from_array(rotation);
    if !is_unit_rotation(rotation) {
        return Err("rotation is not a finite unit quaternion".into());
    }
    // glTF stores quaternions in xyzw order; normalize within the accepted tolerance.
    let matrix = Mat4::from_scale_rotation_translation(
        Vec3::from_array(scale),
        rotation.normalize(),
        Vec3::from_array(translation),
    );
    // Reject overflow introduced by composing otherwise finite TRS components.
    is_finite_matrix(matrix)
        .then_some(matrix)
        .ok_or_else(|| GltfRigValidationError::new("TRS transform is not finite"))
}

/// Reads one optional fixed-size finite number array from a node.
fn optional_array<const N: usize>(
    node: &Value,
    member: &str,
    default: [f32; N],
) -> Result<[f32; N], GltfRigValidationError> {
    // Missing TRS members use glTF's identity defaults; present members must be complete.
    match node.get(member) {
        None => Ok(default),
        Some(value) => {
            // Explicit TRS members must match their fixed glTF cardinality.
            number_array::<N>(value).ok_or_else(|| {
                GltfRigValidationError::new(format!("{member} is not {N} finite numbers"))
            })
        }
    }
}

/// Reads exactly `N` finite JSON numbers and converts them to `f32`.
fn number_array<const N: usize>(value: &Value) -> Option<[f32; N]> {
    let values = value.as_array()?;
    if values.len() != N {
        return None;
    }
    // Convert only after the input cardinality matches the destination array.
    let mut output = [0.0; N];
    for (component, value) in output.iter_mut().zip(values) {
        *component = value.as_f64()? as f32;
    }
    // Conversion can overflow even when the JSON number itself was finite.
    output
        .iter()
        .all(|component| component.is_finite())
        .then_some(output)
}

/// Checks matrix finiteness, affine layout, and the absence of shear.
fn is_valid_local_matrix(matrix: Mat4) -> bool {
    // Reject projective matrices before checking their linear basis vectors.
    if !is_finite_matrix(matrix)
        || matrix.x_axis.w.abs() > 1.0e-5
        || matrix.y_axis.w.abs() > 1.0e-5
        || matrix.z_axis.w.abs() > 1.0e-5
        || (matrix.w_axis.w - 1.0).abs() > 1.0e-5
    {
        return false;
    }
    let axes = [
        matrix.x_axis.truncate(),
        matrix.y_axis.truncate(),
        matrix.z_axis.truncate(),
    ];
    // Every pair of basis vectors must be orthogonal relative to its scale.
    for (index, first) in axes.iter().enumerate() {
        for second in axes.iter().skip(index + 1) {
            let scale = first.length() * second.length();
            if first.dot(*second).abs() > scale * 1.0e-4 {
                return false;
            }
        }
    }
    true
}

/// Returns whether all sixteen matrix components are finite.
fn is_finite_matrix(matrix: Mat4) -> bool {
    matrix
        .to_cols_array()
        .iter()
        .all(|component| component.is_finite())
}

/// Returns whether a quaternion is finite and unit length within float tolerance.
fn is_unit_rotation(rotation: Quat) -> bool {
    let length_squared = rotation.length_squared();
    rotation.is_finite() && length_squared.is_finite() && (length_squared - 1.0).abs() <= 1.0e-4
}

/// Builds a GLB input error from a local diagnostic.
fn invalid(problem: &str) -> GltfRigError {
    GltfRigError::Invalid(GltfRigValidationError::new(problem))
}

#[cfg(test)]
mod tests;
