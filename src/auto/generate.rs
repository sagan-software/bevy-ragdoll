//! The generator pipeline: keep, select, classify, shape, weigh and joint.

use std::f32::consts::PI;

use bevy::math::Vec3;

use super::humanoid::{self, Bones};
use super::{BoneBody, Skeleton};
use crate::profile::{
    AngleRange, BodyRole, BodySpec, JointLimits, JointSpec, MAX_BODIES, ProfileSpec, ShapeSpec,
};

/// Body density in kilograms per cubic metre, close to human tissue.
const DENSITY: f32 = 985.0;
/// Shortest kept segment as a fraction of the skeleton size.
const MIN_SEGMENT: f32 = 0.04;
/// Total mass in kilograms that the torque table was tuned for.
const REFERENCE_MASS: f32 = 80.0;
/// Bone-name fragments of helper bones that never carry bodies.
const HELPER_NAMES: &[&str] = &[
    "ik", "pole", "target", "socket", "attach", "weapon", "prop", "root", "armature",
];
/// Bone-name fragments of bones that merge into their parent body.
const MERGE_NAMES: &[&str] = &["twist", "end", "roll", "helper", "corrective"];

/// Generates unvalidated profile data for `skeleton`.
pub(super) fn generate(skeleton: &Skeleton) -> ProfileSpec {
    let rig = Rig::new(skeleton);
    let bodies = rig.select_bodies();
    if bodies.is_empty() {
        return ProfileSpec::default();
    }
    let tree = BodyTree::new(&rig, bodies);
    let humanoid = rig.humanoid_slots().is_some();
    let roles = rig.roles(&tree);
    let segments = (0..tree.bones.len())
        .map(|body| rig.segment(&tree, &roles, body))
        .collect::<Vec<_>>();
    let mut body_specs = tree
        .bones
        .iter()
        .zip(&roles)
        .zip(&segments)
        .map(|((bone, role), segment)| rig.body_spec(*bone, *role, segment))
        .collect::<Vec<_>>();
    if humanoid {
        rig.distribute_humanoid_mass(&tree, &roles, &mut body_specs);
    }
    rig.normalize_mass(&tree, &mut body_specs);
    let total = body_specs.iter().map(|body| body.mass).sum::<f32>();
    let joints = (1..tree.bones.len())
        .map(|child| rig.joint_spec(&tree, &roles, &segments, child, total))
        .collect();
    ProfileSpec {
        bodies: body_specs,
        joints,
    }
}

/// Lowercase name test for one of the given fragments as a whole token prefix.
fn has_fragment(name: &str, fragments: &[&str]) -> bool {
    let lower = name.to_ascii_lowercase();
    lower
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .any(|token| {
            fragments.iter().any(|fragment| {
                token == *fragment
                    || (token.starts_with(fragment) && *fragment != "end" && fragment.len() > 2)
                    || (*fragment == "end" && token.ends_with("end"))
            })
        })
}

/// Skeleton facts shared by every generator stage.
struct Rig<'a> {
    /// The input skeleton.
    skeleton: &'a Skeleton,
    /// Valid parent index of each bone; out-of-order parents become roots.
    parents: Vec<Option<usize>>,
    /// Child indexes of each bone.
    children: Vec<Vec<usize>>,
    /// Bone head positions in skeleton space.
    heads: Vec<Vec3>,
    /// Whether each bone may carry a body or contribute geometry.
    eligible: Vec<bool>,
    /// Whether each bone's name allows a body, ignoring `Skip` overrides.
    named: Vec<bool>,
    /// Largest extent of the eligible bone heads in metres.
    size: f32,
    /// Lowest eligible bone head height in metres.
    ground: f32,
    /// Vertical extent of the eligible bone heads in metres.
    height: f32,
}

/// A generated body's capsule segment in skeleton space.
struct Segment {
    /// Segment start, at the bone head.
    start: Vec3,
    /// Segment end, at the main child head or an estimated tip.
    end: Vec3,
}

impl Segment {
    /// Unit direction from start to end, or +Y for a zero-length segment.
    fn direction(&self) -> Vec3 {
        (self.end - self.start).try_normalize().unwrap_or(Vec3::Y)
    }
}

/// Selected bodies as bone indexes with their parent relationships.
struct BodyTree {
    /// Bone index of each body in parent-first order.
    bones: Vec<usize>,
    /// Parent body of each body; body zero is the root.
    parents: Vec<Option<usize>>,
    /// Child bodies of each body.
    children: Vec<Vec<usize>>,
    /// Body that owns each bone's geometry, if any.
    owner: Vec<Option<usize>>,
}

impl<'a> Rig<'a> {
    /// Computes parents, children, eligibility and skeleton size.
    fn new(skeleton: &'a Skeleton) -> Self {
        let count = skeleton.bones.len();
        let parents = skeleton
            .bones
            .iter()
            .enumerate()
            .map(|(index, bone)| bone.parent.filter(|parent| *parent < index))
            .collect::<Vec<_>>();
        let mut children = vec![Vec::new(); count];
        for (index, parent) in parents.iter().enumerate() {
            if let Some(parent) = parent {
                children[*parent].push(index);
            }
        }
        // `Skip` removes a whole subtree; helper names only remove the bone itself.
        let mut skipped = vec![false; count];
        let mut eligible = vec![false; count];
        let mut named = vec![false; count];
        for (index, bone) in skeleton.bones.iter().enumerate() {
            skipped[index] = bone.overrides.body == BoneBody::Skip
                || parents[index].is_some_and(|parent| skipped[parent]);
            named[index] = bone.overrides.body == BoneBody::Body
                || !has_fragment(&bone.name, HELPER_NAMES);
            eligible[index] = !skipped[index] && named[index];
        }
        let heads = skeleton
            .bones
            .iter()
            .map(|bone| Vec3::from(bone.rest.translation))
            .collect::<Vec<_>>();
        let (min, max) = heads
            .iter()
            .zip(&eligible)
            .filter(|(_, eligible)| **eligible)
            .fold((Vec3::MAX, Vec3::MIN), |(min, max), (head, _)| {
                (min.min(*head), max.max(*head))
            });
        let extent = (max - min).max(Vec3::ZERO);
        let size = if extent.max_element() > 1.0e-3 {
            extent.max_element()
        } else {
            1.0
        };
        Self {
            skeleton,
            parents,
            children,
            heads,
            eligible,
            named,
            size,
            ground: min.y,
            height: if extent.y > 1.0e-3 { extent.y } else { size },
        }
    }

    /// Returns the nearest eligible ancestor of `bone`.
    fn eligible_parent(&self, bone: usize) -> Option<usize> {
        let mut current = self.parents[bone];
        while let Some(parent) = current {
            if self.eligible[parent] {
                return Some(parent);
            }
            current = self.parents[parent];
        }
        None
    }

    /// Chooses body bones from the humanoid preset or from segment lengths.
    fn select_bodies(&self) -> Vec<usize> {
        let names = self
            .skeleton
            .bones
            .iter()
            .map(|bone| bone.name.as_str())
            .collect::<Vec<_>>();
        let forced = |bone: usize| self.skeleton.bones[bone].overrides.body == BoneBody::Body;
        let bones = Bones {
            names: &names,
            parents: &self.parents,
            eligible: &self.named,
        };
        if let Some(slots) = humanoid::detect(&bones) {
            // Skipped slots lose their body but keep the humanoid layout.
            let mut selected = slots
                .iter()
                .map(|(bone, _)| *bone)
                .filter(|bone| self.eligible[*bone])
                .collect::<Vec<_>>();
            selected.extend((0..names.len()).filter(|bone| forced(*bone) && self.eligible[*bone]));
            selected.sort_unstable();
            selected.dedup();
            return selected;
        }
        // Raise the minimum segment length until the body count fits the masks.
        let mut min_length = MIN_SEGMENT * self.size;
        loop {
            let selected = (0..names.len())
                .filter(|bone| self.eligible[*bone])
                .filter(|bone| forced(*bone) || self.keeps(*bone, min_length))
                .collect::<Vec<_>>();
            if selected.len() <= MAX_BODIES {
                return selected;
            }
            min_length *= 1.25;
        }
    }

    /// Returns whether an automatic bone is long enough to carry a body.
    fn keeps(&self, bone: usize, min_length: f32) -> bool {
        let overrides = &self.skeleton.bones[bone].overrides;
        if overrides.body == BoneBody::Merge || has_fragment(&self.skeleton.bones[bone].name, MERGE_NAMES) {
            return false;
        }
        let Some(parent) = self.eligible_parent(bone) else {
            return true;
        };
        let head = self.heads[bone];
        let length = self.children[bone]
            .iter()
            .filter(|child| self.eligible[**child])
            .map(|child| self.heads[*child].distance(head))
            .fold(None, |longest: Option<f32>, length| {
                Some(longest.map_or(length, |longest| longest.max(length)))
            })
            // A leaf has no known length, so its parent link stands in for it.
            .unwrap_or_else(|| self.heads[parent].distance(head));
        length >= min_length
    }

    /// Assigns a role to every body, from the humanoid preset or topology.
    fn roles(&self, tree: &BodyTree) -> Vec<BodyRole> {
        let names = self
            .skeleton
            .bones
            .iter()
            .map(|bone| bone.name.as_str())
            .collect::<Vec<_>>();
        let bones = Bones {
            names: &names,
            parents: &self.parents,
            eligible: &self.named,
        };
        let mut roles = match humanoid::detect(&bones) {
            Some(slots) => tree
                .bones
                .iter()
                .map(|bone| {
                    slots
                        .iter()
                        .find(|(slot, _)| slot == bone)
                        .map_or(BodyRole::Other, |(_, role)| *role)
                })
                .collect(),
            None => self.topology_roles(tree),
        };
        for (role, bone) in roles.iter_mut().zip(&tree.bones) {
            if let Some(explicit) = self.skeleton.bones[*bone].overrides.role {
                *role = explicit;
            }
        }
        roles
    }

    /// Classifies bodies by tree shape and rest geometry.
    ///
    /// The root is the core. The spine follows the largest child subtree until
    /// it branches. Chains leaving the core or spine are support limbs when
    /// they reach the ground, a tail when they point against the spine, the
    /// head when they continue the spine, and reach limbs otherwise.
    fn topology_roles(&self, tree: &BodyTree) -> Vec<BodyRole> {
        let count = tree.bones.len();
        let mut roles = vec![BodyRole::Other; count];
        roles[0] = BodyRole::Pelvis;
        let mut sizes = vec![1_usize; count];
        for body in (1..count).rev() {
            if let Some(parent) = tree.parents[body] {
                sizes[parent] += sizes[body];
            }
        }
        // Follow a unique largest subtree from the core, then single children.
        let mut spine = Vec::new();
        let mut current = 0;
        loop {
            let children = &tree.children[current];
            let next = if current == 0 {
                let largest = children.iter().map(|child| sizes[*child]).max();
                let mut best = children
                    .iter()
                    .filter(|child| Some(sizes[**child]) == largest);
                match (best.next(), best.next()) {
                    (Some(child), None) => Some(*child),
                    _ => None,
                }
            } else if children.len() == 1 {
                children.first().copied()
            } else {
                None
            };
            let Some(next) = next else { break };
            spine.push(next);
            current = next;
        }
        for body in &spine {
            roles[*body] = BodyRole::Spine;
        }
        let spine_end = spine.last().copied().unwrap_or(0);
        if spine_end != 0 && !tree.children[spine_end].is_empty() {
            roles[spine_end] = BodyRole::Chest;
        }
        let spine_dir = (self.heads[tree.bones[spine_end]] - self.heads[tree.bones[0]])
            .try_normalize()
            .unwrap_or(Vec3::Y);
        // Collect chains that start at the core or a spine body.
        let mut chains = Vec::new();
        for base in std::iter::once(0).chain(spine.iter().copied()) {
            for start in &tree.children[base] {
                if !spine.contains(start) {
                    chains.push((base, *start));
                }
            }
        }
        let mut head_choice: Option<(f32, usize)> = None;
        let mut kinds = Vec::with_capacity(chains.len());
        for (index, (base, start)) in chains.iter().enumerate() {
            let members = tree.subtree(*start);
            let lowest = members
                .iter()
                .flat_map(|(body, _)| tree.owned_bones(*body))
                .map(|bone| self.heads[bone].y)
                .fold(f32::MAX, f32::min);
            let tip = members
                .iter()
                .max_by_key(|(_, depth)| *depth)
                .map_or(*start, |(body, _)| *body);
            let direction = (self.heads[tree.bones[tip]] - self.heads[tree.bones[*base]])
                .try_normalize()
                .unwrap_or(Vec3::NEG_Y);
            let kind = if lowest <= self.ground + 0.15 * self.height {
                ChainKind::Support
            } else if direction.dot(-spine_dir) > 0.5 {
                ChainKind::Tail
            } else {
                let alignment = direction.dot(spine_dir);
                if *base == spine_end
                    && alignment > 0.3
                    && head_choice.is_none_or(|(best, _)| alignment > best)
                {
                    head_choice = Some((alignment, index));
                }
                ChainKind::Reach
            };
            kinds.push(kind);
        }
        if let Some((_, index)) = head_choice {
            kinds[index] = ChainKind::Head;
        }
        for ((_, start), kind) in chains.iter().zip(kinds) {
            let members = tree.subtree(*start);
            let last = members.iter().map(|(_, depth)| *depth).max().unwrap_or(0);
            for (body, depth) in members {
                roles[body] = kind.role(depth, last);
            }
        }
        roles
    }

    /// Computes a body's capsule segment in skeleton space.
    fn segment(&self, tree: &BodyTree, roles: &[BodyRole], body: usize) -> Segment {
        let bone = tree.bones[body];
        let start = self.heads[bone];
        let incoming = tree.parents[body].map(|parent| start - self.heads[tree.bones[parent]]);
        let children = &tree.children[body];
        // Prefer the child body that continues the incoming direction.
        let main_child = match incoming.and_then(Vec3::try_normalize) {
            Some(direction) => children.iter().copied().max_by(|a, b| {
                let score = |child: usize| {
                    (self.heads[tree.bones[child]] - start)
                        .try_normalize()
                        .map_or(-2.0, |toward| toward.dot(direction))
                };
                score(*a).total_cmp(&score(*b))
            }),
            None => children
                .iter()
                .copied()
                .max_by_key(|child| tree.subtree(*child).len()),
        };
        if let Some(child) = main_child {
            // End at the first bone toward the main child, so a merged neck
            // ends the chest at the neck instead of the head.
            let mut first = tree.bones[child];
            while let Some(parent) = self.parents[first].filter(|parent| *parent != bone) {
                first = parent;
            }
            return Segment {
                start,
                end: self.heads[first],
            };
        }
        // A leaf ends at its farthest merged bone, else at an estimated tip.
        let farthest = tree
            .owned_bones(body)
            .map(|owned| self.heads[owned])
            .max_by(|a, b| a.distance(start).total_cmp(&b.distance(start)))
            .filter(|tip| tip.distance(start) > 0.02 * self.size);
        let end = farthest.unwrap_or_else(|| {
            let direction = incoming
                .and_then(Vec3::try_normalize)
                .unwrap_or(Vec3::Y);
            let length = match roles[body] {
                BodyRole::Head => 0.077 * self.size,
                BodyRole::Hand => 0.05 * self.size,
                _ => incoming.map_or(0.1 * self.size, |link| 0.6 * link.length()),
            };
            start + direction * length.max(0.02 * self.size)
        });
        // The skull starts above the head joint, where the neck ends.
        let start = if roles[body] == BodyRole::Head && tree.children[body].is_empty() {
            start + (end - start).normalize_or_zero() * 0.019 * self.size
        } else {
            start
        };
        Segment { start, end }
    }

    /// Builds a body's spec: a capsule inside its segment and its volume mass.
    fn body_spec(&self, bone: usize, role: BodyRole, segment: &Segment) -> BodySpec {
        let source = &self.skeleton.bones[bone];
        let length = segment.start.distance(segment.end);
        let radius = source
            .overrides
            .radius
            .unwrap_or_else(|| default_radius(role, length, self.size))
            .max(0.005 * self.size);
        let (a, b) = (segment.start, segment.end);
        let volume = PI * radius * radius * (length + 4.0 / 3.0 * radius);
        let local = source.rest.inverse();
        BodySpec {
            bone: source.name.clone(),
            shape: ShapeSpec::Capsule {
                a: local.transform_point(a).into(),
                b: local.transform_point(b).into(),
                radius,
            },
            mass: source.overrides.mass.unwrap_or(volume * DENSITY),
            rest: source.rest,
            role: Some(role),
        }
    }

    /// Returns the humanoid body slots when bone names match a convention.
    fn humanoid_slots(&self) -> Option<Vec<(usize, BodyRole)>> {
        let names = self
            .skeleton
            .bones
            .iter()
            .map(|bone| bone.name.as_str())
            .collect::<Vec<_>>();
        humanoid::detect(&Bones {
            names: &names,
            parents: &self.parents,
            eligible: &self.named,
        })
    }

    /// Redistributes automatic masses by the humanoid segment-mass table.
    ///
    /// The total stays the volume-derived total; overridden masses are kept.
    fn distribute_humanoid_mass(&self, tree: &BodyTree, roles: &[BodyRole], bodies: &mut [BodySpec]) {
        let explicit = |body: usize| self.skeleton.bones[tree.bones[body]].overrides.mass.is_some();
        let free = (0..bodies.len()).filter(|body| !explicit(*body));
        let (volume, share) = free.fold((0.0, 0.0), |(volume, share), body| {
            (volume + bodies[body].mass, share + mass_share(roles[body]))
        });
        for (body, spec) in bodies.iter_mut().enumerate() {
            if !explicit(body) {
                spec.mass = volume * mass_share(roles[body]) / share;
            }
        }
    }

    /// Scales automatic masses so the total equals the requested mass.
    fn normalize_mass(&self, tree: &BodyTree, bodies: &mut [BodySpec]) {
        let Some(total) = self.skeleton.mass else {
            return;
        };
        let explicit = |body: usize| self.skeleton.bones[tree.bones[body]].overrides.mass;
        let fixed = (0..bodies.len()).filter_map(explicit).sum::<f32>();
        let free = (0..bodies.len())
            .filter(|body| explicit(*body).is_none())
            .map(|body| bodies[body].mass)
            .sum::<f32>();
        let scale = (total.kilograms() - fixed) / free;
        if !(scale.is_finite() && scale > 0.0) {
            return;
        }
        for (index, body) in bodies.iter_mut().enumerate() {
            if explicit(index).is_none() {
                body.mass *= scale;
            }
        }
    }

    /// Builds the joint between `child` and its parent body.
    fn joint_spec(
        &self,
        tree: &BodyTree,
        roles: &[BodyRole],
        segments: &[Segment],
        child: usize,
        total_mass: f32,
    ) -> JointSpec {
        let parent = tree.parents[child].unwrap_or(0);
        let source = &self.skeleton.bones[tree.bones[child]];
        let parent_rest = self.skeleton.bones[tree.bones[parent]].rest;
        let role = roles[child];
        let template = Template::of(role);
        let limits = source.overrides.limits.unwrap_or_else(|| {
            let axis = self.flex_axis(role, &segments[parent], &segments[child]);
            let away = self.abduction(role, &segments[child]);
            let along = segments[child].direction();
            // Rotating about local `axis` moves the tip toward `axis × along`.
            template.limits(source.rest.rotation.inverse() * axis, |side_x| {
                let local = if side_x { Vec3::X } else { Vec3::Z };
                (source.rest.rotation * local).cross(along).dot(away)
            })
        });
        JointSpec {
            child: child as u8,
            parent: parent as u8,
            frame: parent_rest.inverse() * source.rest,
            limits,
            max_torque: source
                .overrides
                .max_torque
                .unwrap_or(template.torque * total_mass / REFERENCE_MASS),
        }
    }

    /// Direction that moves a limb away from the body: arms up, legs outward.
    fn abduction(&self, role: BodyRole, child: &Segment) -> Vec3 {
        match role {
            BodyRole::UpperArm => Vec3::Y,
            BodyRole::Thigh => Vec3::X * (child.start.x - self.heads[0].x).signum(),
            _ => Vec3::ZERO,
        }
    }

    /// Returns the skeleton-space axis about which positive rotation flexes.
    ///
    /// A bent hinge keeps its rest bend direction. Otherwise the child tip
    /// flexes toward the role's default direction: knees backward (-Z), feet
    /// upward, hands downward and everything else forward (+Z, the glTF front).
    fn flex_axis(&self, role: BodyRole, parent: &Segment, child: &Segment) -> Vec3 {
        let along = child.direction();
        let hinge = matches!(role, BodyRole::LowerArm | BodyRole::Calf);
        let bent = parent.direction().cross(along);
        if hinge && bent.length() > 0.25 {
            return bent.normalize();
        }
        let toward = match role {
            BodyRole::Calf => Vec3::NEG_Z,
            BodyRole::Foot => Vec3::Y,
            BodyRole::Hand => Vec3::NEG_Y,
            _ => Vec3::Z,
        };
        [toward, Vec3::Y, Vec3::Z]
            .into_iter()
            .find_map(|toward| along.cross(toward).try_normalize())
            .unwrap_or(Vec3::X)
    }
}

/// Default capsule radius in metres for a role.
///
/// Torso radii scale with the skeleton size and limb radii with segment length.
fn default_radius(role: BodyRole, length: f32, size: f32) -> f32 {
    match role {
        BodyRole::Pelvis | BodyRole::Spine => 0.077 * size,
        BodyRole::Chest => 0.09 * size,
        BodyRole::Head => 0.064 * size,
        BodyRole::Neck => 0.035 * size,
        BodyRole::Thigh => 0.19 * length,
        BodyRole::Calf => 0.1425 * length,
        BodyRole::Foot => 0.304 * length,
        BodyRole::UpperArm => 0.1786 * length,
        BodyRole::LowerArm => 0.173 * length,
        BodyRole::Hand => 0.5 * length,
        BodyRole::Tail | BodyRole::Other => 0.25 * length,
    }
}

/// Share of total mass for a humanoid body role, from an 80 kg reference.
fn mass_share(role: BodyRole) -> f32 {
    match role {
        BodyRole::Pelvis => 8.94,
        BodyRole::Spine => 13.06,
        BodyRole::Chest => 12.77,
        BodyRole::Neck => 1.5,
        BodyRole::Head => 5.55,
        BodyRole::UpperArm => 2.17,
        BodyRole::LowerArm => 1.3,
        BodyRole::Hand => 0.49,
        BodyRole::Thigh => 11.33,
        BodyRole::Calf => 3.46,
        BodyRole::Foot => 1.1,
        BodyRole::Tail | BodyRole::Other => 1.0,
    }
}

/// Topology class of a chain leaving the core or spine.
#[derive(Clone, Copy)]
enum ChainKind {
    /// A limb that reaches the ground.
    Support,
    /// A limb that does not reach the ground.
    Reach,
    /// The neck and head continuing the spine.
    Head,
    /// A chain pointing against the spine.
    Tail,
}

impl ChainKind {
    /// Role of the body at `depth` in a chain whose deepest body is at `last`.
    const fn role(self, depth: usize, last: usize) -> BodyRole {
        let distal = depth == last && last >= 2;
        match self {
            Self::Support if depth == 0 => BodyRole::Thigh,
            Self::Support if distal => BodyRole::Foot,
            Self::Support => BodyRole::Calf,
            Self::Reach if depth == 0 => BodyRole::UpperArm,
            Self::Reach if distal => BodyRole::Hand,
            Self::Reach => BodyRole::LowerArm,
            Self::Head if depth == last => BodyRole::Head,
            Self::Head => BodyRole::Neck,
            Self::Tail => BodyRole::Tail,
        }
    }
}

/// Joint limit and torque template for a child role, in radians and N·m.
struct Template {
    /// Extension: allowed rotation against the flex direction.
    extend: f32,
    /// Flexion: allowed rotation in the flex direction.
    flex: f32,
    /// Twist about the bone axis in both directions.
    twist: f32,
    /// Side bend toward the abduction direction (away from the body).
    abduct: f32,
    /// Side bend against the abduction direction.
    adduct: f32,
    /// Maximum motor torque at the reference mass.
    torque: f32,
}

impl Template {
    /// Returns the template for a child body role.
    #[allow(
        clippy::approx_constant,
        reason = "the table lists rounded tuned radians, not mathematical constants"
    )]
    const fn of(role: BodyRole) -> Self {
        // Radians rounded to six decimals. The Rapier conformance scenes are
        // chaotic: exact degree conversions change their outcomes.
        let (extend, flex, twist, abduct, adduct, torque) = match role {
            BodyRole::Spine => (0.349_066, 0.610_865, 0.349_066, 0.349_066, 0.349_066, 200.0),
            BodyRole::Chest => (0.349_066, 0.610_865, 0.436_332, 0.349_066, 0.349_066, 150.0),
            BodyRole::Neck | BodyRole::Head => {
                (0.785_398, 0.872_665, 1.047_198, 0.610_865, 0.610_865, 30.0)
            }
            BodyRole::UpperArm => (0.523_599, 1.570_796, 0.785_398, 1.308_997, 0.785_398, 40.0),
            BodyRole::LowerArm => (0.0, 2.443_461, 0.0, 0.0, 0.0, 25.0),
            BodyRole::Hand => (1.047_198, 1.047_198, 0.0, 0.0, 0.0, 8.0),
            BodyRole::Thigh => (0.349_066, 1.919_862, 0.523_599, 0.785_398, 0.436_332, 150.0),
            BodyRole::Calf => (0.0, 2.443_461, 0.0, 0.0, 0.0, 80.0),
            BodyRole::Foot => (0.785_398, 0.349_066, 0.0, 0.0, 0.0, 25.0),
            BodyRole::Tail => (0.5, 0.5, 0.2, 0.5, 0.5, 20.0),
            BodyRole::Pelvis | BodyRole::Other => (0.5, 0.5, 0.3, 0.5, 0.5, 20.0),
        };
        Self {
            extend,
            flex,
            twist,
            abduct,
            adduct,
            torque,
        }
    }

    /// Places the flex range on the child-local X or Z axis nearest `axis`.
    ///
    /// `abduct_sign` is the sign of rotation about the other bend axis that
    /// moves the child away from the body.
    fn limits(&self, axis: Vec3, abduct_sign: impl Fn(bool) -> f32) -> JointLimits {
        let along_x = axis.x.abs() >= axis.z.abs();
        let sign = if along_x { axis.x } else { axis.z };
        let flex = if sign >= 0.0 {
            AngleRange {
                min: -self.extend,
                max: self.flex,
            }
        } else {
            AngleRange {
                min: -self.flex,
                max: self.extend,
            }
        };
        let side = if abduct_sign(!along_x) >= 0.0 {
            AngleRange {
                min: -self.adduct,
                max: self.abduct,
            }
        } else {
            AngleRange {
                min: -self.abduct,
                max: self.adduct,
            }
        };
        let twist = AngleRange {
            min: -self.twist,
            max: self.twist,
        };
        let (x, z) = if along_x { (flex, side) } else { (side, flex) };
        JointLimits { x, twist, z }
    }
}

impl BodyTree {
    /// Links selected bodies to their nearest body ancestors.
    ///
    /// When several bodies have no body ancestor, the root with the most
    /// descendants is kept and the other roots' subtrees are dropped.
    fn new(rig: &Rig<'_>, selected: Vec<usize>) -> Self {
        let count = rig.skeleton.bones.len();
        let mut body_of = vec![None; count];
        let mut owner = vec![None; count];
        for (body, bone) in selected.iter().enumerate() {
            body_of[*bone] = Some(body);
        }
        // Owners follow parent-first order: a bone's owner is itself or its parent's owner.
        for bone in 0..count {
            owner[bone] = body_of[bone].or_else(|| {
                rig.eligible[bone]
                    .then(|| rig.parents[bone].and_then(|parent| owner[parent]))
                    .flatten()
            });
        }
        let parent_body = |bone: usize| rig.parents[bone].and_then(|parent| owner[parent]);
        // Count descendants per root to choose the single root.
        let roots = selected
            .iter()
            .enumerate()
            .filter(|(_, bone)| parent_body(**bone).is_none())
            .map(|(body, _)| body)
            .collect::<Vec<_>>();
        let mut root_of = vec![0_usize; selected.len()];
        for (body, bone) in selected.iter().enumerate() {
            root_of[body] = parent_body(*bone).map_or(body, |parent| root_of[parent]);
        }
        let root = roots
            .iter()
            .copied()
            .max_by_key(|root| {
                let members = root_of.iter().filter(|r| *r == root).count();
                (members, std::cmp::Reverse(*root))
            })
            .unwrap_or(0);
        let kept = selected
            .iter()
            .enumerate()
            .filter(|(body, _)| root_of[*body] == root)
            .map(|(_, bone)| *bone)
            .collect::<Vec<_>>();
        // Rebuild ownership for the kept bodies only.
        let mut body_of = vec![None; count];
        for (body, bone) in kept.iter().enumerate() {
            body_of[*bone] = Some(body);
        }
        let mut owner = vec![None; count];
        for bone in 0..count {
            owner[bone] = body_of[bone].or_else(|| {
                rig.eligible[bone]
                    .then(|| rig.parents[bone].and_then(|parent| owner[parent]))
                    .flatten()
            });
        }
        let parents = kept
            .iter()
            .map(|bone| rig.parents[*bone].and_then(|parent| owner[parent]))
            .collect::<Vec<_>>();
        let mut children = vec![Vec::new(); kept.len()];
        for (body, parent) in parents.iter().enumerate() {
            if let Some(parent) = parent {
                children[*parent].push(body);
            }
        }
        Self {
            bones: kept,
            parents,
            children,
            owner,
        }
    }

    /// Returns `start` and its descendant bodies with their depth below `start`.
    fn subtree(&self, start: usize) -> Vec<(usize, usize)> {
        let mut members = vec![(start, 0)];
        let mut next = 0;
        while let Some((body, depth)) = members.get(next).copied() {
            members.extend(self.children[body].iter().map(|child| (*child, depth + 1)));
            next += 1;
        }
        members
    }

    /// Iterates the bones whose geometry belongs to `body`.
    fn owned_bones(&self, body: usize) -> impl Iterator<Item = usize> + '_ {
        self.owner
            .iter()
            .enumerate()
            .filter(move |(_, owner)| **owner == Some(body))
            .map(|(bone, _)| bone)
    }
}
