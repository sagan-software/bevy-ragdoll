//! Humanoid detection from standard bone-name conventions.
//!
//! One alias table covers UE4 and UE5 mannequins, Mixamo, Unity, Godot and VRM
//! humanoid names, Rigify deform bones, and 3ds Max Biped style names. Spine
//! bodies are chosen structurally, so the conventions' different spine
//! numbering needs no table.

use crate::profile::BodyRole;

/// Body side taken from a bone-name side marker.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Side {
    /// The character's left side (`_l`, `.L`, `Left...`).
    Left,
    /// The character's right side (`_r`, `.R`, `Right...`).
    Right,
}

/// Humanoid limb slots that names must identify.
const LIMB_ALIASES: &[(&[&str], BodyRole)] = &[
    (&["upperarm", "arm"], BodyRole::UpperArm),
    (&["lowerarm", "forearm"], BodyRole::LowerArm),
    (&["hand"], BodyRole::Hand),
    (&["upperleg", "upleg", "thigh"], BodyRole::Thigh),
    (&["lowerleg", "leg", "calf", "shin"], BodyRole::Calf),
    (&["foot"], BodyRole::Foot),
];

/// Splits a bone name into a lowercase base without separators and a side.
///
/// It strips namespace prefixes (`mixamorig:`, `Armature|`), `mixamorigN_`,
/// `DEF-`, trailing numeric segments (`.001`) and side markers.
pub(super) fn normalize(name: &str) -> (String, Option<Side>) {
    // Rig prefixes and a leading side word come off first, then tokens split.
    let (lower, mut side) = strip_prefixes(name);
    let mut tokens = lower
        .split(['_', '.', '-', ' '])
        .filter(|token| !token.is_empty())
        .collect::<Vec<_>>();
    // Trailing numbers name segments (`spine_01`, `upper_arm.L.001`).
    while tokens.last().copied().is_some_and(is_number) {
        tokens.pop();
    }
    // A side token wins over a side word because it is the more specific marker.
    if let Some(marker) = strip_side_token(&mut tokens) {
        side = Some(marker);
    }
    (tokens.concat(), side)
}

/// Lowercases `name` and strips namespace, Mixamo, Rigify and side-word prefixes.
fn strip_prefixes(name: &str) -> (String, Option<Side>) {
    // Drop namespace prefixes such as `mixamorig:` and `Armature|`.
    let name = name.rsplit([':', '|']).next().unwrap_or(name);
    let mut lower = name.to_ascii_lowercase();
    // Strip Mixamo underscore prefixes and Rigify deform prefixes.
    if let Some(rest) = lower.strip_prefix("mixamorig") {
        let rest = rest.trim_start_matches(|c: char| c.is_ascii_digit());
        lower = rest.trim_start_matches('_').to_owned();
    }
    if let Some(rest) = ["def-", "def_", "def."]
        .into_iter()
        .find_map(|prefix| lower.strip_prefix(prefix))
    {
        lower = rest.to_owned();
    }
    // `left` and `right` only count as a whole leading word, so `leg` stays intact.
    let mut side = None;
    // Whole-word side prefixes such as `LeftUpperArm` or `leftUpperArm`.
    for (word, marker) in [("left", Side::Left), ("right", Side::Right)] {
        if let Some(rest) = lower.strip_prefix(word) {
            side = Some(marker);
            lower = rest.to_owned();
        }
    }
    (lower, side)
}

/// Returns whether `token` is a segment number such as `01` or `001`.
fn is_number(part: &str) -> bool {
    part.chars().all(|c| c.is_ascii_digit())
}

/// Removes a side token at either end (`hand_l`, `thigh.R`, `l_hand`).
///
/// A lone token is kept, so a bone named `l` keeps its name.
fn strip_side_token(tokens: &mut Vec<&str>) -> Option<Side> {
    // The last token is checked first because suffixes are the common form.
    for index in [tokens.len().wrapping_sub(1), 0] {
        // Only single-letter or whole-word markers count, so `leg` is not a side.
        let marker = match tokens.get(index).copied() {
            Some("l" | "left") => Some(Side::Left),
            Some("r" | "right") => Some(Side::Right),
            _ => None,
        };
        if let (Some(marker), true) = (marker, tokens.len() > 1) {
            tokens.remove(index);
            return Some(marker);
        }
    }
    None
}

/// Returns the humanoid role named by `name`, with its side for limbs.
///
/// A name that matches no role is tried again without its first token, so
/// 3ds Max Biped style names with a rig prefix and a middle side marker
/// (`Bip01 L Thigh`, `Bip01_Pelvis`) match too.
pub(super) fn role_of(name: &str) -> Option<(BodyRole, Option<Side>)> {
    let (base, side) = normalize(name);
    role_of_base(&base, side).or_else(|| {
        let (base, side) = normalize(without_rig_prefix(name)?);
        role_of_base(&base, side)
    })
}

/// Returns `name` without its first separator-delimited token, or `None`
/// when it has a single token.
fn without_rig_prefix(name: &str) -> Option<&str> {
    let (_, rest) = name.split_once(['_', '.', '-', ' '])?;
    let rest = rest.trim_start_matches(['_', '.', '-', ' ']);
    (!rest.is_empty()).then_some(rest)
}

/// Returns the role of a normalized base name and its side.
fn role_of_base(base: &str, side: Option<Side>) -> Option<(BodyRole, Option<Side>)> {
    match base {
        "hips" | "pelvis" => return Some((BodyRole::Pelvis, None)),
        "head" => return Some((BodyRole::Head, None)),
        _ => {}
    }
    let side = side?;
    LIMB_ALIASES
        .iter()
        .find(|(aliases, _)| aliases.contains(&base))
        .map(|(_, role)| (*role, Some(side)))
}

/// Inputs to humanoid detection: bone names, parents and eligibility.
pub(super) struct Bones<'a> {
    /// Bone names in parent-first order.
    pub(super) names: &'a [&'a str],
    /// Parent index of each bone.
    pub(super) parents: &'a [Option<usize>],
    /// Whether each bone may carry a body by name (helper bones may not).
    pub(super) eligible: &'a [bool],
}

#[expect(
    clippy::indexing_slicing,
    reason = "bone indexes come from the same skeleton vectors and limb chains have three bones"
)]
impl Bones<'_> {
    /// Returns whether `ancestor` is `bone` or one of its ancestors.
    const fn is_ancestor(&self, ancestor: usize, mut bone: usize) -> bool {
        loop {
            if bone == ancestor {
                return true;
            }
            match self.parents[bone] {
                Some(parent) => bone = parent,
                None => return false,
            }
        }
    }

    /// Returns the path from `bone` up to its root, starting with `bone`.
    fn ancestry(&self, mut bone: usize) -> Vec<usize> {
        // Parents precede children, so the walk always ends at a root.
        let mut path = vec![bone];
        while let Some(parent) = self.parents[bone] {
            path.push(parent);
            bone = parent;
        }
        // The path is ordered bone-first, which common_ancestor relies on.
        path
    }

    /// Returns the lowest common ancestor of two bones.
    fn common_ancestor(&self, first: usize, second: usize) -> Option<usize> {
        let second_path = self.ancestry(second);
        self.ancestry(first)
            .into_iter()
            .find(|bone| second_path.contains(bone))
    }
}

/// Finds the 16 humanoid body bones, or `None` when the names do not match.
///
/// The result pairs each body bone with its role in parent-first order: hips,
/// spine (when one exists), chest, head and three bones per limb.
#[expect(
    clippy::indexing_slicing,
    reason = "bone indexes come from the same skeleton vectors and limb chains have three bones"
)]
pub(super) fn detect(bones: &Bones<'_>) -> Option<Vec<(usize, BodyRole)>> {
    // Every eligible bone that names a role and side, in parent-first order.
    let found = bones
        .names
        .iter()
        .enumerate()
        .filter(|(index, _)| bones.eligible[*index])
        .filter_map(|(index, name)| role_of(name).map(|key| (key, index)))
        .collect::<Vec<_>>();
    // Candidates for a role and side, closest to the root first.
    let candidates = |role, side| {
        found
            .iter()
            .filter(move |(key, _)| *key == (role, side))
            .map(|(_, bone)| *bone)
    };
    // All four limbs must match before the torso is located from them.
    let limbs = limbs(bones, candidates)?;
    let get = |role, side| candidates(role, side).next();
    let mut slots = torso(bones, get, &limbs)?;
    // Parent-first bone order is the body order the generator expects.
    slots.extend(limbs);
    slots.sort_by_key(|(bone, _)| *bone);
    Some(slots)
}

/// Finds the twelve limb bones, left arm and leg first, each chain proximal first.
///
/// Each chain takes the first candidates, closest to the root, that descend
/// from one another, so a same-named helper outside the limb, such as an IK
/// target, does not hide the real limb bone.
fn limbs<I: Iterator<Item = usize>>(
    bones: &Bones<'_>,
    candidates: impl Fn(BodyRole, Option<Side>) -> I,
) -> Option<Vec<(usize, BodyRole)>> {
    // Two sides with an arm chain and a leg chain each give twelve slots.
    let mut limbs = Vec::with_capacity(12);
    for side in [Side::Left, Side::Right] {
        for chain in [
            [BodyRole::UpperArm, BodyRole::LowerArm, BodyRole::Hand],
            [BodyRole::Thigh, BodyRole::Calf, BodyRole::Foot],
        ] {
            // A missing segment anywhere means the rig is not a full humanoid.
            let [first, second, third] =
                chain.map(|role| candidates(role, Some(side)).collect::<Vec<_>>());
            // Each limb segment must descend from the previous one.
            let (first, second, third) = first.iter().find_map(|&first| {
                second
                    .iter()
                    .filter(|&&second| bones.is_ancestor(first, second))
                    .find_map(|&second| {
                        third
                            .iter()
                            .find(|&&third| bones.is_ancestor(second, third))
                            .map(|&third| (first, second, third))
                    })
            })?;
            limbs.extend([(first, chain[0]), (second, chain[1]), (third, chain[2])]);
        }
    }
    Some(limbs)
}

/// Finds the hips, spine, chest and head slots from names and the limb roots.
#[expect(
    clippy::indexing_slicing,
    reason = "bone indexes come from the same skeleton vectors and limbs holds twelve slots"
)]
fn torso(
    bones: &Bones<'_>,
    get: impl Fn(BodyRole, Option<Side>) -> Option<usize>,
    limbs: &[(usize, BodyRole)],
) -> Option<Vec<(usize, BodyRole)>> {
    let [arm_l, leg_l, arm_r, leg_r] = [limbs[0].0, limbs[3].0, limbs[6].0, limbs[9].0];
    // Hips come from the name, else from where the legs meet (Rigify `DEF-spine`).
    let hips = get(BodyRole::Pelvis, None).or_else(|| bones.common_ancestor(leg_l, leg_r))?;
    let head = get(BodyRole::Head, None)?;
    let chest = bones.common_ancestor(arm_l, arm_r)?;
    // The torso must form one chain from the hips through the chest to the head.
    if !bones.is_ancestor(hips, chest) || !bones.is_ancestor(chest, head) {
        return None;
    }
    // Slots are pushed proximal first; `detect` sorts them by bone afterwards.
    let mut slots = vec![(hips, BodyRole::Pelvis)];
    if chest != hips {
        // The spine body is the middle eligible bone strictly between hips and chest.
        let between = bones
            .ancestry(chest)
            .into_iter()
            .skip(1)
            .take_while(|bone| *bone != hips)
            .filter(|bone| bones.eligible[*bone])
            .collect::<Vec<_>>();
        // `between` runs from the chest down, so this picks the lower middle bone.
        if let Some(spine) = between.get(between.len() / 2) {
            slots.push((*spine, BodyRole::Spine));
        }
        slots.push((chest, BodyRole::Chest));
    }
    slots.push((head, BodyRole::Head));
    Some(slots)
}
