"""Generate the low-poly skinned test creatures in ``assets/rigs/``.

Run from the repository root with Blender on ``PATH``::

    blender -b --factory-startup --python assets/source/generate_creatures.py

The script starts from an empty scene and writes three GLB files next to this
directory, in ``../rigs/``:

* ``humanoid.glb``: a 1.8 m humanoid in T-pose with UE5 mannequin bone names.
* ``quadruped.glb``: a dog-like animal with a 4-segment tail.
* ``alien.glb``: a creature with 7 radial legs, 3 raised arms, and a head.

Each character is one armature plus one mesh parented to it with an Armature
modifier. The mesh is one tapered cylinder (or sphere) per deform bone, and
every vertex of a piece belongs fully to that bone's vertex group. Coordinates
are Blender world space: Z up, metres, the character faces -Y, and its left
side is +X. The lowest vertex sits at z = 0. The glTF exporter converts this to
Y up with the character facing +Z. The output is deterministic because the
script uses no randomness.
"""

import math
import os

import bpy
from mathutils import Vector

# Output directory, resolved relative to this file.
RIGS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rigs")
)

# Number of sides for every cylinder and sphere.
SEGMENTS = 8


class Bone:
    """One bone: name, parent name, head, tail, piece radii, and flags.

    ``radius`` is ``(radius at head, radius at tail)`` in metres for a
    cylinder piece. ``shape`` is ``"cylinder"``, ``"sphere"`` (centred between
    head and tail, radius ``radius[0]``), or ``None`` for a non-deform bone
    without geometry. ``connected`` requires the head to equal the parent tail.
    """

    def __init__(self, name, parent, head, tail, radius=(0.04, 0.04),
                 shape="cylinder", connected=False):
        self.name = name
        self.parent = parent
        self.head = Vector(head)
        self.tail = Vector(tail)
        self.radius = radius
        self.shape = shape
        self.connected = connected


def mirrored(bones):
    """Return ``bones`` for the left side (+X) plus mirrored right copies.

    In the right copies, X is negated, and the ``_l`` side token in names and
    parents (as a suffix or followed by ``_``) becomes ``_r``.
    """
    out = []
    for side, sign in (("l", 1.0), ("r", -1.0)):
        for b in bones:
            def rename(n):
                if n and n.endswith("_l"):
                    return n[:-2] + "_" + side
                return n.replace("_l_", "_" + side + "_") if n else n
            out.append(Bone(
                rename(b.name), rename(b.parent),
                (b.head.x * sign, b.head.y, b.head.z),
                (b.tail.x * sign, b.tail.y, b.tail.z),
                b.radius, b.shape, b.connected,
            ))
    return out


def humanoid():
    """Bones of a 1.8 m humanoid in T-pose using UE5 mannequin names."""
    core = [
        # Non-deform root bone at the origin.
        Bone("root", None, (0, 0, 0), (0, 0, 0.15), shape=None),
        Bone("pelvis", "root", (0, 0, 0.95), (0, 0, 1.05), (0.13, 0.13)),
        Bone("spine_01", "pelvis", (0, 0, 1.05), (0, 0, 1.18), (0.12, 0.12), connected=True),
        Bone("spine_02", "spine_01", (0, 0, 1.18), (0, 0, 1.31), (0.12, 0.13), connected=True),
        Bone("spine_03", "spine_02", (0, 0, 1.31), (0, 0, 1.44), (0.14, 0.12), connected=True),
        Bone("neck_01", "spine_03", (0, 0, 1.44), (0, 0, 1.56), (0.05, 0.05), connected=True),
        Bone("head", "neck_01", (0, 0, 1.56), (0, 0, 1.79), (0.11,), shape="sphere", connected=True),
    ]
    left = [
        Bone("clavicle_l", "spine_03", (0.03, 0, 1.40), (0.18, 0, 1.42), (0.05, 0.05)),
        Bone("upperarm_l", "clavicle_l", (0.18, 0, 1.42), (0.46, 0, 1.42), (0.05, 0.04), connected=True),
        Bone("lowerarm_l", "upperarm_l", (0.46, 0, 1.42), (0.72, 0, 1.42), (0.04, 0.035), connected=True),
        Bone("hand_l", "lowerarm_l", (0.72, 0, 1.42), (0.90, 0, 1.42), (0.035, 0.03), connected=True),
        Bone("thigh_l", "pelvis", (0.10, 0, 0.95), (0.10, 0, 0.52), (0.07, 0.055)),
        Bone("calf_l", "thigh_l", (0.10, 0, 0.52), (0.10, 0, 0.10), (0.05, 0.04), connected=True),
        Bone("foot_l", "calf_l", (0.10, 0, 0.10), (0.10, -0.13, 0.04), (0.04, 0.04), connected=True),
        Bone("ball_l", "foot_l", (0.10, -0.13, 0.04), (0.10, -0.21, 0.04), (0.04, 0.035), connected=True),
    ]
    return core + mirrored(left)


def quadruped():
    """Bones of a dog-like animal, about 0.6 m at the shoulder."""
    core = [
        Bone("body_hips", None, (0, 0.30, 0.50), (0, 0.0, 0.52), (0.10, 0.11)),
        Bone("body_chest", "body_hips", (0, 0.0, 0.52), (0, -0.28, 0.54), (0.11, 0.12), connected=True),
        Bone("neck", "body_chest", (0, -0.28, 0.54), (0, -0.38, 0.70), (0.06, 0.05), connected=True),
        Bone("skull", "neck", (0, -0.38, 0.70), (0, -0.54, 0.68), (0.07, 0.03), connected=True),
        Bone("tail_1", "body_hips", (0, 0.33, 0.52), (0, 0.42, 0.57), (0.035, 0.03)),
        Bone("tail_2", "tail_1", (0, 0.42, 0.57), (0, 0.50, 0.52), (0.03, 0.025), connected=True),
        Bone("tail_3", "tail_2", (0, 0.50, 0.52), (0, 0.57, 0.46), (0.025, 0.02), connected=True),
        Bone("tail_4", "tail_3", (0, 0.57, 0.46), (0, 0.62, 0.39), (0.02, 0.012), connected=True),
    ]
    x = 0.09
    left = [
        Bone("leg_front_l_1", "body_chest", (x, -0.22, 0.48), (x, -0.20, 0.28), (0.04, 0.035)),
        Bone("leg_front_l_2", "leg_front_l_1", (x, -0.20, 0.28), (x, -0.23, 0.07), (0.03, 0.025), connected=True),
        Bone("leg_front_l_3", "leg_front_l_2", (x, -0.23, 0.07), (x, -0.30, 0.03), (0.03, 0.03), connected=True),
        Bone("leg_back_l_1", "body_hips", (x, 0.26, 0.48), (x, 0.20, 0.28), (0.045, 0.035)),
        Bone("leg_back_l_2", "leg_back_l_1", (x, 0.20, 0.28), (x, 0.28, 0.08), (0.03, 0.025), connected=True),
        Bone("leg_back_l_3", "leg_back_l_2", (x, 0.28, 0.08), (x, 0.22, 0.03), (0.03, 0.03), connected=True),
    ]
    return core + mirrored(left)


def alien():
    """Bones of a creature with 7 radial legs, 3 raised arms, and a head."""
    bones = [
        Bone("torso", None, (0, 0, 0.55), (0, 0, 0.85), (0.18, 0.16)),
        Bone("upper_torso", "torso", (0, 0, 0.85), (0, 0, 1.15), (0.16, 0.12), connected=True),
        Bone("head", "upper_torso", (0, 0, 1.15), (0, 0, 1.39), (0.12,), shape="sphere", connected=True),
    ]

    def at(angle, radius, z):
        # Point at horizontal distance ``radius`` in direction ``angle``.
        # Angle 0 is -Y (the facing direction); angles increase toward +X.
        return (radius * math.sin(angle), -radius * math.cos(angle), z)

    # Legs: hip on the torso, knee raised, foot on the ground.
    for i in range(7):
        a = 2.0 * math.pi * i / 7.0
        n = "leg_%d_" % (i + 1)
        hip, knee, ankle, foot = at(a, 0.15, 0.62), at(a, 0.45, 0.78), at(a, 0.72, 0.38), at(a, 0.82, 0.03)
        bones += [
            Bone(n + "a", "torso", hip, knee, (0.045, 0.04)),
            Bone(n + "b", n + "a", knee, ankle, (0.04, 0.035), connected=True),
            Bone(n + "c", n + "b", ankle, foot, (0.035, 0.03), connected=True),
        ]
    # Arms: shoulder on the upper torso, raised outward and up.
    for j in range(3):
        a = 2.0 * math.pi * j / 3.0 + math.pi / 3.0
        n = "arm_%d_" % (j + 1)
        shoulder, elbow, wrist, tip = at(a, 0.12, 1.05), at(a, 0.35, 1.15), at(a, 0.55, 1.40), at(a, 0.62, 1.60)
        bones += [
            Bone(n + "a", "upper_torso", shoulder, elbow, (0.04, 0.035)),
            Bone(n + "b", n + "a", elbow, wrist, (0.035, 0.03), connected=True),
            Bone(n + "c", n + "b", wrist, tip, (0.03, 0.02), connected=True),
        ]
    return bones


def basis(axis):
    """Return two unit vectors perpendicular to ``axis`` and to each other."""
    ref = Vector((0, 0, 1)) if abs(axis.z) < 0.9 else Vector((1, 0, 0))
    u = axis.cross(ref).normalized()
    return u, axis.cross(u).normalized()


def cylinder(bone):
    """Return ``(verts, faces)`` for a capped tapered cylinder along ``bone``."""
    axis = (bone.tail - bone.head).normalized()
    u, v = basis(axis)
    verts, faces = [], []
    for end, r in ((bone.head, bone.radius[0]), (bone.tail, bone.radius[1])):
        for k in range(SEGMENTS):
            t = 2.0 * math.pi * k / SEGMENTS
            verts.append(end + r * (math.cos(t) * u + math.sin(t) * v))
    verts += [bone.head, bone.tail]
    s = SEGMENTS
    for k in range(s):
        k2 = (k + 1) % s
        faces.append((k, k2, s + k2, s + k))
        faces.append((2 * s, k2, k))
        faces.append((2 * s + 1, s + k, s + k2))
    return verts, faces


def sphere(bone):
    """Return ``(verts, faces)`` for a UV sphere centred on ``bone``."""
    centre = (bone.head + bone.tail) / 2.0
    r = bone.radius[0]
    rings = SEGMENTS // 2
    verts = [centre + Vector((0, 0, -r))]
    for i in range(1, rings):
        phi = math.pi * i / rings - math.pi / 2.0
        for k in range(SEGMENTS):
            t = 2.0 * math.pi * k / SEGMENTS
            verts.append(centre + r * Vector((math.cos(phi) * math.cos(t),
                                              math.cos(phi) * math.sin(t),
                                              math.sin(phi))))
    verts.append(centre + Vector((0, 0, r)))
    top = len(verts) - 1
    faces = []
    for k in range(SEGMENTS):
        k2 = (k + 1) % SEGMENTS
        faces.append((0, 1 + k2, 1 + k))
        last = 1 + (rings - 2) * SEGMENTS
        faces.append((top, last + k, last + k2))
        for i in range(rings - 2):
            a = 1 + i * SEGMENTS
            b = a + SEGMENTS
            faces.append((a + k, a + k2, b + k2, b + k))
    return verts, faces


def clear_scene():
    """Delete every object and orphan data block."""
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for coll in (bpy.data.meshes, bpy.data.armatures, bpy.data.materials):
        for block in list(coll):
            coll.remove(block)


def build(name, bones):
    """Create the armature and skinned mesh for ``bones`` and export a GLB."""
    clear_scene()

    # Build the mesh pieces first so the whole character can be lifted until
    # its lowest vertex rests on z = 0.
    pieces = []
    for b in bones:
        if b.shape == "cylinder":
            pieces.append((b.name, *cylinder(b)))
        elif b.shape == "sphere":
            pieces.append((b.name, *sphere(b)))
    lift = Vector((0, 0, -min(v.z for _, vs, _ in pieces for v in vs)))

    # Armature: bones are created in list order, so every parent exists
    # before its children.
    arm_data = bpy.data.armatures.new(name + "_armature")
    arm = bpy.data.objects.new(name + "_armature", arm_data)
    bpy.context.scene.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    for b in bones:
        eb = arm_data.edit_bones.new(b.name)
        # A geometry-free root bone stays at the origin; all others are lifted.
        offset = lift if b.shape is not None or b.parent else Vector()
        eb.head = b.head + offset
        eb.tail = b.tail + offset
        eb.roll = 0.0
        eb.use_deform = b.shape is not None
        if b.parent:
            eb.parent = arm_data.edit_bones[b.parent]
            eb.use_connect = b.connected
            if b.connected:
                assert (eb.head - eb.parent.tail).length < 1e-6, b.name
    bpy.ops.object.mode_set(mode="OBJECT")

    # Mesh: join all pieces and give each piece full weight on its bone.
    verts, faces, groups = [], [], []
    for bone_name, vs, fs in pieces:
        base = len(verts)
        verts += [v + lift for v in vs]
        faces += [tuple(base + i for i in f) for f in fs]
        groups.append((bone_name, list(range(base, len(verts)))))
    mesh_data = bpy.data.meshes.new(name + "_mesh")
    mesh_data.from_pydata([tuple(v) for v in verts], [], faces)
    mesh_data.update()
    mesh = bpy.data.objects.new(name, mesh_data)
    bpy.context.scene.collection.objects.link(mesh)
    for bone_name, indices in groups:
        mesh.vertex_groups.new(name=bone_name).add(indices, 1.0, "REPLACE")
    mesh.parent = arm
    mesh.modifiers.new("Armature", "ARMATURE").object = arm

    # Apply transforms (all identity here) and export only this character.
    bpy.ops.object.select_all(action="DESELECT")
    arm.select_set(True)
    mesh.select_set(True)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    os.makedirs(RIGS_DIR, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=os.path.join(RIGS_DIR, name + ".glb"),
        export_format="GLB",
        export_yup=True,
        export_skins=True,
        export_animations=False,
        export_extras=True,
        use_selection=True,
    )


def main():
    """Build and export every creature."""
    build("humanoid", humanoid())
    build("quadruped", quadruped())
    build("alien", alien())


main()
