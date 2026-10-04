# Example assets for an MIT OR Apache-2.0 Bevy active-ragdoll crate

Research date: 2026-10-03. Nothing large was downloaded. Bone and clip lists come from range-fetching only the GLB JSON chunk (or a zip central directory plus one small entry). "Verified" means read from the file. "Unverified" means taken from a web page only.

## 1. Humanoid with animations

### Rank 1: Quaternius Universal Animation Library 1 + 2, with Universal Base Characters (CC0)

License evidence:
- https://quaternius.itch.io/universal-animation-library : "Asset license: Creative Commons Zero v1.0 Universal". The description says "Free to use in personal, educational and commercial projects. (CC0 License)".
- https://quaternius.itch.io/universal-animation-library-2 : the description says "(CC0 License)". https://quaternius.com/packs/universalanimationlibrary2.html links https://creativecommons.org/publicdomain/zero/1.0/.
- https://quaternius.itch.io/universal-base-characters : "Asset license: Creative Commons Zero v1.0 Universal".
- https://quaternius.com/faq.html : "All models are under the CC0 License." It also says attribution is not necessary.
- The paid Pro and Source tiers are also CC0. Redistributing their files is legal, but it undercuts the author's sales. Ship Standard-tier files unless the owner decides otherwise.

Downloads (itch.io, free Standard tier; the purchase flow is click-through):
- UAL1: https://quaternius.itch.io/universal-animation-library . Standard.zip 15 MB. Pro.zip 41 MB costs $9.99. Source.zip 46 MB costs $14.99.
- UAL2: https://quaternius.itch.io/universal-animation-library-2 . Standard.zip 17 MB. Source.zip 50 MB costs $14.99.
- Base characters: https://quaternius.itch.io/universal-base-characters . Standard.zip 122 MB. Source.zip 600 MB costs $19.99.
- Third-party CC0 mirror of the v2-naming GLBs, used for the inspection below: https://github.com/Dallolz/moorfall-assets . Paths: `animations/UAL1.glb` (7.6 MB), `animations/UAL2.glb` (8.1 MB), `characters/Superhero_Male.glb` (2.5 MB), `characters/Superhero_Female.glb`, `characters/Mannequin_F.glb`. Its LICENSE.md states CC0 1.0 and credits Quaternius. Prefer the official itch files for the crate.
- An older pre-v2 copy (Rigify `DEF-*` naming, 53 joints) is at https://github.com/J-Ponzo/gltf-universal-animation-library (CC0-1.0 per GitHub). Do not use it; its bone names are obsolete.

Formats: glTF/GLB and FBX. Source tier adds .blend. The Unreal export is GLB since v2.0.

Changelog facts (itch page):
- v2.0 (23/1/2026): "Updated to new rig naming scheme (Same as modular outfits / base chars)".
- v3.0 (16/6/2026) for UAL1 and v2.0 (16/6/2026) for UAL2: "Added root motion to all locomotion and movement animations". Packs now ship "one with full Root Motion and one with Root Motion disabled".
- UAL2 v2.1 (5/7/2026): "Renamed Fall animation". So a Fall clip exists in UAL2, at least in a paid tier. Whether it is in Standard is unverified.

Skeleton (verified, identical in UAL1.glb, UAL2.glb and Superhero_Male.glb, 65 joints): UE4-Mannequin-style names.
`root, pelvis, spine_01, spine_02, spine_03, neck_01, Head, clavicle_l, upperarm_l, lowerarm_l, hand_l, index_01_l..index_03_l, index_04_leaf_l, middle_01..03_l, middle_04_leaf_l, pinky_01..03_l, pinky_04_leaf_l, ring_01..03_l, ring_04_leaf_l, thumb_01..03_l, thumb_04_leaf_l, (same for _r), thigh_l, calf_l, foot_l, ball_l, ball_leaf_l, thigh_r, calf_r, foot_r, ball_r, ball_leaf_r`.
The animation GLBs include one skinned mesh named `Mannequin`, so they work standalone without the base characters.

Standard-tier clips in the mirror (verified):
- UAL1 (43): A_TPose, Crouch_Fwd_Loop, Crouch_Idle_Loop, Dance_Loop, Death01, Driving_Loop, Fixing_Kneeling, Hit_Chest, Hit_Head, Idle_Loop, Idle_Talking_Loop, Idle_Torch_Loop, Interact, Jog_Fwd_Loop, Jump_Land, Jump_Loop, Jump_Start, PickUp_Table, Pistol_Aim_Down, Pistol_Aim_Neutral, Pistol_Aim_Up, Pistol_Idle_Loop, Pistol_Reload, Pistol_Shoot, Punch_Cross, Punch_Jab, Push_Loop, Roll, Sitting_Enter, Sitting_Exit, Sitting_Idle_Loop, Sitting_Talking_Loop, Spell_Simple_Enter, Spell_Simple_Exit, Spell_Simple_Idle_Loop, Spell_Simple_Shoot, Sprint_Loop, Swim_Fwd_Loop, Swim_Idle_Loop, Sword_Attack, Sword_Idle, Walk_Formal_Loop, Walk_Loop.
- UAL2 (43): A_TPose, Chest_Open, ClimbUp_1m_RM, Consume, Farm_Harvest, Farm_PlantSeed, Farm_Watering, Hit_Knockback, Hit_Knockback_RM, Idle_FoldArms_Loop, Idle_Lantern_Loop, Idle_No_Loop, Idle_Rail_Call, Idle_Rail_Loop, Idle_Shield_Break, Idle_Shield_Loop, Idle_TalkingPhone_Loop, LayToIdle, Melee_Hook, Melee_Hook_Rec, NinjaJump_Idle_Loop, NinjaJump_Land, NinjaJump_Start, OverhandThrow, Shield_Dash_RM, Shield_OneShot, Slide_Exit, Slide_Loop, Slide_Start, Sword_Block, Sword_Dash_RM, Sword_Regular_A, Sword_Regular_A_Rec, Sword_Regular_B, Sword_Regular_B_Rec, Sword_Regular_C, Sword_Regular_Combo, TreeChopping_Loop, Walk_Carry_Loop, Yes, Zombie_Idle_Loop, Zombie_Scratch, Zombie_Walk_Fwd_Loop.

Root motion in the mirror (verified by reading `root` and `pelvis` translation keys):
- Walk_Loop, Jog_Fwd_Loop and Zombie_Walk_Fwd_Loop are in place. The `root` translation stays at 0 and the pelvis returns to its start.
- `*_RM` clips carry motion on `root`. For example, Hit_Knockback_RM moves `root` 3.0 m along one axis.
- The mirror predates or omits the June 2026 root-motion export. The official v3 zip should contain both variants.

Pose checks by forward kinematics on the mirror (verified):
- LayToIdle frame 0: lying face-up (supine). The pelvis forward axis points world-up, and the toes point up.
- Hit_Knockback last frame: ends supine on the ground (pelvis height 0.06 m).
- Death01 last frame: ends supine.

Coverage of the requested clips (Standard tier):
| Need | Clip |
| --- | --- |
| idle | Idle_Loop |
| walk | Walk_Loop (also Zombie_Walk_Fwd_Loop, Walk_Carry_Loop) |
| run | Jog_Fwd_Loop, Sprint_Loop |
| turn in place / turning walk | none in Standard. The 8-direction locomotion is in Pro (unverified clip names). |
| hit / flinch | Hit_Chest, Hit_Head |
| stagger / stumble | Hit_Knockback (a knockdown, not a recoverable stagger). No stagger clip found. |
| falling | Jump_Loop or NinjaJump_Idle_Loop as stand-ins. A Fall clip exists in UAL2 (tier unverified). |
| get up face-up (supine) | LayToIdle |
| get up face-down (prone) | none found |
| death | Death01 |

Gaps: turn in place, stagger, prone get-up. Prone get-up could be authored in Blender from LayToIdle, or the crate could roll the ragdoll supine before playing LayToIdle. Pro-tier clip names are not published anywhere I found.

Attribution line (optional under CC0): "Universal Animation Library 1 and 2 and Universal Base Characters by Quaternius (https://quaternius.com), CC0 1.0."

### Rank 2: Kenney Animated Characters (CC0), as a fallback

- Pages: https://kenney.nl/assets/animated-characters-1 (License: Creative Commons CC0), https://kenney.nl/assets/animated-characters-protagonists, https://kenney.nl/assets/animated-characters-retro .
- Direct zip (verified listing): https://kenney.nl/media/pages/assets/animated-characters-protagonists/608191acc4-1774773108/kenney_animated-characters-protagonists.zip . It contains `Model/characterMedium.fbx`, `Animations/idle.fbx`, `jump.fbx`, `run.fbx`, skins and License.txt.
- Format: FBX only. Clips: idle, jump, run. There is no walk, hit, death or get-up clip.
- Bones (verified from the FBX): Unity-Humanoid/Mixamo-style names without the `mixamorig:` prefix. Root, Hips, Spine, Chest, UpperChest, Neck, Head, LeftShoulder, LeftArm, LeftForeArm, LeftHand, LeftHandIndex1-3, LeftHandThumb1-2, LeftUpLeg, LeftLeg, LeftFoot, LeftToes (and Right*). Control bones: HipsCtrl, *FootIK, *KneeCtrl, *HeelRoll, *ToeRoll.
- Kenney Blocky Characters 2.0 (CC0, GLB, 27 clips including die) has no skin. Every part is a separately animated node. It is unsuitable for a skinned ragdoll.

### Rank 3: Khronos glTF sample humanoids (CC-BY 4.0): test-only

- CesiumMan: https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CesiumMan . License CC-BY-4.0 by Cesium. The Cesium logo texture is under LicenseRef-LegalMark-Cesium (a trademark), so avoid shipping it. It has 19 joints (`Skeleton_torso_joint_1`, `leg_joint_L_1` ...) and one unnamed walk clip.
- RiggedFigure: https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/RiggedFigure . License CC-BY-4.0 by Cesium. It has 19 joints (`torso_joint_1..3`, `neck_joint_1..2`, `arm_joint_L_1..3`, `leg_joint_L_1..3,5`) and one unnamed clip.
- Neither has idle, hit, death or get-up clips. Use them only as glTF-loader smoke tests.
- Exclude BrainStem. Its license is the Poser EULA, which does not allow redistribution.
- Bevy's own assets contain no humanoid. `assets/models/animated/` holds only Fox.glb and MorphStressTest.gltf.

### Mocap libraries

- CMU Graphics Lab (http://mocap.cs.cmu.edu/): "This dataset of motions is free for all uses." The same page says: "You may include this data in commercially-sold products, but you may not resell this data directly, even in converted form." That resale clause is a restriction beyond CC0. Shipping a few converted clips in a free repository is not resale. Downstream users of an MIT/Apache crate could still violate the clause, so it needs a separate NOTICE. Format is ASF/AMC (and C3D). BVH conversions exist at cgspeed (linked from the CMU page). Each subject has its own calibrated skeleton. Use needs BVH-to-armature retargeting in Blender before glTF export. The CMU page says finger and thumb data should be ignored and toe and hand joints are noisy. It does contain falls and get-ups (search categories on the site); I did not verify specific trials.
- Not acceptable: Ubisoft LaFAN1 and the Bandai Namco motion dataset. Both are CC BY-NC-ND (from prior knowledge; not re-verified today). Mixamo is excluded by the brief.

## 2. Quadruped and creature rigs

| Rank | Need | Asset | License | Bones | Clips (verified) | Download |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | fox | Quaternius Fox (Ultimate Animated Animal Pack) | CC0 1.0 (poly.pizza "Licence":"CC0 1.0"; quaternius.com pack page links CC0) | 51 incl. IK/pole bones (Body, Back, Torso..Torso3, Neck1-3, Head, Ear1-4.L/R, Front/BackShoulder, *UpperLeg, *LowerLeg, Tail1-8, IK*, PoleTarget*) | Attack, Death, Eating, Gallop, Gallop_Jump, Idle, Idle_2, Idle_2_HeadLow, Idle_HitReact_Left, Idle_HitReact_Right, Jump_ToIdle, Walk. Each clip is duplicated with an `AnimalArmature|` prefix. | https://poly.pizza/m/Bc97C66HKi -> https://static.poly.pizza/e18e86df-1692-48d8-ac6e-1e25ab4ad574.glb |
| 2 | fox | Bevy/Khronos Fox.glb | Mesh CC0 (PixelMannen). Rig and animation CC-BY 4.0 (tomkranis). glTF conversion CC-BY 4.0 (@AsoboStudio, @scurest). | 24 (`_rootJoint`, `b_Root_00`, `b_Hip_01`, `b_Spine01_02`, `b_Spine02_03`, `b_Neck_04`, `b_Head_05`, front legs named `b_*UpperArm/ForeArm/Hand`, `b_Tail01-03`, `b_*Leg01-02`, `b_*Foot01-02`) | Survey, Walk, Run | Local: bevy `assets/models/animated/Fox.glb`. Upstream: https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/Fox |
| 1 | dinosaur, biped with tail | Quaternius T-Rex (Animated Dinosaur Pack, 2018) | CC0 1.0 | 29 (root, Body, Hips, Torso, Shoulders, Neck, Head, Back, Tail1-5, Front/BackLeg, *UpLeg, *LowLeg, *Foot .L/.R) | TRex_Attack, TRex_Death, TRex_Idle, TRex_Jump, TRex_Run, TRex_Walk | https://poly.pizza/m/UYtneO5FpF -> https://static.poly.pizza/34eed102-48f0-43dd-bc6f-ef7a6dfddfbb.glb |
| 1 | dinosaur, alternative | Quaternius Velociraptor | CC0 1.0 | 29 (same rig as T-Rex) | Velociraptor_Attack, _Death, _Idle, _Jump, _Run, _Walk | https://poly.pizza/m/cnlGH2UcDd -> https://static.poly.pizza/c1f0c4cb-c84f-415c-8323-d8cb871a2126.glb |
| 1 | crocodile | "Crocodile" by br-n518 (OpenGameArt) | CC0 (OGA "License(s): CC0") | not inspected (in .7z, 2.4 MB) | idle-loop, walk-loop, attack, death (page text; unverified) | https://opengameart.org/content/crocodile-0 -> https://opengameart.org/sites/default/files/crocodile.7z |
| 1 | rabbit | "Rabbit" by CDmir (OpenGameArt, 2015) | CC0 | not inspected | "rigged and animated"; clip names unpublished | https://opengameart.org/content/rabbit-0 -> https://opengameart.org/sites/default/files/rabbit.blend (6.0 MB), rabbit-FBX.7z (4.4 MB) |
| alt | rabbit | Quaternius "Bunny" | CC0 1.0 | 49, anthropomorphic biped (UpperArm, LowerArm, UpperLeg, LowerLeg, Ear1-3) | Death, Duck, HitReact, Idle, Jump, Jump_Idle, Jump_Land, No, Punch, Run, Walk, Wave, Weapon, Yes | https://poly.pizza/m/irZjWFARyl -> https://static.poly.pizza/084b5ebe-c3eb-4e64-9b17-06e2d1e3da5d.glb |

Rejected:
- Every poly.pizza alligator, crocodile, caiman, rabbit, lizard and gecko found is "Poly by Google" or another author under CC-BY 3.0. All are static, with no skin and no animations.
- Kenney Cube Pets (CC0, 2026, includes bunny and fox, GLB): node animation only, no skin. Clips are static, idle, walk, run, eat, dance, gesture-positive, gesture-negative. It is unusable for a skinned rig. Download: https://kenney.nl/media/pages/assets/cube-pets/44e58e945f-1774520254/kenney_cube-pets_1.0.zip
- Quaternius Frog (CC0, 28 bones): idle, attack, death and jump only, with no walk.

Official Quaternius pack downloads (Google Drive folders linked from the pack pages):
- Ultimate Animated Animals: https://drive.google.com/drive/folders/1uJ3N5HfB7jKTseJUNQr3N4YaN0UuEtHk
- Animated Dinosaurs: https://drive.google.com/drive/folders/1u5Fhu3ziuRlGonW6bUI7uClqBGoSNeF6

## 3. TGF human rig (`content/rigs/human/`)

Files: `human.blend` (502 KB), `build/human.glb` (108 KB, exported by Khronos glTF Blender I/O v5.2.40).

- One skin with 89 joints and no animations.
- There is no skinned visual mesh. No node references the skin. The 16 meshes are the 16 ragdoll capsules.
- The root node `human` has extras `{"tgf_rig": "human"}`.
- Every bone node has extras `tgf_length` (metres) and `tgf_mask` ("upper" or "lower"). IK bones also have `tgf_follows` (for example `ik_foot_l` follows `foot_l`).
- Sockets: `socket_weapon_l`, `socket_weapon_r`, `socket_eyes`, `socket_head_top`, `socket_back`.

Bone names (89):
root, center_of_mass, ik_foot_root, ik_foot_l, ik_foot_r, ik_hand_root, ik_hand_gun, ik_hand_l, ik_hand_r, interaction, pelvis, spine_01, spine_02, spine_03, spine_04, spine_05, clavicle_l, upperarm_l, lowerarm_l, hand_l, index_metacarpal_l, index_01_l, index_02_l, index_03_l, middle_metacarpal_l, middle_01_l, middle_02_l, middle_03_l, pinky_metacarpal_l, pinky_01_l, pinky_02_l, pinky_03_l, ring_metacarpal_l, ring_01_l, ring_02_l, ring_03_l, thumb_01_l, thumb_02_l, thumb_03_l, lowerarm_twist_01_l, lowerarm_twist_02_l, upperarm_twist_01_l, upperarm_twist_02_l, (the same 27 for _r), neck_01, neck_02, head, thigh_l, calf_l, calf_twist_01_l, calf_twist_02_l, foot_l, ball_l, thigh_twist_01_l, thigh_twist_02_l, (the same 8 for _r).

Ragdoll capsules: 16 nodes named `ragdoll_<bone>`. Each is a child of its bone, and the masses total 80.0 kg.
| Capsule | Parent bone | mass_kg | RagdollJoint torque_nm |
| --- | --- | --- | --- |
| ragdoll_pelvis | pelvis | 8.94 | none (root body, RagdollBody only) |
| ragdoll_spine_02 | spine_02 | 13.06 | 200 |
| ragdoll_spine_04 | spine_04 | 12.77 | 150 |
| ragdoll_head | head | 5.55 | 30 |
| ragdoll_upperarm_l/r | upperarm_l/r | 2.17 | 40 |
| ragdoll_lowerarm_l/r | lowerarm_l/r | 1.30 | 25 |
| ragdoll_hand_l/r | hand_l/r | 0.49 | 8 |
| ragdoll_thigh_l/r | thigh_l/r | 11.33 | 150 |
| ragdoll_calf_l/r | calf_l/r | 3.46 | 80 |
| ragdoll_foot_l/r | foot_l/r | 1.10 | 25 |

Skein extras on each capsule:
- `skein`: a list of `{ "tgf_rig::ragdoll::RagdollBody": { "mass_kg": f32 } }` and, except on the pelvis, `{ "tgf_rig::ragdoll::RagdollJoint": { "limit_x": {"min_deg","max_deg"}, "limit_y": {...}, "limit_z": {...}, "torque_nm": f32 } }`.
- There is also a Blender-UI mirror: `skein_two` (with `name` and `selected_type_path`) and `active_component_index`.
- Example: lowerarm uses limit_x 0..140 with y and z locked (a hinge). The calf uses limit_x -140..0.
- The type paths are `tgf_rig::...`. A standalone crate must either register types with those paths or rewrite the extras to its own type paths.

## 4. Mapping Quaternius UAL to TGF's rig

Most names match exactly by name. These match: root, pelvis, spine_01, spine_02, spine_03, neck_01, clavicle, upperarm, lowerarm, hand, index/middle/pinky/ring_01-03, thumb_01-03, thigh, calf, foot and ball (both sides).

A small bone map is still needed:
- `Head` (Quaternius) maps to `head` (TGF). Only the case differs.
- Quaternius has 3 spine bones (UE4 layout). TGF has 5 (UE5 layout). TGF `spine_04` and `spine_05` have no source, and `ragdoll_spine_04` attaches to `spine_04`. Map TGF spine_04 to Quaternius spine_03, or distribute spine_03's rotation over spine_03..05.
- TGF `neck_02` has no source. Leave it at rest or split neck_01's rotation.
- Ignore Quaternius-only leaf bones: `*_04_leaf_*` and `ball_leaf_*`.
- TGF-only bones stay at rest or follow their parent: metacarpals, twist bones, ik_*, center_of_mass and interaction.
- Rest poses and bone roll differ between the rigs. Transfer rotations relative to each rig's rest pose (retargeting); do not copy local rotations raw. Proportions also differ.

For the crate, the simplest path is to ship a Quaternius character and its clips as-is. Then build the ragdoll capsules on its own 65-bone skeleton, using the TGF capsule table as the template. The 16 ragdoll bones map 1:1 except spine_04 -> spine_03 and head -> Head.

Kenney's Humanoid names (Hips, LeftUpLeg, ...) would need a full name map to either rig.

## Attribution lines

- CC0 (optional): "Universal Animation Library 1 & 2, Universal Base Characters, Animated Dinosaur Pack, Ultimate Animated Animal Pack by Quaternius (quaternius.com), CC0 1.0."
- CC0 (optional): "Crocodile by br-n518 (opengameart.org/content/crocodile-0), CC0." and "Rabbit by CDmir (opengameart.org/content/rabbit-0), CC0."
- Required if Fox.glb ships: "Fox: low-poly model by PixelMannen (CC0 1.0); rigging and animation by @tomkranis on Sketchfab (CC-BY 4.0, https://sketchfab.com/models/371dea88d7e04a76af5763f2a36866bc); glTF conversion by @AsoboStudio and @scurest (CC-BY 4.0); via KhronosGroup/glTF-Sample-Assets."
- Required if CesiumMan or RiggedFigure ships: "CesiumMan / RiggedFigure by Cesium, CC-BY 4.0, via KhronosGroup/glTF-Sample-Assets."
- Requested if CMU data ships: "The data used in this project was obtained from mocap.cs.cmu.edu. The database was created with funding from NSF EIA-0196217."

## Unverified or open

- Pro and Source clip names for UAL1 and UAL2, including turn, strafe, Fall and Death02.
- Clip names and bone layout of the OGA rabbit and crocodile (files not downloaded).
- Whether the official UAL v3 Standard zip differs from the Dallolz mirror beyond root motion.
- Kenney Animated Characters 1 direct zip URL (it sits behind the donation flow; the Protagonists zip was inspected instead).
