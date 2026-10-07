//! Tests for the private glTF rig importer.

use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use serde_json::{Map, json};

use super::*;

/// Creates one one-key Skein component entry.
fn component_entry(path: &str, value: Value) -> Value {
    Value::Object(Map::from_iter([(path.to_owned(), value)]))
}

/// Creates a named capsule node with its required body and optional joint.
fn capsule_node(name: &str, joint: Option<Value>) -> Value {
    let mut components = vec![component_entry(NEW_BODY_PATH, json!({"mass_kg": 1.0}))];
    if let Some(joint) = joint {
        components.push(component_entry(NEW_JOINT_PATH, joint));
    }
    json!({
        "name": name,
        "mesh": 0,
        "extras": {"skein": components}
    })
}

/// Creates a two-bone GLB JSON document with two valid capsule bodies.
fn valid_document() -> Value {
    let joint = json!({
        "limit_x": {"min_deg": -90.0, "max_deg": 90.0},
        "limit_y": {"min_deg": 0.0, "max_deg": 0.0},
        "limit_z": {"min_deg": 0.0, "max_deg": 0.0},
        "torque_nm": 12.0
    });
    json!({
        "asset": {"version": "2.0"},
        "nodes": [
            {"name": "root", "children": [1, 2], "extras": {"tgf_length": 1.0}},
            capsule_node("root_capsule", None),
            {"name": "child", "children": [3], "extras": {"tgf_length": 1.0}},
            capsule_node("child_capsule", Some(joint))
        ],
        "skins": [{"joints": [0, 2]}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "accessors": [{"min": [-0.5, -0.1, -0.1], "max": [0.5, 0.1, 0.1]}]
    })
}

/// Builds a GLB container from JSON and optional chunks.
fn glb(json_bytes: &[u8], chunks: &[(u32, Vec<u8>)]) -> Vec<u8> {
    let mut json_chunk = json_bytes.to_vec();
    while !json_chunk.len().is_multiple_of(4) {
        json_chunk.push(b' ');
    }
    let extra_length: usize = chunks.iter().map(|(_, payload)| 8 + payload.len()).sum();
    let total_length = 20 + json_chunk.len() + extra_length;
    let mut bytes = Vec::with_capacity(total_length);
    bytes.extend_from_slice(b"glTF");
    bytes.extend_from_slice(&2_u32.to_le_bytes());
    bytes.extend_from_slice(&(total_length as u32).to_le_bytes());
    bytes.extend_from_slice(&(json_chunk.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&JSON_CHUNK.to_le_bytes());
    bytes.extend_from_slice(&json_chunk);
    for (kind, payload) in chunks {
        bytes.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&kind.to_le_bytes());
        bytes.extend_from_slice(payload);
    }
    bytes
}

/// Imports a JSON document through the public GLB parsing sequence.
fn import(document: &Value) -> Result<ProfileSpec, GltfRigError> {
    let json_bytes = serde_json::to_vec(document).expect("fixture serializes");
    from_glb(&glb(&json_bytes, &[]))
}

/// Returns a parser diagnostic after mutating one valid JSON fixture.
fn diagnostic(document: &Value) -> String {
    import(document)
        .expect_err("the fixture is malformed")
        .to_string()
}

/// Covers the header, JSON chunk, optional BIN chunk, and ignored chunks.
#[test]
fn glb_container_checks_headers_and_chunk_rules() {
    let valid_json = b"{}";
    let valid = glb(valid_json, &[(BIN_CHUNK, vec![0; 4])]);
    assert!(json_chunk(&valid).is_ok());
    let unknown_first = glb(valid_json, &[(0x1234, vec![0; 4])]);
    assert!(json_chunk(&unknown_first).is_ok());
    let bin_then_unknown = glb(valid_json, &[(BIN_CHUNK, vec![0; 4]), (0x1234, vec![0; 4])]);
    assert!(json_chunk(&bin_then_unknown).is_ok());
    assert!(validate_remaining_chunks(&glb(valid_json, &[]), 20 + 4).is_ok());

    let mut bad_magic = glb(valid_json, &[]);
    bad_magic[0] = b'X';
    assert!(json_chunk(&bad_magic).is_err());
    let mut bad_version = glb(valid_json, &[]);
    bad_version[4..8].copy_from_slice(&1_u32.to_le_bytes());
    assert!(json_chunk(&bad_version).is_err());
    let mut bad_total = glb(valid_json, &[]);
    bad_total[8..12].copy_from_slice(&0_u32.to_le_bytes());
    assert!(json_chunk(&bad_total).is_err());
    assert!(json_chunk(b"glTF\x02\0\0\0").is_err());
    assert!(json_chunk(b"glTF\x02\0\0\0\x0c\0\0\0").is_err());

    let mut bad_type = glb(valid_json, &[]);
    bad_type[16..20].copy_from_slice(&BIN_CHUNK.to_le_bytes());
    assert!(json_chunk(&bad_type).is_err());
    let mut unaligned_json = glb(valid_json, &[]);
    unaligned_json[12..16].copy_from_slice(&3_u32.to_le_bytes());
    unaligned_json[8..12].copy_from_slice(&23_u32.to_le_bytes());
    unaligned_json.truncate(23);
    assert!(json_chunk(&unaligned_json).is_err());
    let mut truncated_json = glb(valid_json, &[]);
    truncated_json[12..16].copy_from_slice(&8_u32.to_le_bytes());
    assert!(json_chunk(&truncated_json).is_err());

    let mut truncated_header = glb(valid_json, &[]);
    truncated_header.truncate(23);
    assert!(validate_remaining_chunks(&truncated_header, 20).is_err());
    let mut unaligned_chunk = glb(valid_json, &[(0x1234, vec![0; 4])]);
    unaligned_chunk[24..28].copy_from_slice(&2_u32.to_le_bytes());
    assert!(validate_remaining_chunks(&unaligned_chunk, 24).is_err());
    let mut truncated_payload = glb(valid_json, &[(0x1234, vec![0; 4])]);
    truncated_payload[24..28].copy_from_slice(&8_u32.to_le_bytes());
    assert!(validate_remaining_chunks(&truncated_payload, 24).is_err());
    assert!(json_chunk(&truncated_payload).is_err());
    let repeated_json = glb(valid_json, &[(JSON_CHUNK, vec![0; 4])]);
    assert!(validate_remaining_chunks(&repeated_json, 24).is_err());
    let repeated_bin = glb(
        valid_json,
        &[(BIN_CHUNK, vec![0; 4]), (BIN_CHUNK, vec![0; 4])],
    );
    assert!(validate_remaining_chunks(&repeated_bin, 24).is_err());
    let bin_after_unknown = glb(valid_json, &[(0x1234, vec![0; 4]), (BIN_CHUNK, vec![0; 4])]);
    assert!(validate_remaining_chunks(&bin_after_unknown, 24).is_err());
    let bin_not_second = glb(
        valid_json,
        &[
            (BIN_CHUNK, vec![0; 4]),
            (0x1234, vec![0; 4]),
            (BIN_CHUNK, vec![0; 4]),
        ],
    );
    assert!(validate_remaining_chunks(&bin_not_second, 24).is_err());
}

/// Covers little-endian reads and glTF version-profile boundaries.
#[test]
fn reads_words_and_validates_asset_profile() {
    assert_eq!(read_u32(&[1, 0, 0, 0], 0), Some(1));
    assert_eq!(read_u32(&[1, 0, 0, 0], usize::MAX), None);
    assert_eq!(read_u32(&[1, 0, 0], 0), None);
    assert!(validate_asset_profile(&json!({})).is_err());
    assert!(validate_asset_profile(&json!({"asset":{}})).is_err());
    assert!(validate_asset_profile(&json!({"asset":{"version":2}})).is_err());
    assert!(validate_asset_profile(&json!({"asset":{"version":"2.00"}})).is_err());
    assert!(validate_asset_profile(&json!({"asset":{"version":"2.0"}})).is_ok());
    assert!(validate_asset_profile(&json!({"asset":{"version":"2.0","minVersion":"2.0"}})).is_ok());
    assert!(validate_asset_profile(&json!({"asset":{"version":"1.0"}})).is_err());
    assert!(validate_asset_profile(&json!({"asset":{"version":"2.1"}})).is_err());
    assert!(
        validate_asset_profile(&json!({"asset":{"version":"2.0","minVersion":"1.0"}})).is_err()
    );
    assert!(
        validate_asset_profile(&json!({"asset":{"version":"2.0","minVersion":"2.1"}})).is_err()
    );
    assert!(
        validate_asset_profile(&json!({"asset":{"version":"2.0","minVersion":"2.00"}})).is_err()
    );
    assert!(validate_asset_profile(&json!({"asset":{"version":"2.0","minVersion":2}})).is_err());
    assert_eq!(
        validate_asset_profile(&json!({"asset":{"version":"2.0","minVersion":null}}))
            .unwrap_err()
            .to_string(),
        "the glTF asset minimum version is not a string"
    );
}

/// Rejects required extensions this importer does not implement.
#[test]
fn rig_import_rejects_required_extensions() {
    let mut document = valid_document();
    document["extensionsUsed"] = json!(["VENDOR_ragdoll_test"]);
    document["extensions"] = json!({"VENDOR_ragdoll_test":{"profile":true}});
    assert!(import(&document).is_ok());

    document["extensionsRequired"] = json!(["VENDOR_ragdoll_test"]);
    assert_eq!(
        diagnostic(&document),
        "invalid GLB ragdoll rig: the glTF asset requires unsupported extensions"
    );
}

/// Rejects malformed required-extension lists.
#[test]
fn rig_import_rejects_malformed_required_extension_lists() {
    for required in [json!(null), json!([]), json!([2])] {
        let mut document = valid_document();
        document["extensionsRequired"] = required;
        assert!(import(&document).is_err());
    }
}

/// Covers malformed node trees, hierarchy cycles, and transform composition.
#[test]
fn node_tree_validates_children_and_world_matrices() {
    let mut document = valid_document();
    assert!(NodeTree::new(&document).is_ok());
    assert_eq!(NodeTree::new(&document).unwrap().name(0), Some("root"));
    assert!(NodeTree::new(&document).unwrap().name(1).is_some());
    assert!(NodeTree::new(&document).unwrap().extras(1).is_some());
    assert!(NodeTree::new(&document).unwrap().extras(2).is_some());
    document["nodes"] = json!(null);
    assert!(NodeTree::new(&document).is_err());

    for children in [json!(1), json!([-1]), json!([99]), json!([0])] {
        let mut invalid = valid_document();
        invalid["nodes"][0]["children"] = children;
        assert!(NodeTree::new(&invalid).is_err());
    }
    let mut repeated_parent = valid_document();
    repeated_parent["nodes"][2]["children"] = json!([1, 3]);
    assert!(NodeTree::new(&repeated_parent).is_err());
    let cycle = json!({"nodes":[{"children":[1]},{"children":[0]}]});
    assert!(NodeTree::new(&cycle).is_err());

    let mut bad_transform = valid_document();
    bad_transform["nodes"][0]["translation"] = json!([0.0, 0.0]);
    assert!(NodeTree::new(&bad_transform).is_err());
    let overflow =
        json!({"nodes":[{"scale":[1.0e20,1.0,1.0],"children":[1]},{"scale":[1.0e20,1.0,1.0]}]});
    assert!(NodeTree::new(&overflow).is_err());
}

/// Covers skin shape, joint references, roots, names, extras, and bone transforms.
#[test]
fn bone_import_rejects_invalid_skin_and_bone_metadata() {
    let document = valid_document();
    let nodes = NodeTree::new(&document).unwrap();
    assert!(read_bones(&document, &nodes).is_ok());

    let disconnected = json!({
        "skins": [{"joints": [0, 1, 2]}],
        "nodes": [
            {"name":"root", "extras":{"tgf_length":1.0}},
            {"name":"cycle_a", "extras":{"tgf_length":1.0}},
            {"name":"cycle_b", "extras":{"tgf_length":1.0}}
        ]
    });
    let disconnected_nodes = disconnected["nodes"].as_array().unwrap();
    let disconnected_tree = NodeTree {
        nodes: disconnected_nodes,
        parents: vec![None, Some(2), Some(1)],
        world: vec![Mat4::IDENTITY; 3],
    };
    assert!(
        read_bones(&disconnected, &disconnected_tree)
            .err()
            .is_some_and(|problem| problem.to_string().contains("connected bone tree"))
    );

    for skins in [
        json!(null),
        json!([]),
        json!([{}, {}]),
        json!([{"joints":[]}]),
        json!([{"joints":[99]}]),
        json!([{"joints":[0,0]}]),
    ] {
        let mut invalid = valid_document();
        invalid["skins"] = skins;
        let tree = NodeTree::new(&invalid).unwrap();
        assert!(read_bones(&invalid, &tree).is_err());
    }

    let mut multiple_roots = valid_document();
    multiple_roots["skins"][0]["joints"] = json!([0, 2]);
    multiple_roots["nodes"][2]
        .as_object_mut()
        .unwrap()
        .remove("children");
    multiple_roots["nodes"][2]["children"] = json!([3]);
    multiple_roots["nodes"][2]["extras"]["tgf_length"] = json!(1.0);
    // Node 2 has no joint ancestor when detached from the root.
    multiple_roots["nodes"][0]["children"] = json!([1]);
    let tree = NodeTree::new(&multiple_roots).unwrap();
    assert!(read_bones(&multiple_roots, &tree).is_err());

    for name in [json!(null), json!("")] {
        let mut invalid = valid_document();
        invalid["nodes"][0]["name"] = name;
        assert!(diagnostic(&invalid).contains("non-empty name"));
    }
    let mut duplicate_name = valid_document();
    duplicate_name["nodes"][2]["name"] = json!("root");
    assert!(diagnostic(&duplicate_name).contains("listed twice"));
    for extras in [json!(null), json!([])] {
        let mut invalid = valid_document();
        invalid["nodes"][0]["extras"] = extras;
        assert!(diagnostic(&invalid).contains("extras are missing or not an object"));
    }
    for extras in [json!({"tgf_length":0.0}), json!({"tgf_length":"1"})] {
        let mut invalid = valid_document();
        invalid["nodes"][0]["extras"] = extras;
        assert!(diagnostic(&invalid).contains("tgf_length"));
    }
    let mut singular = valid_document();
    singular["nodes"][0]["scale"] = json!([0.0, 1.0, 1.0]);
    assert!(diagnostic(&singular).contains("singular"));
    assert!(pose_of(Mat4::from_scale(Vec3::ZERO)).is_none());
    let non_isometric_document = valid_document();
    let mut non_isometric = NodeTree::new(&non_isometric_document).unwrap();
    non_isometric.world[0] = Mat4::from_translation(Vec3::new(f32::NAN, 0.0, 0.0));
    let problem = read_bones(&non_isometric_document, &non_isometric)
        .err()
        .expect("the decomposed rest pose must be a finite isometry");
    assert!(problem.to_string().contains("cannot form an isometry"));
}

/// Covers body association, capsule geometry, and every Skein component failure.
#[test]
fn body_import_validates_components_bounds_and_parent_links() {
    let document = valid_document();
    let tree = NodeTree::new(&document).unwrap();
    let (bones, bone_for_node) = read_bones(&document, &tree).unwrap();
    let (bodies, joints) = read_bodies(&document, &tree, &bones, &bone_for_node).unwrap();
    assert_eq!(bodies.len(), 2);
    assert_eq!(joints.len(), 1);
    assert_eq!(bodies[0].bone, "root");
    assert!(bodies[0].rest.rotation.is_finite());

    let no_components = json!({"nodes":[{"name":"empty"}]});
    let no_components_tree = NodeTree::new(&no_components).unwrap();
    assert!(skein_components(&no_components_tree, 0).unwrap().is_empty());
    let no_skein = json!({"nodes":[{"extras":{}}]});
    assert!(
        skein_components(&NodeTree::new(&no_skein).unwrap(), 0)
            .unwrap()
            .is_empty()
    );
    let mut bad_extras = json!({"nodes":[{"extras":[]} ]});
    assert!(skein_components(&NodeTree::new(&bad_extras).unwrap(), 0).is_err());
    bad_extras["nodes"][0]["extras"] = json!({"skein":{}});
    assert!(skein_components(&NodeTree::new(&bad_extras).unwrap(), 0).is_err());
    bad_extras["nodes"][0]["extras"] = json!({"skein":[{}, {"A":1,"B":2}]});
    assert!(skein_components(&NodeTree::new(&bad_extras).unwrap(), 0).is_err());

    let mut invalid = valid_document();
    let components = invalid["nodes"][1]["extras"]["skein"]
        .as_array_mut()
        .unwrap();
    components.push(component_entry(NEW_BODY_PATH, json!({"mass_kg":1.0})));
    assert!(diagnostic(&invalid).contains("attached more than once"));
    let mut malformed_component = valid_document();
    malformed_component["nodes"][1]["extras"]["skein"][0] =
        component_entry(NEW_BODY_PATH, json!({"mass_kg":"heavy"}));
    assert!(diagnostic(&malformed_component).contains("RagdollBody"));
    let mut joint_without_body = valid_document();
    joint_without_body["nodes"][1]["extras"]["skein"] = json!([component_entry(
        NEW_JOINT_PATH,
        json!({
            "limit_x": {"min_deg": -90.0, "max_deg": 90.0},
            "limit_y": {"min_deg": 0.0, "max_deg": 0.0},
            "limit_z": {"min_deg": 0.0, "max_deg": 0.0},
            "torque_nm": 12.0
        })
    )]);
    assert!(diagnostic(&joint_without_body).contains("has no RagdollBody"));
    let mut invalid_body = valid_document();
    invalid_body["nodes"][1]["extras"]["skein"][0] =
        component_entry(NEW_BODY_PATH, json!({"mass_kg":0.0}));
    assert!(diagnostic(&invalid_body).contains("mass_kg"));
    let mut invalid_joint = valid_document();
    invalid_joint["nodes"][3]["extras"]["skein"][1] = component_entry(
        NEW_JOINT_PATH,
        json!({"limit_x":{"min_deg":1.0,"max_deg":0.0},"limit_y":{"min_deg":0.0,"max_deg":0.0},"limit_z":{"min_deg":0.0,"max_deg":0.0},"torque_nm":1.0}),
    );
    assert!(diagnostic(&invalid_joint).contains("limit_x"));
    let mut malformed_joint = valid_document();
    malformed_joint["nodes"][3]["extras"]["skein"][1] = component_entry(NEW_JOINT_PATH, json!({}));
    assert!(diagnostic(&malformed_joint).contains("RagdollJoint"));

    let mut absent_accessors = valid_document();
    absent_accessors["accessors"] = json!(null);
    assert!(diagnostic(&absent_accessors).contains("accessors member"));
    let mut missing_position_accessor = valid_document();
    missing_position_accessor["accessors"] = json!([]);
    assert!(diagnostic(&missing_position_accessor).contains("POSITION accessor index"));
    let mut reversed = valid_document();
    reversed["accessors"][0]["min"] = json!([1.0, 0.0, 0.0]);
    assert!(diagnostic(&reversed).contains("reversed bound"));
    let mut non_round = valid_document();
    non_round["accessors"][0]["max"] = json!([0.5, 0.2, 0.1]);
    assert!(diagnostic(&non_round).contains("round capsule"));
    let mut absent_meshes = valid_document();
    absent_meshes["meshes"] = json!(null);
    assert!(diagnostic(&absent_meshes).contains("meshes member"));
    let mut missing_mesh = valid_document();
    missing_mesh["nodes"][1]
        .as_object_mut()
        .unwrap()
        .remove("mesh");
    assert!(diagnostic(&missing_mesh).contains("body mesh index"));
    let mut empty_primitives = valid_document();
    empty_primitives["meshes"][0]["primitives"] = json!([]);
    assert!(diagnostic(&empty_primitives).contains("no primitives"));
    let mut no_position_bounds = valid_document();
    no_position_bounds["accessors"][0]
        .as_object_mut()
        .unwrap()
        .remove("min");
    assert!(diagnostic(&no_position_bounds).contains("finite min"));
    let mut no_maximum = valid_document();
    no_maximum["accessors"][0]
        .as_object_mut()
        .unwrap()
        .remove("max");
    assert!(diagnostic(&no_maximum).contains("finite max"));

    let mut malformed_skein = valid_document();
    malformed_skein["nodes"][1]["extras"]["skein"] = json!({});
    assert!(diagnostic(&malformed_skein).contains("skein extras are not a list"));

    let mut non_finite_world = NodeTree::new(&document).unwrap();
    non_finite_world.world[1] = Mat4::from_translation(Vec3::splat(f32::INFINITY));
    let overflow_problem = read_body(&document, &non_finite_world, 1, 0, &bones)
        .err()
        .expect("the transformed capsule must remain finite");
    assert!(
        overflow_problem
            .to_string()
            .contains("capsule transform is not finite"),
        "{overflow_problem}"
    );

    let mut duplicate_bodies = valid_document();
    duplicate_bodies["nodes"][0]["children"] = json!([1, 2, 4]);
    duplicate_bodies["nodes"]
        .as_array_mut()
        .unwrap()
        .push(capsule_node("duplicate", None));
    assert!(diagnostic(&duplicate_bodies).contains("two ragdoll body nodes"));

    let mut no_ragdoll_bodies = valid_document();
    no_ragdoll_bodies["nodes"][1]["extras"]["skein"] = json!([]);
    no_ragdoll_bodies["nodes"][3]["extras"]["skein"] = json!([]);
    assert!(diagnostic(&no_ragdoll_bodies).contains("root bodies; it needs one"));

    let mut root_has_joint = valid_document();
    root_has_joint["nodes"][1]["extras"]["skein"].as_array_mut().unwrap().push(component_entry(NEW_JOINT_PATH, json!({"limit_x":{"min_deg":0.0,"max_deg":0.0},"limit_y":{"min_deg":0.0,"max_deg":0.0},"limit_z":{"min_deg":0.0,"max_deg":0.0},"torque_nm":1.0})));
    assert!(diagnostic(&root_has_joint).contains("root body"));
    let mut child_no_joint = valid_document();
    child_no_joint["nodes"][3]["extras"]["skein"]
        .as_array_mut()
        .unwrap()
        .pop();
    assert!(diagnostic(&child_no_joint).contains("has no RagdollJoint"));
}

/// Covers small parser helpers, local matrices, and all fixed-array failures.
#[test]
fn transform_and_array_helpers_check_each_shape() {
    assert_eq!(json_index(Some(&json!(0)), 1, "item").unwrap(), 0);
    assert!(json_index(None, 1, "item").is_err());
    assert!(json_index(Some(&json!(-1)), 1, "item").is_err());
    assert!(json_index(Some(&json!(1)), 1, "item").is_err());
    assert_eq!(
        vector3(Some(&json!([1, 2, 3]))),
        Some(Vec3::new(1.0, 2.0, 3.0))
    );
    assert!(vector3(Some(&json!([1, 2]))).is_none());
    assert!(vector3(Some(&json!("bad"))).is_none());
    assert!(vector3(Some(&json!([1, "two", 3]))).is_none());
    assert!(vector3(Some(&json!([1.0e100, 0, 0]))).is_none());
    let converted = radians(AngleRange {
        min_deg: -180.0,
        max_deg: 180.0,
    });
    assert!((converted.min + PI).abs() < 1e-6 && (converted.max - PI).abs() < 1e-6);

    assert!(local_transform(&json!({})).is_ok());
    assert!(local_transform(&json!({"translation":[1,2,3],"scale":[2,2,2]})).is_ok());
    assert!(local_transform(&json!({"matrix":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]})).is_ok());
    assert!(local_transform(&json!({"matrix":[],"translation":[0,0,0]})).is_err());
    assert!(local_transform(&json!({"matrix":[]})).is_err());
    assert!(local_transform(&json!({"matrix":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,2]})).is_err());
    assert!(
        local_transform(&json!({"matrix":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],"rotation":[0,0,0,1]}))
            .is_err()
    );
    assert!(local_transform(&json!({"rotation":[0,0,0,2]})).is_err());
    assert!(local_transform(&json!({"rotation":[0,0,0]})).is_err());
    assert!(local_transform(&json!({"scale":[1.0e100,1,1]})).is_err());
    assert!(optional_array::<3>(&json!({}), "translation", [0.0; 3]).is_ok());
    assert!(optional_array::<3>(&json!({"translation":[1,2,3]}), "translation", [0.0; 3]).is_ok());
    assert!(optional_array::<3>(&json!({"translation":[1,2]}), "translation", [0.0; 3]).is_err());
    assert!(number_array::<2>(&json!([1, 2])).is_some());
    assert!(number_array::<2>(&json!([1])).is_none());
    assert!(number_array::<2>(&json!("bad")).is_none());
    assert!(number_array::<2>(&json!([1, "two"])).is_none());
    assert!(number_array::<2>(&json!([1.0e100, 0])).is_none());

    let shear = Mat4::from_cols(
        Vec3::X.extend(0.0),
        Vec3::new(1.0, 1.0, 0.0).extend(0.0),
        Vec3::Z.extend(0.0),
        Vec3::ZERO.extend(1.0),
    );
    assert!(is_valid_local_matrix(Mat4::IDENTITY));
    assert!(!is_valid_local_matrix(Mat4::from_scale(Vec3::splat(
        f32::INFINITY
    ))));
    assert!(!is_valid_local_matrix(Mat4::from_cols_array(&[
        1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0
    ])));
    assert!(!is_valid_local_matrix(shear));
    assert!(is_finite_matrix(Mat4::IDENTITY));
    assert!(!is_finite_matrix(Mat4::from_scale(Vec3::splat(
        f32::INFINITY
    ))));
    assert!(is_unit_rotation(Quat::IDENTITY));
    assert!(!is_unit_rotation(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)));
}

/// Covers checked slice access, an out-of-range parent index, and capsule axis ordering.
#[test]
fn private_index_and_capsule_axis_boundaries_are_checked() {
    assert!(slot(&[1], 1, "test item").is_err());
    assert!(slot_mut(&mut [1], 1, "test item").is_err());

    let body = ImportedBody {
        bone: 0,
        spec: BodySpec {
            bone: "root".to_owned(),
            shape: ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 1.0,
            },
            mass: 1.0,
            rest: Isometry3d::IDENTITY,
            role: None,
        },
        joint: Some(RagdollJoint::default()),
    };
    let problem = make_joint_spec(0, 256, &body, RagdollJoint::default(), &[])
        .expect_err("a parent index above 255 cannot fit the profile joint");
    assert!(
        problem
            .to_string()
            .contains("body parent index exceeds 255")
    );

    assert_eq!(longest_capsule_axis([3.0, 2.0, 1.0]), 0);
    assert_eq!(longest_capsule_axis([1.0, 3.0, 2.0]), 1);
    assert_eq!(longest_capsule_axis([1.0, 1.0, 1.0]), 2);
}

/// Finds a finite unit quaternion whose largest finite scale overflows its matrix.
#[test]
fn trs_matrix_overflow_is_rejected() {
    let angles = [-PI, -FRAC_PI_2, -FRAC_PI_4, 0.0, FRAC_PI_4, FRAC_PI_2, PI];
    let mut overflow_found = false;
    'rotations: for x in angles {
        for y in angles {
            for z in angles {
                let rotation =
                    Quat::from_rotation_x(x) * Quat::from_rotation_y(y) * Quat::from_rotation_z(z);
                let node = json!({
                    "rotation": rotation.to_array(),
                    "scale": Vec3::splat(f32::MAX).to_array()
                });
                if local_transform(&node)
                    .err()
                    .is_some_and(|problem| problem.to_string() == "TRS transform is not finite")
                {
                    overflow_found = true;
                    break 'rotations;
                }
            }
        }
    }
    assert!(
        overflow_found,
        "a finite TRS input should overflow the matrix"
    );
}

/// Covers GLB JSON syntax errors and maps each importer stage error.
#[test]
fn importer_maps_invalid_documents_and_preserves_valid_values() {
    let valid = import(&valid_document()).unwrap();
    assert_eq!(valid.bodies.len(), 2);
    assert_eq!(valid.joints[0].child, 1);
    let invalid_json = glb(b"{]", &[]);
    assert!(matches!(
        from_glb(&invalid_json),
        Err(GltfRigError::Json(_))
    ));
    let mut invalid_header = glb(b"{}", &[]);
    invalid_header[0] = b'X';
    assert!(matches!(
        from_glb(&invalid_header),
        Err(GltfRigError::Invalid(_))
    ));

    let mut wrong_version = valid_document();
    wrong_version["asset"]["version"] = json!("1.0");
    assert!(diagnostic(&wrong_version).contains("version"));
    let mut invalid_nodes = valid_document();
    invalid_nodes["nodes"] = json!(false);
    assert!(diagnostic(&invalid_nodes).contains("nodes member"));
    let mut invalid_skin = valid_document();
    invalid_skin["skins"] = json!([]);
    assert!(diagnostic(&invalid_skin).contains("skins"));
    let mut invalid_body = valid_document();
    invalid_body["nodes"][1]["mesh"] = json!(99);
    assert!(diagnostic(&invalid_body).contains("index is missing or out of range"));
}

/// Rejects a body child index that cannot fit the serialized profile index.
#[test]
fn body_import_rejects_child_indices_above_u8() {
    let body_count = 257;
    let bone_nodes: Vec<Value> = (0..body_count)
        .map(|index| json!({"name": format!("bone_{index}")}))
        .collect();
    let joint = json!({
        "limit_x": {"min_deg": -90.0, "max_deg": 90.0},
        "limit_y": {"min_deg": 0.0, "max_deg": 0.0},
        "limit_z": {"min_deg": 0.0, "max_deg": 0.0},
        "torque_nm": 12.0
    });
    let capsule_nodes = (0..body_count)
        .map(|index| {
            capsule_node(
                &format!("capsule_{index}"),
                (index != 0).then(|| joint.clone()),
            )
        })
        .collect::<Vec<_>>();
    let mut node_values = bone_nodes;
    node_values.extend(capsule_nodes);
    let node_values = Value::Array(node_values);
    let nodes = node_values.as_array().unwrap();
    let mut parents = vec![None; body_count * 2];
    for (index, parent) in parents.iter_mut().enumerate().take(body_count).skip(1) {
        *parent = Some(index - 1);
    }
    for (index, parent) in parents.iter_mut().enumerate().skip(body_count) {
        *parent = Some(index - body_count);
    }
    let tree = NodeTree {
        nodes,
        parents,
        world: vec![Mat4::IDENTITY; body_count * 2],
    };
    let bones = (0..body_count)
        .map(|index| Bone {
            name: format!("bone_{index}"),
            parent: index.checked_sub(1),
            rest: Isometry3d::IDENTITY,
            rest_matrix: Mat4::IDENTITY,
        })
        .collect::<Vec<_>>();
    let mut bone_for_node = vec![None; body_count * 2];
    for (index, bone) in bone_for_node.iter_mut().enumerate().take(body_count) {
        *bone = Some(index);
    }
    let document = json!({
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "accessors": [{"min": [-0.5, -0.1, -0.1], "max": [0.5, 0.1, 0.1]}]
    });
    let problem = read_bodies(&document, &tree, &bones, &bone_for_node)
        .expect_err("the 257th body cannot fit the public body index");
    assert!(problem.to_string().contains("body child index exceeds 255"));
}
