//! GLB 2.0 import support for building validated ragdoll profiles.
//!
//! Enable the `gltf` feature to read skeletal nodes, rest transforms, capsule
//! bounds, and Skein extras from a binary glTF file. The importer returns
//! authoring data through [`crate::ProfileSpec::from_glb`] and exposes typed
//! import failures through [`GltfRigError`]. Profile validation remains a
//! separate step so callers can inspect or edit imported authoring data before
//! constructing a runtime-ready [`crate::RagdollProfile`].

mod rig;

pub use self::rig::{GltfRigError, GltfRigValidationError};
