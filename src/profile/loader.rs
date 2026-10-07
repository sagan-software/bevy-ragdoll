//! Bevy asset loading for validated RON ragdoll profiles.

use std::io;

use bevy::asset::{AssetLoader, LoadContext, io::Reader};
use bevy::reflect::TypePath;

use super::{ProfileSpec, RagdollProfile};

/// A Bevy asset loader for RON ragdoll profiles using the registered
/// `.ragdoll.ron` extension. It rejects malformed authoring data and profiles
/// that fail body, joint, shape, or tree validation before creating an asset.
#[derive(Clone, Copy, Debug, Default, TypePath)]
pub struct RagdollProfileLoader;

/// A RON profile loading or validation failure with its original source error.
///
/// The variants separate reader failures, RON syntax failures, and profile
/// invariant failures so callers can report or retry the correct asset stage.
#[derive(Debug, thiserror::Error)]
pub enum RagdollProfileLoaderError {
    /// The Bevy asset reader failed before all source bytes could be obtained;
    /// the original I/O error remains available through the error source chain.
    #[error("could not read the ragdoll profile: {0}")]
    Read(#[from] io::Error),
    /// The input bytes are not a syntactically valid RON profile document; the
    /// parser retains its line and column location for asset diagnostics.
    #[error("could not parse the ragdoll profile RON: {0}")]
    Ron(#[from] ron::error::SpannedError),
    /// The parsed profile spec violates a body, joint, geometry, mass, or tree
    /// invariant required before Bevy receives the validated profile asset.
    #[error(transparent)]
    Profile(#[from] super::ProfileError),
}

impl AssetLoader for RagdollProfileLoader {
    type Asset = RagdollProfile;
    type Settings = ();
    type Error = RagdollProfileLoaderError;

    /// Parses RON profile data and validates it before creating the asset.
    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let spec: ProfileSpec = ron::de::from_bytes(&bytes)?;
        Ok(RagdollProfile::new(spec)?)
    }

    /// Matches profile assets ending in `.ragdoll.ron`.
    fn extensions(&self) -> &[&str] {
        &["ragdoll.ron"]
    }
}
