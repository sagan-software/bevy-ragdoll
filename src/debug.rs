//! Debug meshes that show each physics body's collision shape.

use bevy::prelude::{
    Added, App, Assets, ChildOf, Color, Commands, Entity, IntoScheduleConfigs, Mesh, Mesh3d,
    MeshMaterial3d, Plugin, PostUpdate, Query, ResMut, StandardMaterial, Visibility,
};
use bevy::transform::TransformSystems;

use crate::profile::BodyIndex;
use crate::runtime::body::BodyShape;
use crate::runtime::sets::RagdollSystems;

/// Adds a colored mesh matching its collision shape to every new body.
///
/// Add it next to [`crate::RagdollPlugin`] to see generated bodies. Each body
/// gets a child mesh in one of three colors chosen by its body index.
#[derive(Clone, Copy, Debug, Default)]
pub struct RagdollDebugPlugin;

impl Plugin for RagdollDebugPlugin {
    fn build(&self, app: &mut App) {
        // Run right after bodies spawn so their meshes render on the same frame.
        app.add_systems(
            PostUpdate,
            add_body_meshes
                .after(RagdollSystems::Bind)
                .before(TransformSystems::Propagate),
        );
    }
}

/// Spawns one shape mesh as a child of each newly created body entity.
#[expect(
    clippy::indexing_slicing,
    reason = "the index is reduced modulo the palette length"
)]
fn add_body_meshes(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    bodies: Query<'_, '_, (Entity, &BodyShape, &BodyIndex), Added<BodyShape>>,
) {
    const PALETTE: [Color; 3] = [
        Color::srgb(0.18, 0.62, 0.76),
        Color::srgb(0.25, 0.78, 0.64),
        Color::srgb(0.92, 0.66, 0.34),
    ];
    for (body, shape, index) in &bodies {
        let (mesh, transform) = shape.0.mesh();
        commands.entity(body).insert(Visibility::Inherited);
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(material(PALETTE[index.get() % PALETTE.len()]))),
            transform,
            ChildOf(body),
        ));
    }
}

/// Returns the slightly glossy opaque material used for body meshes.
fn material(color: Color) -> StandardMaterial {
    let mut material = StandardMaterial::from(color);
    material.metallic = 0.02;
    material.perceptual_roughness = 0.42;
    material
}
