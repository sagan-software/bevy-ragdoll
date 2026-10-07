//! Debug meshes that show each physics body's collision shape.

use bevy::prelude::{
    Added, App, Assets, ChildOf, Color, Commands, Entity, Mesh, Mesh3d, MeshMaterial3d, Plugin,
    Query, ResMut, StandardMaterial, Update, Visibility,
};

use crate::profile::BodyIndex;
use crate::runtime::body::BodyShape;

/// Adds a colored mesh matching its collision shape to every new body.
pub struct RagdollDebugPlugin;

impl Plugin for RagdollDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, add_body_meshes);
    }
}

/// Spawns one shape mesh as a child of each newly created body entity.
fn add_body_meshes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    bodies: Query<(Entity, &BodyShape, &BodyIndex), Added<BodyShape>>,
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
            MeshMaterial3d(materials.add(PALETTE[index.get() % PALETTE.len()])),
            transform,
            ChildOf(body),
        ));
    }
}
