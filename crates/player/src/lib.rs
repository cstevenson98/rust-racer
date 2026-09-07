use bevy::prelude::*;
use mechanical_system::{DomainConstraint, LagrangianSystem};
use track_core::LagrangianBody;

const DOT_RADIUS: f32 = 8.0;

/// Player input and on-track physics (placeholder).
pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_inverse_r_particle);
    }
}

fn spawn_inverse_r_particle(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let center = Vec2::new(0.5, 0.5);
    let q = Vec2::new(0.72, 0.5);
    let radial = (q - center).normalize_or_zero();
    let tangent = Vec2::new(-radial.y, radial.x);
    let qdot = tangent * 0.22;

    commands.spawn((
        LagrangianBody(LagrangianSystem::inverse_r_2d(
            q,
            qdot,
            center,
            0.08,
            DomainConstraint::None,
        )),
        Mesh2d(meshes.add(Circle::new(DOT_RADIUS))),
        MeshMaterial2d(materials.add(Color::srgb(0.9, 0.85, 0.2))),
        Transform::from_xyz((q.x - 0.5) * 400.0, (q.y - 0.5) * 400.0, 1.0),
    ));
}
