use bevy::prelude::*;
use mechanical_system::{DomainConstraint, LagrangianSystem};
use track_core::LagrangianBody;

const DOT_RADIUS: f32 = 8.0;

/// Number of non-holonomic skates spawned at the origin.
const KNIFE_EDGE_COUNT: usize = 12;
const KNIFE_EDGE_LEN: f32 = 16.0;
const KNIFE_EDGE_WIDTH: f32 = 3.0;

/// Player input and on-track physics (placeholder).
pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (spawn_inverse_r_particle, spawn_knife_edges));
    }
}

/// A fan of knife edges, all released from the origin with random headings and
/// turn rates. None of them can ever slide sideways.
fn spawn_knife_edges(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let blade = meshes.add(Rectangle::new(KNIFE_EDGE_LEN, KNIFE_EDGE_WIDTH));

    for i in 0..KNIFE_EDGE_COUNT {
        let hue = 360.0 * i as f32 / KNIFE_EDGE_COUNT as f32;
        commands.spawn((
            LagrangianBody(LagrangianSystem::knife_edge_random(Vec2::ZERO)),
            Mesh2d(blade.clone()),
            MeshMaterial2d(materials.add(Color::hsl(hue, 0.8, 0.65))),
            Transform::from_xyz(0.0, 0.0, 2.0),
        ));
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
