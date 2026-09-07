use bevy::prelude::*;
use mechanical_system::{DomainConstraint, FreeNBody15System, FREE_NBODY_N};

const UNIT_TO_SCREEN: f32 = 400.0;
const DOT_RADIUS: f32 = 5.0;

/// Compile-time free N-body sim (15 non-interacting 2D particles).
#[derive(Resource)]
pub struct FreeNBodySim(pub FreeNBody15System);

impl Default for FreeNBodySim {
    fn default() -> Self {
        Self(FreeNBody15System::free_nbody_random(DomainConstraint::UnitBox))
    }
}

pub(crate) fn register_free_nbody(app: &mut App) {
    app.init_resource::<FreeNBodySim>()
        .add_systems(Update, (update_free_nbody, draw_free_nbody));
}

fn update_free_nbody(time: Res<Time>, mut sim: ResMut<FreeNBodySim>) {
    sim.0.update_euler(time.delta_secs());
}

fn draw_free_nbody(sim: Res<FreeNBodySim>, mut gizmos: Gizmos) {
    for i in 0..FREE_NBODY_N {
        let q = sim.0.particle_pos(i);
        let p = Vec2::new((q.x - 0.5) * UNIT_TO_SCREEN, (q.y - 0.5) * UNIT_TO_SCREEN);
        gizmos.circle_2d(Isometry2d::from_translation(p), DOT_RADIUS, Color::srgb(0.4, 0.75, 1.0));
    }
}
