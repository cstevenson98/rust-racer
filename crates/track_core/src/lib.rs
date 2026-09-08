mod components;
mod free_nbody_sim;
mod spline;

pub use components::{
    ActiveSpline, ControlPoint, FreeParticleBody, InverseRBody, KnifeEdgeBody, LagrangianBody,
    TrackParam,
};
pub use free_nbody_sim::FreeNBodySim;
pub use spline::{weighted_sum, CubicSplineSegment, Spline};

use bevy::prelude::*;

/// Registers shared track resources and on-track mechanical updates.
pub struct TrackCorePlugin;

impl Plugin for TrackCorePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveSpline>()
            .add_systems(Update, (update_inverse_r_2d, update_knife_edges));
        free_nbody_sim::register_free_nbody(app);
    }
}

/// Maps unit-box coordinates `[0, 1]^2` to a screen span centered at the origin.
const UNIT_TO_SCREEN: f32 = 400.0;

fn update_inverse_r_2d(
    time: Res<Time>,
    mut bodies: Query<(&mut InverseRBody, Option<&mut Transform>)>,
) {
    let dt = time.delta_secs();
    for (mut body, transform) in &mut bodies {
        body.0.update_euler(dt);
        if let Some(mut transform) = transform {
            let q = body.0.q();
            transform.translation.x = (q.x - 0.5) * UNIT_TO_SCREEN;
            transform.translation.y = (q.y - 0.5) * UNIT_TO_SCREEN;
        }
    }
}

/// Knife edges live directly in screen coordinates, with `θ` driving rotation.
fn update_knife_edges(time: Res<Time>, mut bodies: Query<(&mut KnifeEdgeBody, &mut Transform)>) {
    let dt = time.delta_secs();
    for (mut body, mut transform) in &mut bodies {
        body.0.update_euler(dt);
        let pos = body.0.pos();
        transform.translation.x = pos.x;
        transform.translation.y = pos.y;
        transform.rotation = Quat::from_rotation_z(body.0.heading());
    }
}
