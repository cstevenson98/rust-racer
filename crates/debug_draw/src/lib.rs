use bevy::prelude::*;
use track_core::{ActiveSpline, ControlPoint};

const RING_RADIUS: f32 = 12.0;
const CROSS_HALF: f32 = 10.0;
const SPLINE_SAMPLES: u32 = 64;

pub struct DebugDrawPlugin;

impl Plugin for DebugDrawPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugDraw>()
            .add_systems(Update, (draw_point_overlays, draw_spline));
    }
}

/// When enabled, draws control-point overlays and the active spline as gizmos.
#[derive(Resource, Clone, Copy)]
pub struct DebugDraw(pub bool);

impl Default for DebugDraw {
    fn default() -> Self {
        Self(true)
    }
}

fn draw_point_overlays(
    debug: Res<DebugDraw>,
    mut gizmos: Gizmos,
    points: Query<&Transform, With<ControlPoint>>,
) {
    if !debug.0 {
        return;
    }
    for tf in &points {
        let iso = Isometry2d::from_translation(tf.translation.truncate());
        gizmos.circle_2d(iso, RING_RADIUS, Color::BLACK);
        gizmos.cross_2d(iso, CROSS_HALF, Color::BLACK);
    }
}

fn draw_spline(debug: Res<DebugDraw>, mut gizmos: Gizmos, spline: Res<ActiveSpline>) {
    if !debug.0 || spline.0.segments.is_empty() {
        return;
    }
    let samples = spline.0.evaluate_range(SPLINE_SAMPLES);
    gizmos.linestrip_2d(samples, Color::srgb(0.85, 0.35, 0.2));
}
