mod components;
mod spline;

pub use components::{ActiveSpline, ControlPoint, TrackParam};
pub use spline::{weighted_sum, CubicSplineSegment, Spline};

use bevy::prelude::*;

/// Registers shared track resources. No simulation systems.
pub struct TrackCorePlugin;

impl Plugin for TrackCorePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveSpline>();
    }
}
