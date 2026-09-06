use bevy::prelude::*;

use crate::Spline;

/// Marker for a track control point (editable handle).
#[derive(Component)]
pub struct ControlPoint;

/// Parameter position of something moving along the track (`s` in segment space for now).
#[derive(Component, Default, Copy, Clone)]
pub struct TrackParam {
    pub s: f32,
}

/// Cached multi-segment spline rebuilt from control-point transforms.
#[derive(Resource, Default)]
pub struct ActiveSpline(pub Spline);
