use bevy::prelude::*;
use mechanical_system::{FreeParticle, InverseRPotential, LagrangianModel, LagrangianSystem};

use crate::Spline;

/// Marker for a track control point (editable handle).
#[derive(Component)]
pub struct ControlPoint;

/// Parameter position of something moving along the track (`s` in segment space for now).
#[derive(Component, Default, Copy, Clone)]
pub struct TrackParam {
    pub s: f32,
}

/// ECS handle for a concrete Lagrangian system (`M` picks which array/query).
#[derive(Component)]
pub struct LagrangianBody<M>(pub LagrangianSystem<M>)
where
    M: LagrangianModel + Send + Sync + 'static;

/// Cached multi-segment spline rebuilt from control-point transforms.
#[derive(Resource, Default)]
pub struct ActiveSpline(pub Spline);

pub type FreeParticleBody = LagrangianBody<FreeParticle>;
pub type InverseRBody = LagrangianBody<InverseRPotential>;
