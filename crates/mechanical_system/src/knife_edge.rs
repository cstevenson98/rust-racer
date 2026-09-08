//! Knife edge (skate / rolling wheel): the minimal non-holonomic system.
//!
//! Generalised coordinates are `q = (x, y, θ)`, packed into a `Vec3`. The body
//! may drive along its heading and rotate freely, but it can never slip
//! sideways, which is the non-integrable velocity constraint
//!
//! ```text
//! (−sin θ, cos θ, 0) · q̇ = 0
//! ```
//!
//! No choice of coordinates removes that constraint, so the integrator enforces
//! it every step by projecting `q̇` onto its null space.

use glam::{Vec2, Vec3};
use rand::Rng;

use crate::{DomainConstraint, LagrangianModel, LagrangianState, LagrangianSystem};

pub struct KnifeEdge {
    /// Acceleration along the current heading.
    pub drive: f32,
    /// Angular acceleration (steering torque, m = 1).
    pub steer: f32,
}

impl LagrangianModel for KnifeEdge {
    type Coords = Vec3; // (x, y, θ)

    fn accel(&mut self, q: Vec3, _qdot: Vec3) -> Vec3 {
        let (sin, cos) = q.z.sin_cos();
        Vec3::new(self.drive * cos, self.drive * sin, self.steer)
    }

    fn momentum(&mut self, _q: Vec3, qdot: Vec3) -> Vec3 {
        qdot
    }

    fn constraint_row(&mut self, q: Vec3) -> Option<Vec3> {
        let (sin, cos) = q.z.sin_cos();
        Some(Vec3::new(-sin, cos, 0.0))
    }
}

impl LagrangianSystem<KnifeEdge> {
    pub fn knife_edge(
        pos: Vec2,
        heading: f32,
        speed: f32,
        turn_rate: f32,
        drive: f32,
        steer: f32,
    ) -> Self {
        let (sin, cos) = heading.sin_cos();
        Self {
            state: LagrangianState {
                q: Vec3::new(pos.x, pos.y, heading),
                qdot: Vec3::new(speed * cos, speed * sin, turn_rate),
            },
            model: KnifeEdge { drive, steer },
            constraint: DomainConstraint::None,
        }
    }

    /// Random heading, speed and turn rate, all starting from `pos`.
    ///
    /// With no drive or steering torque the turn rate is constant, so each one
    /// traces a circle of radius `speed / |turn_rate|`.
    pub fn knife_edge_random(pos: Vec2) -> Self {
        let mut rng = rand::rng();
        let turn_rate = rng.random_range(0.3..1.2) * if rng.random_bool(0.5) { 1.0 } else { -1.0 };
        Self::knife_edge(
            pos,
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(40.0..140.0),
            turn_rate,
            0.0,
            0.0,
        )
    }

    pub fn pos(&self) -> Vec2 {
        self.state.q.truncate()
    }

    pub fn heading(&self) -> f32 {
        self.state.q.z
    }

    /// Residual `a(q) · q̇`, which the projection should hold at zero.
    pub fn slip(&self) -> f32 {
        let (sin, cos) = self.state.q.z.sin_cos();
        Vec3::new(-sin, cos, 0.0).dot(self.state.qdot)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    #[test]
    fn constraint_holds_under_integration() {
        let mut sys = LagrangianSystem::knife_edge(Vec2::ZERO, 0.3, 100.0, 0.8, 0.0, 0.0);
        for _ in 0..2000 {
            sys.update_euler(DT);
            assert!(sys.slip().abs() < 1e-3, "slip = {}", sys.slip());
        }
    }

    #[test]
    fn sideways_velocity_is_removed() {
        let mut sys = LagrangianSystem::knife_edge(Vec2::ZERO, 0.0, 0.0, 0.0, 0.0, 0.0);
        sys.state.qdot = Vec3::new(0.0, 50.0, 0.0); // shove it broadside
        sys.update_euler(DT);
        assert!(sys.state.qdot.y.abs() < 1e-4, "qdot = {}", sys.state.qdot);
    }

    #[test]
    fn constant_turn_rate_traces_a_circle() {
        let (speed, turn_rate) = (100.0, 1.0);
        let mut sys = LagrangianSystem::knife_edge(Vec2::ZERO, 0.0, speed, turn_rate, 0.0, 0.0);
        let radius = speed / turn_rate;
        let center = Vec2::new(0.0, radius);

        for _ in 0..600 {
            sys.update_euler(DT);
            let r = (sys.pos() - center).length();
            assert!((r - radius).abs() < 0.1 * radius, "radius drifted to {r}");
        }
    }
}
