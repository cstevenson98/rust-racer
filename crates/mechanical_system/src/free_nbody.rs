//! Compile-time N-body free particles in 2D (no interactions).
//!
//! Configuration is an `SMatrix<f32, 2, N>`: each column is one particle's `(x, y)`.

use nalgebra::{SMatrix, SVector};
use rand::Rng;

use crate::{
    CoordSpace, DomainConstraint, LagrangianModel, LagrangianState, LagrangianSystem, UnitBoxBound,
};

/// Column-major particle state: column `i` is particle `i`'s 2D coordinates.
pub type FreeNBodyCoords<const N: usize> = SMatrix<f32, 2, N>;

impl<const N: usize> CoordSpace for FreeNBodyCoords<N> {
    fn integrate(q: Self, qdot: Self, dt: f32) -> Self {
        q + qdot * dt
    }
}

impl<const N: usize> UnitBoxBound for FreeNBodyCoords<N> {
    fn enforce_unit_box(q: &mut Self, qdot: &mut Self) {
        for i in 0..N {
            let mut p = q.column(i).into_owned();
            let mut v = qdot.column(i).into_owned();
            for k in 0..2 {
                if p[k] < 0.0 || p[k] > 1.0 {
                    v[k] = -v[k];
                    p[k] = p[k].clamp(0.0, 1.0);
                }
            }
            q.set_column(i, &p);
            qdot.set_column(i, &v);
        }
    }
}

/// N non-interacting free particles (m = 1): L = Σ ½|q̇ᵢ|², so q̈ = 0 and p = q̇.
pub struct FreeNBody<const N: usize>;

impl<const N: usize> LagrangianModel for FreeNBody<N> {
    type Coords = FreeNBodyCoords<N>;

    fn accel(&mut self, _q: Self::Coords, _qdot: Self::Coords) -> Self::Coords {
        Self::Coords::zeros()
    }

    fn momentum(&mut self, _q: Self::Coords, qdot: Self::Coords) -> Self::Coords {
        qdot
    }
}

impl<const N: usize> LagrangianSystem<FreeNBody<N>> {
    /// Random positions in `(0, 1)²` and small random velocities.
    pub fn free_nbody_random(constraint: DomainConstraint) -> Self {
        let mut rng = rand::rng();
        let mut q = FreeNBodyCoords::<N>::zeros();
        let mut qdot = FreeNBodyCoords::<N>::zeros();
        for i in 0..N {
            let pos = SVector::<f32, 2>::new(rng.random_range(0.05..0.95), rng.random_range(0.05..0.95));
            let vel = SVector::<f32, 2>::new(rng.random_range(-0.25..0.25), rng.random_range(-0.25..0.25));
            q.set_column(i, &pos);
            qdot.set_column(i, &vel);
        }
        Self {
            state: LagrangianState { q, qdot },
            model: FreeNBody,
            constraint,
        }
    }

    pub fn particle_pos(&self, i: usize) -> glam::Vec2 {
        let c = self.state.q.column(i);
        glam::Vec2::new(c[0], c[1])
    }
}

/// Default compile-time flock size used by the game.
pub const FREE_NBODY_N: usize = 15;

pub type FreeNBody15 = FreeNBody<FREE_NBODY_N>;
pub type FreeNBody15System = LagrangianSystem<FreeNBody15>;
