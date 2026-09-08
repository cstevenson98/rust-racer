mod free_nbody;
mod knife_edge;

pub use free_nbody::{
    FreeNBody, FreeNBody15, FreeNBody15System, FreeNBodyCoords, FREE_NBODY_N,
};
pub use knife_edge::KnifeEdge;

/// Configuration coordinates for Euler integration: `q' = q + q̇·dt`.
pub trait CoordSpace: Copy + Send + Sync + 'static {
    fn integrate(q: Self, qdot: Self, dt: f32) -> Self;

    /// Euclidean inner product, used to project onto velocity constraints.
    fn dot(a: Self, b: Self) -> f32;
}

impl CoordSpace for f32 {
    fn integrate(q: Self, qdot: Self, dt: f32) -> Self {
        q + qdot * dt
    }

    fn dot(a: Self, b: Self) -> f32 {
        a * b
    }
}

impl CoordSpace for glam::Vec2 {
    fn integrate(q: Self, qdot: Self, dt: f32) -> Self {
        q + qdot * dt
    }

    fn dot(a: Self, b: Self) -> f32 {
        a.dot(b)
    }
}

impl CoordSpace for glam::Vec3 {
    fn integrate(q: Self, qdot: Self, dt: f32) -> Self {
        q + qdot * dt
    }

    fn dot(a: Self, b: Self) -> f32 {
        a.dot(b)
    }
}

/// Bounce `q` into the unit box `[0, 1]^n`, flipping velocity on contact.
pub trait UnitBoxBound: CoordSpace {
    fn enforce_unit_box(q: &mut Self, qdot: &mut Self);
}

impl UnitBoxBound for f32 {
    fn enforce_unit_box(q: &mut Self, qdot: &mut Self) {
        if *q < 0.0 || *q > 1.0 {
            *qdot = -*qdot;
            *q = q.clamp(0.0, 1.0);
        }
    }
}

impl UnitBoxBound for glam::Vec2 {
    fn enforce_unit_box(q: &mut Self, qdot: &mut Self) {
        if q.x < 0.0 || q.x > 1.0 {
            qdot.x = -qdot.x;
            q.x = q.x.clamp(0.0, 1.0);
        }
        if q.y < 0.0 || q.y > 1.0 {
            qdot.y = -qdot.y;
            q.y = q.y.clamp(0.0, 1.0);
        }
    }
}

impl UnitBoxBound for glam::Vec3 {
    fn enforce_unit_box(q: &mut Self, qdot: &mut Self) {
        if q.x < 0.0 || q.x > 1.0 {
            qdot.x = -qdot.x;
            q.x = q.x.clamp(0.0, 1.0);
        }
        if q.y < 0.0 || q.y > 1.0 {
            qdot.y = -qdot.y;
            q.y = q.y.clamp(0.0, 1.0);
        }
        if q.z < 0.0 || q.z > 1.0 {
            qdot.z = -qdot.z;
            q.z = q.z.clamp(0.0, 1.0);
        }
    }
}

/// Optional domain constraint applied after each Euler step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DomainConstraint {
    #[default]
    None,
    /// Keep `q` inside the unit box `[0, 1]^n` (component-wise bounce).
    UnitBox,
}

pub struct LagrangianState<C> {
    pub q: C,
    pub qdot: C,
}

pub struct HamiltonianState<C> {
    pub q: C,
    pub p: C,
}

pub trait LagrangianModel {
    type Coords: CoordSpace;

    fn accel(&mut self, q: Self::Coords, qdot: Self::Coords) -> Self::Coords;

    /// Legendre map: \(p = \partial L / \partial \dot q\).
    fn momentum(&mut self, q: Self::Coords, qdot: Self::Coords) -> Self::Coords;

    /// Non-holonomic constraint row `a(q)`, asserting `a(q) · q̇ = 0`.
    ///
    /// Written in whatever generalised coordinates the model uses; the
    /// integrator only needs the row, not its meaning. `None` means the model
    /// is unconstrained.
    fn constraint_row(&mut self, _q: Self::Coords) -> Option<Self::Coords> {
        None
    }
}

pub trait HamiltonianModel {
    type Coords: CoordSpace;

    fn dh_dq(&mut self, q: Self::Coords, p: Self::Coords) -> Self::Coords;
    fn dh_dp(&mut self, q: Self::Coords, p: Self::Coords) -> Self::Coords;

    /// \(\dot q = \partial H / \partial p\) (defaults to `dh_dp`).
    fn velocity(&mut self, q: Self::Coords, p: Self::Coords) -> Self::Coords {
        self.dh_dp(q, p)
    }
}

/// Free particle in 2D: L = T = ½|q̇|² (m = 1), so q̈ = 0 and p = q̇.
pub struct FreeParticle;

impl LagrangianModel for FreeParticle {
    type Coords = glam::Vec2;

    fn accel(&mut self, _q: glam::Vec2, _qdot: glam::Vec2) -> glam::Vec2 {
        glam::Vec2::ZERO
    }

    fn momentum(&mut self, _q: glam::Vec2, qdot: glam::Vec2) -> glam::Vec2 {
        qdot
    }
}

impl HamiltonianModel for FreeParticle {
    type Coords = glam::Vec2;

    fn dh_dq(&mut self, _q: glam::Vec2, _p: glam::Vec2) -> glam::Vec2 {
        glam::Vec2::ZERO
    }

    fn dh_dp(&mut self, _q: glam::Vec2, p: glam::Vec2) -> glam::Vec2 {
        p
    }
}

/// Attractive 1/r potential: V = -k / |q − center| (m = 1).
pub struct InverseRPotential {
    pub center: glam::Vec2,
    pub k: f32,
}

impl LagrangianModel for InverseRPotential {
    type Coords = glam::Vec2;

    fn accel(&mut self, q: glam::Vec2, _qdot: glam::Vec2) -> glam::Vec2 {
        let d = q - self.center;
        let r2 = d.length_squared().max(1e-8);
        let r = r2.sqrt();
        -self.k * d / (r2 * r)
    }

    fn momentum(&mut self, _q: glam::Vec2, qdot: glam::Vec2) -> glam::Vec2 {
        qdot
    }
}

impl HamiltonianModel for InverseRPotential {
    type Coords = glam::Vec2;

    fn dh_dq(&mut self, q: glam::Vec2, _p: glam::Vec2) -> glam::Vec2 {
        let d = q - self.center;
        let r2 = d.length_squared().max(1e-8);
        let r = r2.sqrt();
        self.k * d / (r2 * r)
    }

    fn dh_dp(&mut self, _q: glam::Vec2, p: glam::Vec2) -> glam::Vec2 {
        p
    }
}

pub struct LagrangianSystem<M: LagrangianModel> {
    pub state: LagrangianState<M::Coords>,
    pub model: M,
    pub constraint: DomainConstraint,
}

impl<M: LagrangianModel> LagrangianSystem<M>
where
    M::Coords: UnitBoxBound,
{
    pub fn q(&self) -> M::Coords {
        self.state.q
    }

    pub fn qdot(&self) -> M::Coords {
        self.state.qdot
    }

    /// Canonical momentum via \(p = \partial L / \partial \dot q\).
    pub fn p(&mut self) -> M::Coords {
        let q = self.state.q;
        let qdot = self.state.qdot;
        self.model.momentum(q, qdot)
    }

    pub fn update_euler(&mut self, dt: f32) {
        let q = self.state.q;
        let qdot = self.state.qdot;
        let qddot = self.model.accel(q, qdot);

        let q_next = M::Coords::integrate(q, qdot, dt);
        let mut qdot_next = M::Coords::integrate(qdot, qddot, dt);

        // Non-holonomic constraint: remove the velocity component along a(q).
        if let Some(a) = self.model.constraint_row(q_next) {
            let aa = M::Coords::dot(a, a);
            if aa > 1e-12 {
                let excess = M::Coords::dot(a, qdot_next) / aa;
                qdot_next = M::Coords::integrate(qdot_next, a, -excess);
            }
        }

        self.state.q = q_next;
        self.state.qdot = qdot_next;

        if self.constraint == DomainConstraint::UnitBox {
            M::Coords::enforce_unit_box(&mut self.state.q, &mut self.state.qdot);
        }
    }
}

impl LagrangianSystem<FreeParticle> {
    pub fn free_particle_2d(
        q: glam::Vec2,
        qdot: glam::Vec2,
        constraint: DomainConstraint,
    ) -> Self {
        Self {
            state: LagrangianState { q, qdot },
            model: FreeParticle,
            constraint,
        }
    }
}

impl LagrangianSystem<InverseRPotential> {
    pub fn inverse_r_2d(
        q: glam::Vec2,
        qdot: glam::Vec2,
        center: glam::Vec2,
        k: f32,
        constraint: DomainConstraint,
    ) -> Self {
        Self {
            state: LagrangianState { q, qdot },
            model: InverseRPotential { center, k },
            constraint,
        }
    }
}

pub struct HamiltonianSystem<M: HamiltonianModel> {
    pub state: HamiltonianState<M::Coords>,
    pub model: M,
    pub constraint: DomainConstraint,
}

impl<M: HamiltonianModel> HamiltonianSystem<M>
where
    M::Coords: UnitBoxBound,
{
    pub fn q(&self) -> M::Coords {
        self.state.q
    }

    pub fn p(&self) -> M::Coords {
        self.state.p
    }

    /// Configuration velocity via \(\dot q = \partial H / \partial p\).
    pub fn qdot(&mut self) -> M::Coords {
        let q = self.state.q;
        let p = self.state.p;
        self.model.velocity(q, p)
    }

    pub fn update_euler(&mut self, dt: f32) {
        let q = self.state.q;
        let p = self.state.p;
        let qdot = self.model.velocity(q, p);
        let dp = self.model.dh_dq(q, p);
        self.state.q = M::Coords::integrate(q, qdot, dt);
        self.state.p = M::Coords::integrate(p, dp, -dt);

        if self.constraint == DomainConstraint::UnitBox {
            let mut q = self.state.q;
            let mut v = qdot;
            M::Coords::enforce_unit_box(&mut q, &mut v);
            self.state.q = q;
        }
    }
}
