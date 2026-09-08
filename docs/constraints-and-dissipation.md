# Constraints and Dissipation — A Working Note

Companion to `hierarchical-multiphysics-primer.md`, narrowed to two things the current code doesn't yet have a real story for: **constraints** (especially non-holonomic ones) and **damping**. Worked examples are the dragged mass / tractrix and the Chaplygin sleigh.

---

## 1. Taxonomy first, because the words matter

| Kind | Form | Effect | Example |
|------|------|--------|---------|
| **Holonomic** | `g(q, t) = 0` | removes a *configuration* DOF | rigid rod, pulley rope, gear ratio |
| **Non-holonomic** | `A(q) q̇ + b(q,t) = 0`, non-integrable | removes a *velocity* DOF, keeps configuration reachability | rolling wheel, skate, trailer |
| **Unilateral** | `g(q) ≥ 0` | active only on contact; impulsive | wall, floor, collision |
| **Rheonomic** | any of the above with explicit `t` | injects energy | prescribed handle motion, motor |

The distinction that trips everyone up is **holonomic vs non-holonomic**. A holonomic constraint shrinks *where you can be*. A non-holonomic one shrinks *how you can move* but not where you can end up: a car can reach any parking spot, it just can't slide sideways to get there. Formally, `A(q) q̇ = 0` is non-holonomic exactly when it is **not** the time derivative of any `g(q) = 0` — Frobenius' theorem is the test, but in practice you know from the physics (rolling and knife-edges are non-holonomic; rods, ropes and gears are holonomic).

The practical consequence for a solver: holonomic constraints can usually be eliminated by choosing better coordinates, so they cost you nothing at runtime. Non-holonomic ones **cannot** — no choice of coordinates removes them, so they must be enforced every step. That's why they need machinery and holonomic ones often don't.

---

## 2. Audit: what's actually in the code today

Constraints currently exist as exactly one thing:

```rust
pub enum DomainConstraint { None, UnitBox }

pub trait UnitBoxBound: CoordSpace {
    fn enforce_unit_box(q: &mut Self, qdot: &mut Self);
}
```

applied *after* the Euler step. In the taxonomy above this is a **unilateral** constraint (`0 ≤ q ≤ 1`) handled by a **position-level projection with velocity reflection** — a perfectly respectable technique, it just isn't the same category as the rod/rope/rolling constraints you'll want next. Some honest observations:

**It clamps where it should reflect.** `q.clamp(0.0, 1.0)` throws away the overshoot. A particle that crosses the wall by `δ` should end the step at `bound − δ` (it travelled the same distance, in the other direction), not at `bound`. As written, each bounce quietly loses a little energy and, for glancing hits, a little path length:

```text
clamp:   q ← 1.0          (loses δ)
reflect: q ← 2·1.0 − q    (keeps δ)
```

**There's no restitution.** `qdot ← −qdot` is perfectly elastic. A coefficient `e ∈ [0,1]` (`qdot ← −e·qdot`) is one character of work and gives you inelastic walls. Worth noting that this is a *dissipation* mechanism, but an impulsive, non-smooth one — it is **not** expressible as a Rayleigh function (§7).

**The Hamiltonian path doesn't actually bounce.** In `HamiltonianSystem::update_euler`:

```rust
let mut v = qdot;
M::Coords::enforce_unit_box(&mut q, &mut v);
self.state.q = q;   // v was reflected, then dropped — p is never flipped
```

`p` is left pointing into the wall, so a Hamiltonian particle clamps to the boundary and keeps pushing. For the current `dh_dp = p` models, reflecting `p` directly is correct; in general you want to reflect `p` through the metric (`p ← p − 2(p·n̂)n̂` for a flat wall with `M = I`).

**`UnitBoxBound` is a bound on the whole system, not just the constrained path.** Every method on `LagrangianSystem<M>` — including `q()` and `qdot()` — currently requires `M::Coords: UnitBoxBound`, so a coordinate type that has no sensible unit box can't use the system at all. When constraints become their own concept (§8), that bound should move to the constrained code path.

**`constraint` lives on the system, not the model.** For unilateral contact that's the *right* call — collisions genuinely sit outside the variational structure. But it means the current mechanism can't express holonomic or non-holonomic constraints, which need to know about `M(q)` and therefore belong either inside the model or in a projection layer that sees both.

None of this is urgent. It's worth writing down because the next constraints you add are a different species, and it would be easy to try to grow `DomainConstraint` into them.

---

## 3. Holonomic constraints: two honest routes

Given `g(q) = 0`:

**Route A — reduce.** Find a parametrisation `q = φ(r)` of the constraint surface and do unconstrained mechanics in `r`, with metric `M̃(r) = DφᵀM Dφ`. Exact, minimal state, no drift, no linear algebra. The pulley-seesaw in the primer is this route. Limitation: you need to *find* `φ`, which is easy for tree-like machines and hard for closed loops.

**Route B — multipliers.** Keep redundant `q`, add `λ`:

```text
M q̈ = F + Gᵀλ,     G = ∂g/∂q,     g(q) = 0
```

Differentiate the constraint twice and solve the saddle-point system for `λ`. General (handles loops), but you inherit **constraint drift**: `g(q)` wanders from zero, so you need stabilisation (Baumgarte, or better, an explicit projection of `q` and `q̇` back onto the manifold after each step).

Rule of thumb: reduce when you can, multipliers when the topology forces it.

---

## 4. Non-holonomic constraints: the mechanics

Write the constraint as `A(q) q̇ = 0` (`k` rows for `k` constraints). The correct principle is **Lagrange–d'Alembert**: constraint forces do no virtual work, so they act along `Aᵀ`:

```text
M(q) q̈ + ½M′[q̇,q̇] + ∂V/∂q = Q_ext + Aᵀ λ
A(q) q̇ = 0
```

Two warnings that cost people a lot of time:

**Lagrange–d'Alembert ≠ vakonomic.** If you instead do constrained variational optimisation (extremise the action over paths satisfying the constraint), you get *different* equations. Both are mathematically respectable; only d'Alembert describes rolling. Optimal-control texts often mean the vakonomic one, so check which you're reading.

**Non-holonomic systems are not Hamiltonian.** Energy is still conserved (constraint forces do no work), but there is no symplectic structure and Liouville's theorem fails. The practical upshot is spectacular: **a non-holonomic system can have asymptotically stable equilibria while conserving energy exactly** — see the sleigh in §6. Don't expect symplectic-integrator guarantees to carry over, and don't be alarmed when trajectories converge.

### The projection recipe (what you'd actually implement)

Differentiate the constraint: `A q̈ + Ȧ q̇ = 0`. Substitute `q̈ = M⁻¹(F + Aᵀλ)` and solve:

```text
λ  = −(A M⁻¹ Aᵀ)⁻¹ (A M⁻¹ F + Ȧ q̇)
q̈  = M⁻¹ (F + Aᵀ λ)
```

For a **single** constraint row and `M = m·I` this is scalar arithmetic — no linear algebra library needed:

```text
λ  = −(A·F + m (Ȧ·q̇)) / |A|²
q̈  = (F + λA) / m
```

Then, because floating point drifts, add a velocity-level projection each step (the exact analogue of `enforce_unit_box`, one level up):

```text
q̇ ← q̇ − Aᵀ (A Aᵀ)⁻¹ (A q̇)
```

This gives you a clean unifying picture of what "enforcing a constraint" means:

| Projected at | Fixes | Example |
|--------------|-------|---------|
| Position | `g(q) = 0` | unit box clamp, holonomic drift correction |
| Velocity | `A q̇ = 0` | non-holonomic drift, collision impulse |
| Acceleration | keeps `A q̇ = 0` smooth | the `λ` computation above |

Your current code does the first. Non-holonomic work needs the second and third.

---

## 5. Worked example: dragged mass and the tractrix

This is the best possible first non-holonomic example, because it needs **no dynamics at all** — no masses, no forces, no integrator subtleties. Just two constraints and a driven input.

```text
        h(t)  ← you drag this (handle / mouse cursor)
         ●
         │╲
         │ ╲  rigid rod, length ℓ
         │  ╲
         │   ●  x(t)  ← trailing mass
         │    ↘ can only move ALONG the rod
```

**Setup.** Handle `h(t)` (an input `u`), trailing point `x(t)`, rigid rod of length `ℓ`. Let `u = (h − x)/ℓ` be the unit vector from the mass toward the handle, and `n ⊥ u`.

Two constraints:

```text
holonomic:      |h − x| = ℓ            (rigid rod)
non-holonomic:  ẋ · n = 0              (no sideways slip — a knife edge)
```

**Solve.** Differentiate the rod constraint: `(x − h)·(ẋ − ḣ) = 0`, i.e. `u·ẋ = u·ḣ`. The non-holonomic condition says `ẋ = v u` for some scalar `v`. Substituting gives `v = u·ḣ`, so:

```text
ẋ = (u · ḣ) u
```

**The trailing mass moves at the projection of the handle's velocity onto the rod.** Drag sideways and it doesn't move at all; drag along the rod and it follows one-for-one.

Note the division of labour: the **non-holonomic** constraint supplies the *direction* of `ẋ`, the **holonomic** one supplies the *magnitude*. Together they leave zero degrees of freedom — this is a purely driven system, which is why it's so easy.

**Why it's the tractrix.** Drag the handle along the x-axis, `h(t) = (t, 0)`, starting with the mass at `(0, ℓ)`. Writing `x = (X, Y)` and using `t − X = √(ℓ² − Y²)`:

```text
Ẋ = (ℓ² − Y²)/ℓ²
Ẏ = −Y√(ℓ² − Y²)/ℓ²
dX/dY = −√(ℓ² − Y²)/Y
```

which integrates to the tractrix

```text
X = ℓ · ln[(ℓ + √(ℓ² − Y²)) / Y] − √(ℓ² − Y²)
```

with its defining property: **the tangent segment from the curve to the asymptote always has length `ℓ`** — which is just the rod, restated. (Flavour: revolve it about the asymptote and you get the pseudosphere, the constant-negative-curvature surface. Your shopping trolley traces hyperbolic geometry.)

**Implementing it exposes a gap in the current traits.** This model is **first order** — `ẋ = F(x, t)` — but `LagrangianModel` is second order (`accel`). You need a sibling trait:

```rust
pub trait FlowModel {
    type Coords: CoordSpace;
    /// ż = F(z, t)
    fn rate(&mut self, z: Self::Coords, t: f32) -> Self::Coords;
}
```

and the integrator is one line, because `CoordSpace::integrate(q, qdot, dt) = q + qdot·dt` is *already* a first-order Euler step:

```rust
self.state = C::integrate(self.state, self.model.rate(self.state, t), dt);
```

That small addition buys you kinematic constraints, overdamped dynamics (§8), and first-order fields like heat diffusion — all of which currently have nowhere to live.

```rust
pub struct DraggedMass {
    pub rod_len: f32,
    pub handle: glam::Vec2,      // input u, set each frame
    pub handle_vel: glam::Vec2,
}

impl FlowModel for DraggedMass {
    type Coords = glam::Vec2; // x

    fn rate(&mut self, x: glam::Vec2, _t: f32) -> glam::Vec2 {
        let u = (self.handle - x).normalize_or_zero();
        u * u.dot(self.handle_vel)
    }
}
```

**Make it a demo:** drive `handle` from the mouse cursor, integrate, draw the rod and the trace. It is genuinely satisfying, it's ~30 lines, and it's a correct non-holonomic system. Drift note: `|h − x|` will slowly wander from `ℓ` under Euler, so re-project each step (`x ← h − ℓ·u`) — that's your position-level projection from §4, in its simplest possible form.

**Extension — the trailer train.** Chain `n` rods, each trailing the previous joint. Same rule applied `n` times, and you get the classic jack-knifing truck. It's a one-loop generalisation and a nice granularity story: `n = 1` is a caster, `n = 3` is a lorry.

---

## 6. Worked example: the Chaplygin sleigh (dynamics + non-holonomic)

Now add mass and remove the handle. A rigid body slides on a plane on one **knife edge** (plus two frictionless supports). The blade cannot move sideways.

```text
        ╱ blade (knife edge, no lateral slip)
   ────●────────────●
     contact    a   CoM,  mass m, inertia J about CoM
```

Configuration `q = (x, y, θ)`, blade at distance `a` from the centre of mass along the body axis. The constraint is that the *contact point's* velocity has no component along `n(θ) = (−sin θ, cos θ)`.

Reducing to body-frame variables — forward speed `v` at the contact point, turn rate `ω = θ̇` — gives the standard reduced equations:

```text
v̇ = a ω²
ω̇ = −m a v ω / (J + m a²)
```

**Check energy.** With `T = ½m(v² + a²ω²) + ½Jω²`:

```text
dT/dt = m v v̇ + (m a² + J) ω ω̇
      = m a v ω²  −  m a v ω²  =  0
```

Energy is exactly conserved. **And yet**: linearise about steady straight motion `(v = v₀ > 0, ω = 0)` and the second equation reads `ω̇ ≈ −(m a v₀ / (J + m a²)) ω`, which decays exponentially. The sleigh spins down and settles into a straight line, permanently — while conserving energy the whole time.

This is the single clearest demonstration of why non-holonomic systems sit outside the Hamiltonian world. In a Hamiltonian system, Liouville's theorem forbids phase-space volume from contracting, so asymptotically stable equilibria are impossible. Here they're routine. (The energy isn't lost; it's transferred from rotation into translation, and there's no route back.)

It's also a genuinely good test case: two scalar ODEs, an exact conserved quantity to check your integrator against, and a qualitative behaviour (spin-down) that's obvious on screen.

---

## 7. Rayleigh dissipation: damping without leaving the formalism

Everything so far conserves energy. Real machines don't. The standard way to get damping while staying in the Lagrangian framework is a **Rayleigh dissipation function** `ℱ(q, q̇)`, which enters the equations as an extra term:

```text
d/dt (∂L/∂q̇) − ∂L/∂q + ∂ℱ/∂q̇ = Q_ext
```

So the damping generalised force is `Q_damp = −∂ℱ/∂q̇`. The canonical choice is quadratic in velocity:

```text
ℱ = ½ q̇ᵀ C(q) q̇       ⟹    Q_damp = −C q̇
```

with `C` symmetric positive semi-definite. The reason this is *the* right abstraction rather than "just subtract `c·q̇`" is the energy identity:

```text
dE/dt = −q̇ᵀ (∂ℱ/∂q̇) = −q̇ᵀ C q̇ = −2ℱ  ≤ 0
```

For a quadratic homogeneous `ℱ`, **the rate of energy loss is exactly `2ℱ`**. So `ℱ` isn't a fudge factor — it's a scalar you can log, plot, and use to audit whether your coupled machine is leaking power where it should be. `C ⪰ 0` is the precise statement of "this term can only remove energy".

### The useful cases

| Physics | `ℱ` | Force |
|---------|-----|-------|
| Viscous / linear drag | `½c|q̇|²` | `−c q̇` |
| Quadratic (aero) drag | `(c/3)|q̇|³` | `−c|q̇| q̇` |
| Joint/bearing friction | `½c θ̇²` | `−c θ̇` |
| Structural (FEM) | `½ q̇ᵀ(αM + βK)q̇` | `−(αM + βK) q̇` |
| Relative damping (dashpot between `i`,`j`) | `½c(q̇ᵢ − q̇ⱼ)²` | equal and opposite |

That last row is the one that matters for coupled machines: writing `ℱ` in terms of *relative* velocities makes the damping forces automatically obey Newton's third law and vanish under rigid translation. Getting that wrong by hand is a classic source of machines that mysteriously self-propel.

### What Rayleigh can't do

**Coulomb (dry) friction is not a Rayleigh function.** The force `−μN·sign(q̇)` would need `ℱ = μN|q̇|`, which isn't differentiable at `q̇ = 0` — and that non-differentiability *is* the physics (stiction: a set-valued force that holds the body still until the applied force exceeds `μN`). Handling it properly means convex analysis / complementarity, not a smooth gradient. If you need it soon, the pragmatic hack is a regularised `−μN·tanh(q̇/ε)`, with the honest caveat that it can't truly stick.

**Impulsive losses aren't Rayleigh either.** The restitution coefficient from §2 removes energy in a single instant, not at a rate. Different mechanism, different code path.

### Hamiltonian side

In `(q, p)` coordinates the damping enters the momentum equation:

```text
q̇ = ∂H/∂p
ṗ = −∂H/∂q − ∂ℱ/∂q̇ |_{q̇ = ∂H/∂p}
```

This deliberately breaks the symplectic structure — correct physics, but it means any symplectic-integrator guarantees stop applying. Don't debug a "drifting" damped simulation as if it were a conservation bug.

### Code shape

Two options, both fitting the existing generics:

**(a) A trait method with a zero default** — smallest change:

```rust
pub trait LagrangianModel {
    type Coords: CoordSpace;
    fn accel(&mut self, q: Self::Coords, qdot: Self::Coords) -> Self::Coords;
    fn momentum(&mut self, q: Self::Coords, qdot: Self::Coords) -> Self::Coords;

    /// −∂ℱ/∂q̇, the Rayleigh damping force. Default: undamped.
    fn damping(&mut self, _q: Self::Coords, _qdot: Self::Coords) -> Self::Coords { /* zero */ }
}
```

**(b) A decorator model** — composable, and a nice demonstration of why you picked generics over an enum:

```rust
pub struct Damped<M: LagrangianModel> { pub inner: M, pub c: f32 }

impl<M: LagrangianModel> LagrangianModel for Damped<M> {
    type Coords = M::Coords;
    fn accel(&mut self, q: Self::Coords, qdot: Self::Coords) -> Self::Coords {
        let a = self.inner.accel(q, qdot);
        Self::Coords::integrate(a, qdot, -self.c)   // a − c·q̇
    }
    fn momentum(&mut self, q: Self::Coords, qdot: Self::Coords) -> Self::Coords {
        self.inner.momentum(q, qdot)
    }
}
```

The `integrate(a, qdot, -c)` trick works because `integrate` is literally `a + qdot·(−c)`, but it's a pun, not an abstraction. Both options really want `CoordSpace` widened with honest `add` / `scale` operations (and eventually an inner product, for `M`-weighted reflections and energy). That widening is probably the highest-leverage small refactor available right now.

One caveat on `Damped<M>`: dividing the damping force by mass to get an acceleration is only valid when `M = m·I`. Once models carry a real mass matrix, damping must be applied as a *force* before the `M⁻¹` solve, not bolted onto an acceleration.

---

## 8. The bridge: damping turns the sleigh into the tractrix

The two halves of this note are the same problem at different granularities.

Take the dynamic dragged mass — a body on a knife edge, pulled by a rod, now with viscous drag `−c ẋ`:

```text
m ẍ = F_rod + Aᵀλ − c ẋ
```

Let inertia become negligible relative to drag (`m/c → 0`, the **overdamped** or quasi-static limit). The `m ẍ` term drops, forces balance instantaneously, and what survives is a purely first-order kinematic law — exactly the `ẋ = (u·ḣ)u` of §5.

**The kinematic tractrix is the overdamped limit of the dynamic dragged mass.** Which gives you a genuine granularity ladder on one toy, in the sense of the primer's §6:

| Level | State | Model | When |
|-------|--------|-------|------|
| L0 | `x` | `ẋ = (u·ḣ)u`, first order | cursor toys, cheap AI followers, high drag |
| L1 | `x, ẋ` | dynamic, knife-edge `λ`, Rayleigh drag | visible overshoot and slew |
| L2 | `x, ẋ, θ, θ̇` | full sleigh with body inertia | jack-knifing, spin-down |

Same ports at every level (`u = handle motion`, `y = trailing position, rod direction, tension`), so a renderer or a parent machine can't tell which is running. That is the thing worth building toward, and this is a much cheaper place to practise it than a field solver.

---

## 9. Wedges, in order

1. **Fix the two small things in the current code** — reflect instead of clamp; flip `p` in the Hamiltonian bounce. Add a restitution coefficient while you're in there.
2. **Widen `CoordSpace`** with `add`, `sub`, `scale`, and (later) `dot`. Everything below wants it, and the `integrate`-as-scalar-multiply pun stops being tempting.
3. **`FlowModel` trait** (first-order `ż = F(z,t)`) — ~15 lines, and it's the only thing standing between you and §5.
4. **Dragged mass demo** driven by the mouse, with the rod re-projected each step. Draw the trace; confirm the tangent-length property visually.
5. **Rayleigh damping** as `Damped<M>` and/or a defaulted trait method, plus a `dissipation_rate()` output so you can plot `2ℱ` next to energy.
6. **Chaplygin sleigh** as two scalar ODEs — your first true `A(q) q̇ = 0` system, with an exact conserved energy to test the integrator against and an obvious qualitative behaviour to eyeball.
7. **General projection layer** (`λ` from §4) once you have a mass matrix, so constraints stop being hand-derived per model.
8. **Only then**: closed-loop multipliers, contact with friction, complementarity.

Each wedge should answer the same questions as before: *what is `z`, what is `y`, what is `u`, and what does this reduce to in the limit?*

---

## 10. Reading nudges

- Bloch, *Nonholonomic Mechanics and Control* — the reference; Chaplygin sleigh, reduction, stability
- Neimark & Fufaev, *Dynamics of Nonholonomic Systems* — the classical treatment
- Goldstein, ch. 1–2 — Rayleigh dissipation in its standard presentation
- Jose & Saletan, §3.x — clear on why vakonomic ≠ d'Alembert
- Brogliato, *Nonsmooth Mechanics* — when Coulomb friction and impacts stop being optional
- Any differential-geometry text (e.g. Lee, *Smooth Manifolds*) — Frobenius integrability, if you want the formal non-holonomy test
