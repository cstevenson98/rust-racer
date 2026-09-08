# Hierarchical Classical Multiphysics — Primer & Nudges

A sketch for growing `mechanical_system` into a flexible, hierarchical classical engine: coupled machines, discretised fields, waves, constraints, and coarse-grained “effective” models. Not a roadmap commitment — a vocabulary, some worked examples, and a few first wedges.

---

## 1. What you’re actually building

At every level you want the same abstract loop:

```text
state z  →  model F  →  ż = F(z, u, t)  →  integrate  →  observe y = H(z)
```

| Symbol | Meaning |
|--------|---------|
| `z` | Micro (or meso) state in some coordinate chart |
| `u` | Inputs / controls / boundary data |
| `y` | Outputs / ports / what the parent level sees |
| `F` | Dynamics (Euler–Lagrange, Hamilton, DAE, semi-discrete PDE, …) |
| `H` | Observation / aggregation map |

Hierarchy means **stacking** these loops. A child’s `y` becomes a parent’s `u` or part of a parent’s `z`, and a parent’s `u` may set a child’s boundary conditions. The important discipline is that a parent never reaches into a child’s state directly — it only reads `y` and writes `u`. That single rule is what lets you swap a child’s implementation (rigid rope → elastic rope → discretised string) without touching the parent.

Your current split already points this way:

- **Coords** (`CoordSpace`) — what `z` is made of
- **Model traits** (`LagrangianModel` / `HamiltonianModel`) — how `F` is expressed
- **Constraint config** (`DomainConstraint`) — first taste of projection / DAE
- **Separate typed arrays** (#3 generics) — different granularities don’t share one `Vec<dyn>`

---

## 2. Generalised coordinates as the spine

### Why coordinates are the abstraction, not “objects”

Most game engines model *things* (rigid bodies, joints) and hard-code the solver. A Lagrangian engine models *configuration spaces*, and everything else — a bead on a wire, a lever, a string, a flock of particles — is “some manifold `Q` plus a kinetic metric plus a potential”. The payoff is that constraints, machines, and fields stop being separate subsystems, and become different choices of `Q`.

Treat “coordinates” as a **chart**, not as `Vec2`:

```text
Q: configuration manifold (or discrete stand-in)
TQ: (q, q̇)     Lagrangian chart
T*Q: (q, p)    Hamiltonian chart
```

In code, keep pushing on:

1. **`CoordSpace`** — integrate, scale, maybe inner product
2. **Charts / bundles** — `LagrangianState<C>` vs `HamiltonianState<C>`
3. **Legendre maps** — you already have `momentum` / `velocity`; those are the seed of chart changes

### The single most valuable upgrade: a mass matrix

Right now every model quietly assumes `m = 1`, so `p = q̇` and `T = ½|q̇|²`. The general form is

```text
T = ½ q̇ᵀ M(q) q̇        p = M(q) q̇
```

`M(q)` is a Riemannian metric on `Q`. Once you have it:

- a bead on a curve gets `M(s) = m |P′(s)|²` for free
- a lever gets `M = I` (moment of inertia)
- a pulley machine gets a **configuration-dependent** `M(θ)` automatically
- a discretised string gets a sparse `M`
- rigid bodies later get the standard `6×6` spatial inertia

And the Euler–Lagrange equation in this form is always

```text
M(q) q̈ + ½ M′(q)[q̇, q̇] = −∂V/∂q + Q_ext(u)
```

That single line covers every example in section 8. The `½ M′ q̇²` term is what people call centrifugal/Coriolis; it appears purely because `M` depends on position.

**Nudge:** widen `LagrangianModel` with an optional `mass(&mut self, q) -> Metric` (default identity), and let `momentum` use it. Even a scalar `f32` metric unlocks the seesaw and the bead.

### Transforming between Lagrangians

You rarely “convert L₁ into L₂” as strings of symbols. You change **description**:

| Move | Idea | Concretely |
|------|------|------------|
| **Point transform** | `q = φ(r)` — reparametrise the same system | Cartesian → polar; world position → track `s` |
| **Gauge / total derivative** | `L' = L + dF/dt` — same EL equations | drop constant offsets, shift momenta |
| **Legendre** | `L(q,q̇) ↔ H(q,p)` — same dynamics, different chart | your `momentum` / `velocity` pair |
| **Constraint reduction** | eliminate DOF using `g(q) = 0` | pulley + lever collapses to one angle |
| **Symmetry reduction** | quotient by a group action (Routh) | ignore cyclic angle, keep its momentum |
| **Coarse-graining** | new coords `r = π(q)` plus a closure | string → first two modes |

The first four are *exact*. The last one is where you must make and record a modelling choice.

Key mechanical fact worth internalising: if `q = φ(r)`, the Lagrangian in the new chart is just `L̃(r, ṙ) = L(φ(r), Dφ(r) ṙ)`, and the metric transforms as `M̃ = DφᵀM Dφ`. That is the *entire* content of “changing coordinates”, and it’s something you can implement generically once and reuse for constraints, tracks, and coarse-graining alike.

**Nudge:** add an explicit trait for *kinematic maps*:

```text
trait CoordMap {
    type Fine;
    type Coarse;
    fn embed(coarse: Coarse) → Fine;      // r ↦ q = φ(r)
    fn project(fine: Fine) → Coarse;      // q ↦ r = π(q)
    fn jacobian(coarse: Coarse) → Dφ;     // q̇ = Dφ · ṙ
}
```

First instances: `s ∈ [0,1]` along a spline ↔ world `x`; then N-particle block embeddings. Note `embed` + `jacobian` is exactly what you need to *derive* a constrained model rather than hand-deriving it — see section 8.1.

---

## 3. Coupling and simple machines

A “simple machine” is usually:

- several bodies / bars / pulleys
- **holonomic** constraints `g(q) = 0` (and their derivative `ġ = 0`)
- forces / motors at joints

The conceptual point: a machine is not new physics, it is a **submanifold** of a bigger configuration space. A pulley says “this rope length is constant”. A lever says “these two tips are rigidly linked through a pivot”. Gears say “θ₂ = −(r₁/r₂)θ₁”. All are `g(q) = 0`.

Formulation ladder (pick one to implement first, don’t implement all):

1. **Reduced coordinates** — parametrise the constraint manifold, then run unconstrained EL in `r`. Exact, small state, no drift, no solver. Cost: you must find the parametrisation, which is only easy for tree-like machines. **Start here.**
2. **Index-1/3 DAE with multipliers** — keep `q` redundant, add `λ` for `g(q) = 0`, solve a saddle-point system each step. General (handles loops), but needs linear algebra and stabilisation against drift.
3. **Penalty / Baumgarte** — soft constraints via stiff springs. Trivial to write, ruins energy behaviour and timestep budget. Fine for gameplay-only bits.

**Ports:** expose scalar or vector **effort/flow** pairs (force/velocity, torque/ω) so machines compose like circuit elements. The product `effort × flow` is power, so composing along ports gives you a free energy audit — if a coupling leaks power you did it wrong. That’s bond-graph / port-Hamiltonian intuition without buying the whole religion.

**Nudge:** the pulley-seesaw in section 8.1 is a better first machine than a generic rod, because it produces a non-trivial `M(θ)` while still being one DOF.

---

## 4. Non-holonomic constraints

Non-holonomic constraints restrict velocities in a way that is **not** integrable to any `g(q) = 0` — e.g. rolling without slipping, or a knife-edge/skate that can only move along its own heading. The give-away is that you can still reach every configuration, you just can’t take arbitrary paths there (parallel parking).

Written as `A(q) q̇ = 0`, the equations of motion become

```text
M q̈ + ½M′[q̇,q̇] + ∂V/∂q = Aᵀ λ,      A(q) q̇ = 0
```

Two important warnings:

- **Lagrange–d’Alembert ≠ vakonomic.** Constrained variational optimisation gives *different* (wrong, for rolling) equations than the d’Alembert principle. Use d’Alembert for real mechanics.
- Non-holonomic systems are **not** Hamiltonian in the usual sense; don’t expect your symplectic integrator’s guarantees to survive.

**Nudge:** don’t start here. Get holonomic machines solid first. When you do: one rolling wheel or one skate in 2D, with `A(q)` as a single row, is enough to learn the shape.

---

## 5. Discretised fields and wave equations

Continuum → finite DOF the same way N-body did:

```text
field u(x,t)  →  nodal vector U(t) ∈ R^N
```

Semi-discrete wave equation (prototype):

```text
M Ü + K U = f(t)
```

That’s **already** a Lagrangian system:

```text
L = ½ U̇ᵀ M U̇ − ½ Uᵀ K U
```

with `Coords = DVector` (or blocked `SVector`), and sparse `M`, `K`. Nothing about your trait needs to change — a field is just a model whose `Coords` is big and whose `accel` is a sparse solve/matvec:

```text
accel(U, U̇) = M⁻¹ (−K U + f)
```

For a lumped-mass discretisation `M` is diagonal, so `M⁻¹` is elementwise and there is no linear solve at all. That’s the version to write first.

Where sparsity actually pays: `K` for a 1D string is tridiagonal, so `K U` is `O(N)`, not `O(N²)`. Your existing free N-body is dense-but-trivial (`accel = 0`); the string is the first place `nalgebra-sparse` earns its keep.

**Integrator warning:** explicit Euler is *unconditionally unstable* for the wave equation — energy grows every step regardless of `dt`. This is the point where you’ll want velocity-Verlet / leapfrog or Newmark. Good news: those are still one function of `(q, q̇, accel)`, so they slot in beside `update_euler` as another method on the same system type.

Hierarchy angle:

- **Fine:** nodal `U` (N ≈ 100s)
- **Coarse:** modal amplitudes `r` from the first few eigenvectors of `(K, M)`, or POD modes from a recorded run
- **Effective:** `M̄ r̈ + K̄ r = B u`, with `y = C r` — a handful of DOF, same interface

Because the modes are `U ≈ Φ r`, this is *literally* a `CoordMap` with `embed = Φ`, `project = Φᵀ M`. Coarse-graining a field is the same machinery as constraining a machine.

---

## 6. Effective models: inputs, outputs, granularity

This is the hierarchical payoff.

```text
          u (inputs)
              │
              ▼
     ┌─────────────────┐
     │  effective z̄    │   z̄ = π(z_micro) or an independent reduced state
     │  ż̄ = F̄(z̄, u)   │
     └─────────────────┘
              │
              ▼
          y = H(z̄)     (outputs / ports to parent)
```

Varying granularity = choosing different `(π, F̄, H)` for the same physics:

| Level | State | Typical use |
|-------|--------|-------------|
| Micro | all particles / all nodes | accuracy, contact, visuals |
| Meso | rigid aggregates, few modes | machines on flexible track |
| Macro | 1–2 DOF effective `L` | gameplay, AI, control, prediction |

**Contracts to design early:**

1. **`Observation`** — `y = H(z)` is a *pure function of state*. If you find yourself wanting to store `y`, it probably belongs in `z`. (Rope tension in section 8.1 is a good test case: it feels like state, it isn’t.)
2. **`Actuation`** — how `u` enters `F`: as a generalised force `Q(u)`, a prescribed coordinate (rheonomic constraint), or a boundary condition on a field.
3. **`Consistency`** — is `F̄` an *exact* reduction (constraint elimination, symmetry reduction) or a *fitted* one (linearisation, mode truncation, empirical damping)? Tag it, because only the first kind is safe to swap silently.

A useful discipline: every level should be able to answer *“if I run the finer model and project with π, do I get roughly this?”* Even if you only check it once by hand, having `π` written down makes the claim falsifiable.

**Nudge:** for the inverse-R particle, define

- `y = (q, |q − c|, energy)`
- `u =` time-varying `k(t)` or a moving centre `c(t)`

Then a “coarse” model in polar coordinates about `c` — same physics, different chart, exact — via `CoordMap`. That’s hierarchy without multiphysics yet, and it gives you a working `CoordMap` to reuse on the machines.

---

## 7. Suggested software shape (maximum flexibility without soup)

Prefer **composition of explicit layers** over one uber-enum:

```text
mechanical_system/          # charts, traits, integrators, pure models
  coords.rs                 # CoordSpace, metrics, CoordMap
  lagrangian.rs             # trait + systems
  hamiltonian.rs            # trait + systems
  integrators.rs            # euler, verlet, newmark
  models/                   # free particle, inverse-r, machines
  constraints/              # holonomic first
  fields/                   # string1d
track_core/ (or a sim crate) # Resources, coupling glue, rendering
```

Patterns that age well:

- **#3 generics** per concrete model type (you chose this)
- **Resources** for coupled / N-body / field systems (one owner of `z`)
- **Entities** as views (meshes, gizmos), not as the coupled store
- **Ports** as small structs `(y, u)` between systems, stepped in an explicit order

Avoid early: `Box<dyn Any>` physics, one global DAE with every multiplier on Earth, automatic symbolic Lagrangian manipulation, and premature autodiff. Hand-derive the first few machines — it’s how you learn which abstractions actually pay.

One thing worth deciding sooner rather than later: **who owns time**. Sub-models may want different timesteps (a stiff string vs a slow lever). A simple convention — parents step children an integer number of substeps per parent step, and only exchange `u`/`y` at parent boundaries — avoids a lot of pain later.

---

## 8. Worked examples

### 8.1 Pulley driving a seesaw (the canonical first machine)

```text
            ╭─◯─╮   ideal pulley, fixed at P
           ╱     ╲
      rope│       │rope
          │       ▓  hanging mass m   ↓ g
          │
    ●─────┴───────────────●
   left    △ pivot O     right
    tip                   tip

    θ = lever tilt, defined so the left tip height is  a·sin θ
```

**Bodies:** a uniform lever of mass `M_l`, half-length `a`, pivoted at its centre (so `I = M_l a² / 3` about the pivot); a hanging mass `m`; an ideal massless rope over an ideal massless pulley.

**Step 1 — count DOF.** Naively: lever angle `θ`, mass height `y_m` → 2 DOF. The inextensible rope imposes one holonomic constraint, so the machine is **1 DOF**. Choose `q = θ`.

**Step 2 — write the constraint as a map.** With the pulley directly above the left tip and small tilt, the tip→pulley segment length is `h − a sin θ`, so a constant total rope length forces

```text
y_m(θ) = c − a sin θ            ⟹   ẏ_m = −a cos θ · θ̇
```

Falling mass ⇒ rising left tip, which is the machine you wanted. This *is* the `CoordMap`: `embed(θ) = (θ, c − a sin θ)`, with Jacobian `(1, −a cos θ)`.

**Step 3 — push the metric through the map.** Kinetic energy of both bodies in terms of the single coordinate:

```text
T = ½ I θ̇²  +  ½ m ẏ_m²
  = ½ (I + m a² cos²θ) θ̇²
  = ½ A(θ) θ̇²,          A(θ) = I + m a² cos²θ
```

**This is the lesson of the example.** A constant-inertia lever plus a constant-mass weight produce a **configuration-dependent effective inertia** `A(θ)`, purely from the geometry of the coupling. Near `θ = 0` the mass contributes fully (`A ≈ I + ma²`); near `θ = ±π/2` the rope is instantaneously perpendicular to the tip’s motion and the mass decouples (`A → I`).

**Step 4 — potential and equation of motion.**

```text
V(θ) = m g y_m(θ) = m g (c − a sin θ)          (lever CoM sits at the pivot)

A(θ) θ̈ + ½ A′(θ) θ̇² = −∂V/∂θ = m g a cos θ

⟹  θ̈ = [ m g a cos θ + m a² sin θ cos θ · θ̇² ] / (I + m a² cos²θ)
```

The `θ̇²` term is exactly the `½M′[q̇,q̇]` term from section 2 — you get it for free from the metric, you never “add a centrifugal force”.

**Step 5 — it drops into your existing engine unchanged.** `Coords = f32` already implements `CoordSpace` and `UnitBoxBound`:

```rust
pub struct PulleySeesaw {
    pub half_len: f32,      // a
    pub lever_inertia: f32, // I about the pivot
    pub hanging_mass: f32,  // m
    pub gravity: f32,       // g
    pub drive_torque: f32,  // input u: motor/hand at the pivot
}

impl LagrangianModel for PulleySeesaw {
    type Coords = f32; // θ

    fn accel(&mut self, theta: f32, thetadot: f32) -> f32 {
        let (sin, cos) = theta.sin_cos();
        let ma2 = self.hanging_mass * self.half_len * self.half_len;
        let inertia = self.lever_inertia + ma2 * cos * cos;
        let gravity_torque = self.hanging_mass * self.gravity * self.half_len * cos;
        let geometric = ma2 * sin * cos * thetadot * thetadot;
        (gravity_torque + geometric + self.drive_torque) / inertia
    }

    fn momentum(&mut self, theta: f32, thetadot: f32) -> f32 {
        let cos = theta.cos();
        let ma2 = self.hanging_mass * self.half_len * self.half_len;
        (self.lever_inertia + ma2 * cos * cos) * thetadot
    }
}
```

Note `momentum` is no longer the identity — this is the first model where the Legendre map is genuinely non-trivial, which makes it a good test that your `p()` / `qdot()` helpers are actually doing something.

**Step 6 — ports.** Inputs and outputs of the machine, all pure functions of `(θ, θ̇)`:

```text
u = drive_torque(t)            input at the pivot
y_tip  = (−a cos θ, a sin θ)   left tip position, for rendering or coupling
y_mass = c − a sin θ           hanging mass height
y_T    = m (g − a cos θ · θ̈ + a sin θ · θ̇²)     rope tension
y_E    = ½ A(θ) θ̇² + V(θ)      total energy, for a drift check
```

Tension is the interesting one: it *feels* like state but is a function of the state, and it’s the natural quantity to hand to a parent (“the rope snaps if `y_T > T_max`”) or to a child (“the rope is elastic, here’s the load”).

**Step 7 — granularity ladder for the same machine.** This is the hierarchical story in miniature:

| Level | State | Model | When |
|-------|--------|-------|------|
| L0 macro | `θ, θ̇` | linearise: `θ̈ ≈ m g a / (I + m a²)`, constant | gameplay, AI prediction, UI |
| L1 meso | `θ, θ̇` | the nonlinear `A(θ)` above | the default sim |
| L2 fine | `θ, y_m` + velocities | elastic rope: drop the constraint, add `V_rope = ½k(ℓ(θ,y_m) − L₀)²` | rope stretch, snap, slack |
| L3 micro | `θ`, plus `N` string nodes | rope as a discretised 1D wave (section 5), pulley as a boundary condition | visible rope waves, friction at the pulley |

Every level exposes the same `y` (tip position, mass height, tension), so the renderer and any parent machine don’t know which one is running. That is the whole design goal, demonstrated on something you can hand-check.

**Sanity checks to write as tests:**
- with `m = 0`, the lever should just hold still (or spin freely under `u`)
- with `g = 0` and `u = 0`, `A(θ) θ̇` should be… *not* conserved (`θ` isn’t cyclic), but energy `½A θ̇²` should be
- L0 and L1 should agree for small `θ` over short horizons
- L1 and L2 should agree as `k → ∞`

### 8.2 Bead on your spline track (connects mechanics to `track_core`)

The track already gives you `P(s)`; that’s a `CoordMap` from `s ∈ [0,1]` into the plane. A mass sliding on it is a 1-DOF Lagrangian system whose metric comes straight from the curve:

```text
T = ½ m |P′(s)|² ṡ²        ⟹   A(s) = m |P′(s)|²
V = m g P_y(s)

A(s) s̈ + ½ A′(s) ṡ² = −∂V/∂s

⟹  s̈ = −[ g P′_y(s) + (P′(s) · P″(s)) ṡ² ] / |P′(s)|²
```

Same shape as the seesaw: position-dependent inertia plus a geometric `ṡ²` term. Note that `s` is *not* arc length for a Bezier, which is exactly why `|P′(s)|²` shows up — the metric absorbs the parametrisation, so you don’t need arc-length reparametrisation to get correct physics. (You may still want it for stable rendering.)

You need `P′` and `P″`, which for your cubic segments are one and two differentiations of the same weighted-points polynomial you already evaluate — cheap to add next to `evaluate`.

This is the wedge that finally connects `mechanical_system` to the track: the current `s`-space free particle is the degenerate case `|P′| = const`, `g = 0`.

### 8.3 Seesaw + bead: composing two machines through a port

Put the bead’s track on the lever (or drop a bead onto the right arm) and you have a two-machine system:

```text
   lever θ ──y_tip──▶ track transform ──▶ bead s
        ▲                                   │
        └──────── y_reaction (torque) ◀──────┘
```

Two honest options:

- **Monolithic:** treat `q = (θ, s)` as one 2-DOF Lagrangian, derive `M(q)` once (it will be non-diagonal — that off-diagonal *is* the coupling). Exact, and the right choice if the masses are comparable.
- **Weakly coupled:** step the lever, hand the bead its (accelerating, rotating) frame as `u`, step the bead, hand back the reaction torque as `y`. Cheap, modular, and wrong at O(dt) — fine when the bead is light.

The fact that both are expressible with the same traits, and that you can *test one against the other*, is the payoff for keeping ports explicit.

### 8.4 Plucked string (first field, first real sparsity)

```text
u(x,t), fixed ends → nodes U₁..U_N, spacing h

M = diag(ρh)                (lumped mass)
K = (τ/h) · tridiag(−1, 2, −1)
accel(U, U̇) = M⁻¹(−K U + f)
```

Start with `N ≈ 64`, a triangular initial displacement, zero initial velocity, and *velocity-Verlet, not Euler*. Render as a gizmo linestrip. Then:

- couple it to 8.1 as the L3 rope
- reduce it to two modes and check the coarse model tracks the fine one for a while
- add a driving boundary node as `u(t)` to feel “input to a field”

---

## 9. First wedges (do in order)

1. **Metric hook** — add `mass(q)` to the Lagrangian trait (default `1`), route `momentum` through it. Unlocks 8.1 and 8.2 with no other changes.
2. **`PulleySeesaw`** as a Resource with gizmo rendering — one DOF, non-trivial `M(θ)`, real ports.
3. **`CoordMap` trait**, with the seesaw constraint and the spline `s ↔ x` as its first two instances.
4. **`P′`, `P″` on the spline** + the bead model (8.2). Now the track is physical.
5. **Velocity-Verlet** beside `update_euler`, and an energy-drift check on 8.1.
6. **1D string** (8.4) with sparse `K`; then use it as the elastic rope in 8.1’s L3.
7. **Observation / actuation structs** — formalise `y` and `u` once you have two machines wanting to talk (8.3).
8. **Only then** multipliers for closed loops, or non-holonomic contacts.

Each wedge should answer: *what is `z`, what is `y`, what is `u`, who owns the Resource, and what does it reduce to?*

---

## 10. Mental model (one paragraph)

A hierarchical classical multiphysics engine is less “one solver for everything” and more a **grammar**: generalised coordinates with a metric; chart changes that carry both the coordinates and the metric; models that produce `ż` and accept `u`; constraints as either reduced coordinates or multipliers; fields as large `L`/`H` with sparse operators; and parents that only ever see `y = H(z)` and only ever speak `u`. Flexibility comes from keeping those interfaces boring and letting each level pick its own `Coords` and its own integrator — exactly the direction your generic Lagrangian/Hamiltonian split is already going.

---

## 11. Reading nudges (optional)

- Jose & Saletan, *Classical Dynamics* — constraints, Legendre, non-holonomic caution
- Lanczos, *The Variational Principles of Mechanics* — the “why” behind the grammar
- Marsden & Ratiu, *Introduction to Mechanics and Symmetry* — reduction, momentum maps
- Hairer, Lubich & Wanner, *Geometric Numerical Integration* — why Euler will hurt fields and orbits
- Featherstone, *Rigid Body Dynamics Algorithms* — if machines grow into articulated trees
- Duindam et al., *Modeling and Control of Complex Physical Systems* — port-Hamiltonian composition

Keep the code classical and explicit; borrow vocabulary from port-Hamiltonian without requiring the full stack.
