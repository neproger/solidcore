# 38 — ArcArcMiter mathematical specification

Status: specification and oracle design only. No Boundary Solver, Exact Torus,
Direct builder or tolerance change.

## 1. Exact-source contract (A)

For source surfaces `S1`, `S2` with bounded domains `D1`, `D2`, a candidate seam
`S(t)` is admissible only when

```text
for every sample t:
    exists (u1,v1) in D1: S(t) = S1(u1,v1)
    exists (u2,v2) in D2: S(t) = S2(u2,v2)
```

The rail residual is internal solver diagnostics, not an acceptance criterion.
The independent `boundary_validation::validate_joint_boundary` gate implements
this contract.

## 2. Arc–arc geometry

For a planar arc `C(u)` with center `c`, radius `R`, plane normal `Z`, and a
circular profile of radius `r`, the exact sweep is

```text
S(u, theta) = c + (R + sigma(u) * r cos theta) radial(u)
              + r sin theta * Z
```

where `sigma` is the profile offset sign in the local arc frame. At a fixed
profile angle `theta`, the offset section is a circle:

```text
rho(theta) = R + sigma * r cos(theta)
center     = arc center
```

The common boundary must equal the intersection of the two **same-theta offset
circles**, restricted to the two original angular domains:

```text
center1 + rho1(theta) radial1(u1)
  = center2 + rho2(theta) radial2(u2)
```

with `u1` in `[start1, end1]`, `u2` in `[start2, end2]`.

This is the correct miter condition. A generic Torus/Torus surface intersection
is not sufficient: it can contain branches with different profile phases, and
those branches do not connect the two profile correspondences.

## 3. Existence and branch rules

For each sampled `theta`:

1. Compute `rho1(theta)`, `rho2(theta)`.
2. Compute the circle–circle intersection of the two offset circles.
3. Keep only intersection points whose angle lies in the corresponding original
   arc domain.
4. Select the branch continuous with the joint; reject jumps.
5. If no point exists, the profile sample is uncovered.
6. If the offset radius is zero or negative, or the arcs are tangent/concentric,
   classify the degeneracy explicitly.

`ArcArcMiter` is admissible only if every profile sample has one continuous
same-theta common point. Otherwise it returns `UncoveredProfile` or a more
specific typed refusal. It must not invent a tangent extension.

## 4. Consequences for the open Arc–Arc case

The existing `ExactMiter` already refuses the open Arc–Arc case because some
profile samples require the tangent extension. Under contract A those samples
have no same-theta common point on the original bounded arcs. Therefore:

- the current `HybridAtJunction` result is not an Exact-source boundary;
- a separate `ArcArcMiter` cannot simply accept it;
- if the analytic offset circles do not intersect inside both original domains,
  the correct result under A is a typed refusal, not a fabricated seam.

A different construction (explicit extension surface, adaptive cut, or a
different joint model) is required if this case must produce a body.

## 5. Independent analytic oracle

The oracle must not call the rail solver. For each profile sample:

- build the two offset circles from arc center/radius and `theta`;
- compute their intersections analytically;
- filter by original arc angular domains;
- compare with the candidate seam;
- report the first uncovered `theta`, both arc parameters, both surface UVs and
  the exact 3D gap.

It must also test:

- profile seam (closed profile endpoint);
- tangent/one-point intersections;
- zero/negative offset radius;
- arc-domain endpoint cases;
- branch continuity.

## 6. Implementation order

1. Add the analytic circle–circle oracle as a test.
2. Prove or reject the existence of a continuous same-theta boundary for the
   open case.
3. Specify the `ArcArcMiter` result/refusal cases.
4. Implement only if the oracle and `boundary_validation` pass.
5. Re-run existing ExactMiter, line–arc and mixed-source regressions.

No further `HybridAtJunction` extension is planned for this case.

## Oracle result for the open Arc�Arc case

The independent circle�circle oracle (68 profile samples, no rail solver) gives:

`	ext
missing intersections: 0
outside both original arc domains: 33
multiple branches: 0
branch discontinuities: 0
first outside: sample 35
`

Every pair of offset circles intersects geometrically, but 33 of the 68 intersection points lie outside the original arc domains. Therefore the current open Arc�Arc case has **no continuous same-theta common boundary on the original bounded arcs**. Under contract A its correct result is a typed refusal, not a tangent extension.

## Preliminary typed refusal

`ust
enum ArcArcMiterError {
    UncoveredProfile { theta: f64 },
    NoCircleIntersection { theta: f64 },
    MultipleBranches { theta: f64 },
    BranchDiscontinuity { theta: f64 },
    DegenerateOffsetCircle { theta: f64 },
    OutsideOriginalArcDomain { theta: f64 },
}
`

The final API is deferred until the alternative joint construction is chosen.

