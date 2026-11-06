# 42 — Extension Surface vs Adaptive Cut/Bridge

Status: comparative mathematical specification. No implementation. Boundary
Solver, Direct builder, validator and tolerances are unchanged.

## 1. Shared admissibility questions

Both strategies must answer the same seven questions before implementation:

1. Which exact geometric conditions admit the strategy?
2. Which original surfaces may be modified?
3. What is a valid common boundary?
4. Which B-rep guarantees must hold?
5. When must the strategy return a typed refusal?
6. How does the independent validator check the result?
7. How is the strategy represented in diagnostics and the public API?

A strategy that cannot answer one of these must not be added.

## 2. Extension Surface

### Geometric idea

Continue one or both source pieces beyond their original bounded domains with a
new, explicitly owned surface, then solve/validate the joint against that
extended source.

For an arc, a tangent extension is **not** another Torus. It is a separate
geometric piece: a ruled/tangent patch, a cylinder/plane segment, or a Level B
surface with its own domain.

### Allowed change

- original source surfaces remain the original bounded surfaces;
- extension surfaces are additional, named geometry;
- the seam may lie on original + extension surfaces only when the validator sees
  the actual extension geometry;
- the original path parameter domain is not silently redefined.

### Common boundary condition

For every seam sample:

```text
S(t) lies on an owned source surface or its explicitly declared extension
S(t) lies in that surface's declared UV domain
```

The extension must have a documented construction, ownership and validity
domain. It cannot be an implicit continuation inside a Torus parameter range.

### Required guarantees

- extension is represented in the result/diagnostics;
- B-rep faces may use extension surfaces;
- seam is validated against actual extension Surface objects;
- path/profile geometry outside the original domain is explicitly reported;
- no extension is created merely to make a candidate pass.

### Typed refusal

- requested extension violates the chosen tangent/continuity rule;
- extension self-intersects or leaves the allowed path envelope;
- extension surface cannot be built exactly/with the declared approximation;
- seam is not on both actual surfaces.

### Main risk

The solid contains geometry that is not the original sweep. That must be a
user-visible semantic change, not a hidden repair.

## 3. Adaptive Cut / Bridge

### Geometric idea

Keep the original bounded source surfaces, cut back their usable domains, and
connect the remaining pieces through an explicitly constructed bridge/cut
surface within documented bounds.

The original surfaces are not extended; their trim regions are changed.

### Allowed change

- original surfaces remain unchanged;
- the piece boundary is moved inward/along the source;
- a bridge surface and its two interfaces are added;
- the path may need documented local adjustment.

### Common boundary condition

Each new interface must satisfy the same contract A condition:

```text
bridge interface S(t) lies on original source surface
and on the bridge surface
inside both declared domains
```

The bridge itself must be validated independently; a bridge that only joins
vertices is not a valid geometric connection.

### Required guarantees

- trim regions are explicit;
- bridge construction has a documented bound (miter length, path deviation,
  curvature/tangent continuity);
- bridge is manifold and outward-consistent with both pieces;
- no original surface is silently replaced;
- validator checks both interfaces and the bridge.

### Typed refusal

- no trim region can satisfy the required clearance;
- bridge exceeds its documented envelope or self-intersects;
- tangent/curvature continuity cannot be met;
- bridge cannot be represented as a valid surface;
- interface does not lie on the original bounded surface.

### Main risk

The bridge can be geometrically valid but semantically artificial. It must be
an explicit strategy with bounds, not a generic fallback.

## 4. Comparison

| Criterion | Extension Surface | Adaptive Cut/Bridge |
|---|---|---|
| Original bounded surfaces | preserved; extension is additional | preserved; trim regions change |
| Geometry added | extension surface(s) | bridge/cut surface(s) |
| Path domain | extended outside original | reduced/local adjustment |
| Exactness | extension has its own exactness contract | bridge has its own exactness contract |
| Seam condition | seam on source + extension | seam on source + bridge |
| B-rep change | extra side topology | extra bridge topology and interfaces |
| Main semantic risk | original sweep is changed by extension | artificial transition geometry |
| Validation | actual extension surfaces | actual bridge and both interfaces |
| Likely complexity | lower topology, higher source semantics | higher topology, more geometric constraints |

## 5. Common API/diagnostics requirements

Both strategies must expose:

- strategy kind and source/extensions used;
- original vs extension vs bridge classification;
- seam/interface membership results;
- first failing sample and its U/V/domain;
- typed refusal reason;
- whether the result is a sweep, an extension, or a bridge construction.

`PlanarSweepResult` must make the distinction visible. A consumer must not have
to infer whether the solid is still the original sweep.

## 6. Independent oracle requirements

For either strategy:

1. Build the actual source/extension/bridge Surface objects.
2. Sample every seam/interface, including segment boundaries and closed-profile
   seam.
3. Project each point onto both declared surfaces.
4. Require gap tolerance and UV-domain membership.
5. Check continuity, shared vertices, manifold and outward orientation after
   the B-rep exists.
6. Verify the documented envelope for extension/bridge.
7. Return a typed refusal if any requirement fails.

The oracle must not call the Boundary Solver's rail equations to prove the
result.

## 7. Decision rule

Do not implement both now. Choose one only after specifying and testing:

- the exact geometry that the strategy may add;
- the semantic guarantee to the user;
- the refusal envelope;
- the independent oracle;
- the public diagnostics.

If neither can provide a clear semantic guarantee, typed refusal remains the
correct outcome for the original bounded exact case.

## 8. Status

- Exact Contract A: stable.
- Open Arc–Arc: typed refusal.
- Extension Surface: specification only.
- Adaptive Cut/Bridge: specification only.
- No geometry, solver, builder or tolerance change.

## 9. Geometry Ownership and User Intent

This section is mandatory before either extension or bridge is implemented.

### 9.1 Who authorizes added geometry?

A strategy may add geometry only under an explicit user/API intent. The default
policy stays Exact Contract A. There must be no automatic switch from Exact to
extension or bridge. Proposed policy shape:

`ust
enum GeometryPolicy {
    ExactOnly,          // default; refusal when no exact boundary exists
    AllowExtension,     // extension surfaces are explicitly permitted
    AllowAdaptiveBridge,// bridge/cut geometry is explicitly permitted
}
`

The policy is an input contract, not a fallback selected by the solver.

### 9.2 Is added geometry part of the original sweep?

No. Extension and bridge geometry are **generated geometry**, owned by the
strategy, and are not silently reclassified as the original sweep. The original
bounded source surfaces remain the original inputs.

### 9.3 How does the API report generated geometry?

The result must carry provenance, at minimum:

`	ext
original pieces and their source kinds
generated extension/bridge pieces
interfaces between original and generated geometry
strategy used
refusal reason, if any
`

A consumer must be able to tell whether the solid is:

- the exact original sweep;
- an original sweep plus explicitly owned extension geometry;
- an original sweep plus a bridge/cut transition.

### 9.4 Where is the boundary between original and generated geometry?

The boundary is an explicit interface surface/curve set. It must be:

- separately identified in diagnostics;
- validated on both the original bounded surface and the generated surface;
- included in the B-rep as a named construction boundary, not inferred from
  geometry alone.

### 9.5 Can the user get strict Exact behavior?

Yes. GeometryPolicy::ExactOnly must remain the default and must be
selectable regardless of whether extension/bridge implementations exist. It
returns typed refusal, never generated geometry.

### 9.6 Validation and oracle requirements

The independent validator must receive the actual Surface objects, original and
generated, and check every interface membership. The oracle must also confirm
that:

- generated geometry is labeled as generated;
- the interface belongs to both adjacent declared surfaces;
- no generated surface is reported as an original piece;
- the chosen policy matches the result;
- disabling the policy produces typed refusal.

