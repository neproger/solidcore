# 22 — Arc–arc geometric consistency audit

Status: **PASS for the 3D geometry.** The miter curve lies on both arc–arc
swept surfaces. The remaining direct B-rep failure is UV/trim parameterization,
not a wrong surface extension, curvature or coordinate frame.

## 1. Case

Same boundary case as `21-arc-arc-direct-blocked.md`:

- arc A: centre `(0,1)`, radius `1`, `-180°..-90°`;
- arc B: centre `(-1,0)`, radius `1`, `0°..90°`;
- circular profile radius `0.25`, `K=4`;
- `HybridAtJunction`: 36 exact / 32 junction samples;
- rail residual `6.995e-14`.

## 2. Audit method

1. Build both Level B swept side surfaces with the normal preparation.
2. Build the `JointBoundary` with the existing solver.
3. For each seam sample (17 points per seam segment), drop it onto **each**
   surface independently and record the maximum gap.
4. Record the builder's analytic-vs-Level-B surface deviation for each side.

This directly tests the three hypotheses: wrong arc continuation, lost
curvature, and a mismatched profile/coordinate frame.

## 3. Results

| Check | Value |
|---|---|
| Surface A analytic-vs-Level-B max deviation | `4.280e-5` |
| Surface B analytic-vs-Level-B max deviation | `4.280e-5` |
| Miter curve distance to surface A | `3.779e-6` |
| Miter curve distance to surface B | `3.779e-6` |
| Miter curve distance to both (max) | `3.779e-6` |
| Boundary rail residual | `6.995e-14` |

The miter samples are well inside the kernel validation scale; the surface
builder and the boundary solver agree on the 3D geometry.

## 4. Conclusion

Hypotheses 1–3 are **not supported** for this case:

- the boundary does not use a different continuation from the surface builder;
- the surface builder preserves the arc curvature to the measured level;
- the profile/normal placement is consistent, since the same 3D seam drops
  onto both surfaces with the same small gap.

The 11 `check_brep` failures of `21-arc-arc-direct-blocked.md` therefore arise
after the 3D geometry, in the UV trim/loop creation for the periodic profile.
This also explains why replacing the 3D seam or its rails would not help.

## 5. Next

Keep the geometry pipeline unchanged. The UV-trim audit of the failing loop is in
`23-arc-arc-uv-trim-audit.md`:

- dump the four `UVTrimCurve`s of the loop and their end parameters;
- compare the seam edge's UV trim branch with the two rails and the cap;
- find whether `create_uv_trim_curve` chooses `V≈1` where the rails use
  `V=0`, or whether the loop closure uses a different branch after
  `assign_uv_curves`.

Only that UV-level fix should be attempted in the Direct B-rep Builder.

## 6. Reproduce

The audit is the `arc_arc_geometric_consistency` test in
`prototype/tests/direct_joint.rs`; it prints the values above and asserts each
distance is at most `1e-4`.
