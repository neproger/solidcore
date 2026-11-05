# 24 — Direct B-rep architecture audit vs SolidCore periodic faces

Status: architectural cause identified. The arc–arc failure is not a missing
periodic-branch patch in `create_uv_trim_curve`; it is the way the joint builder
uses an **unsplit periodic surface** for its K side faces.

## 1. What SolidCore already does

Two existing constructions show the kernel contract.

### `build_direct_single` (our Milestone 1 path)

`build_direct_single` splits the swept surface at the K profile V boundaries
with `Surface::split_at(..., SurfParamType::V)`. Each side face is then built on
one **non-periodic patch**, with its own local UV domain. The rails and cap
segments are iso-curves of that patch.

### Production rotational sweep

`crates/solidcore/src/sweep/rotational_sweep_geometry.rs` and
`analytic_revolution.rs` do not independently drop each shared 3D edge on each
face. They obtain the boundary from
`Surface::create_natural_uv_trim_curves` and carry the resulting 3D curves,
UV lines and orientations together into the face/loop construction (see the
`EdgeSweepHigher` path around the `natural` block).

The UV trim and the 3D edge are created as **one pair from the surface
boundary**, not as two independent representations.

## 2. What `direct_brep.rs` does instead

`build_direct_chain` (and the old 2-piece joint builder) uses the **full
periodic side surface** for all K side faces:

- one `Surface` per piece with the full V domain;
- K faces with hand-built loops over seam, rail and cap edges;
- each shared 3D edge is dropped independently on each adjacent surface to make
  its UV trim (`create_uv_trim_curve`).

This is a different contract from `build_direct_single` and from the production
rotational sweep.

For line–line and line–arc the seam lands near `V = 0` on both sides, so the
full-surface loops happen to close. Arc–arc lands on the other periodic
representative (`V ≈ 1` for the rail and `V = 0` for the seam edge), which the
unsplit-face loops cannot reconcile. The UV-branch corrections therefore only
move one side of a loop.

## 3. Audit conclusion

The failure belongs to **surface parameterization and loop construction**:

| Question | Finding |
|---|---|
| Surface parameterization | side surfaces are periodic and not split at the K profile boundaries |
| Edge representation | 3D edge and its UV trims are generated independently; kernel's natural UV boundary is not used |
| Loop construction | K side faces are loops over one full periodic surface, not one bounded patch |

The 3D geometry (`22`) is correct; the representation violates the kernel's
periodic-face pattern.

## 4. Recommended refactor

Change the joint/chain builder to mirror `build_direct_single`:

1. split each side surface into K patches at its profile V boundaries;
2. create each K side face on its own patch with the patch's local UV domain;
3. keep the joint seam vertices/edges shared between the two adjacent patches;
4. keep start/end caps at the outer patches' U boundaries;
5. do not add any arc–arc condition; line–line and line–arc use the same patch
   path.

This removes the cross-periodic loop instead of trying to repair it after the
fact. `create_uv_trim_curve` can then receive or derive a local UV curve on a
non-periodic patch, where the branch ambiguity disappears.

## 5. Criteria after the refactor

- Arc–arc direct: `check_brep` 0 failures, closed UV loops, manifold/outward,
  integrity.
- Line–line, line–arc exact/hybrid and N=3 remain green.
- No special-case `if arc_arc`.
- No change to Boundary Solver or Surface Builder 3D geometry.

## 6. Current decision

- Stop periodic-branch post-processing attempts.
- Keep the 3D geometry and the N-segment topology results.
- Refactor the side-face construction to split patches, then retest arc–arc.
- AdaptiveCut remains blocked until arc–arc direct is green.

## 7. Patch-split experiment (not complete)

The side-face construction was changed to split each side surface at the K
profile V boundaries with `Surface::split_at` and to create each side face on
its own patch. No arc–arc condition was added.

Result:

- line–line, line–arc exact/hybrid and N=3 line–line–line remain green;
- arc–arc direct improves from 11 failures and V gap `0.7499833` to 12
  failures and V gap `0.25`;
- it is **not** green: the remaining `0.25` gap shows that the patch-local
  boundary mapping is still incomplete. The rail/cap/seam edges and their UV
  boundaries still need to be expressed entirely in each patch's local V
  domain, not in the full-surface V values.

This confirms the architectural direction but is not the finished fix. The
next step is to audit the patch-local face loop: each patch's V domain,
rails at its two boundaries, and cap profile edge must use one local
parameter interval.