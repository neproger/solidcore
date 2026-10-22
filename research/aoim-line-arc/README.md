# AOIM Line–Arc — independent geometric research

Experimental Rust implementation of the **Analytical Offset-Intersection Miter (AOIM)** concept associated with [solidcore issue #447](https://github.com/tangency-labs/solidcore/issues/447).

This code is **standalone** and intentionally NOT part of the SolidCore kernel or its Rust↔C++ difftests.

## Geometry and conventions

Incoming path lies on the x-axis; the next path segment is supported by a circle of radius `R` through the origin with center `(R sin α, R cos α)`.

For signed offset `d`:
- incoming supporting line: `y = -d`
- circular supporting offset: `(x-R sin α)^2 + (y-R cos α)^2 = (R+d)^2`

Eliminating `d` produces the parabola:

`x² − 2R sin(α)·x + 2R(1−cos(α))·y = 0`.

The implemented local root `x = R sin(α) − sqrt((R+d)^2 − (−d−R cos α)^2)` goes through the origin when `d = 0`.

The model assumes a specific signed offset convention. It checks intersection of **supporting line and circle**, not that the intersection belongs to finite/extended path segments or to the swept face parameter domains.

## Run tests

```sh
cd research/aoim-line-arc
cargo test
```

Tests check: the zero-offset junction, selected offset intersections on both loci and the analytic parabola, tangent slope, and invalid inputs.

## Research status / next step

**Validated algebraically:** the selected corresponding offset intersections satisfy the parabola equation.

**NOT yet validated:** a common trimmed B-rep boundary, no overlaps or gaps across whole profile regions, trimmed surface domains, inner-radius degeneracies, or improvement in time/memory compared with SMLib.

Next: explicit 2D swept-region tests with a rectangular/concave profile and branch/domain checks, then B-rep experiments when suitable SolidCore APIs are available.

**No upstream PR is intended until that evidence exists.**
