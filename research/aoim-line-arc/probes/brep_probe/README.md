# AOIM B-rep probe

Public-API probes for the direct-shell design in `../../BREP_PLAN.md`. The probe
does not modify the kernel.

```sh
cd research/aoim-line-arc/probes/brep_probe
cargo run --offline
```

Current results (the probe prints every face and assertion):

| case | profile | contour | F / E / V | area |
|---|---|---:|---:|---:|
| 90° baseline | `[0, 2]` | 6 | 8 / 18 / 12 | 51.933998 |
| 90° symmetric | `[-2, 2]` | 7 | 9 / 21 / 14 | 105.358356 |
| 30° | `[-2, 2]` | 7 | 9 / 21 / 14 | 88.382434 |
| 150° | `[-2, 2]` | 7 | 9 / 21 / 14 | 127.197778 |
| 179° (`line_length=500`) | `[-2, 2]` | 7 | 9 / 21 / 14 | 2492.149982 |

All cases end with `AOIM_PROBE_PASS` for both `caps-first` and `sides-first`.
Each has 4 edge uses per edge (2 `Same`, 2 `Opposite`), one void and one solid
shell/region, and one faceuse per face in each shell. The last face still
returns the p6 `TestClosureSplitRegion` error, but the split is complete; the
flags are already correct and `reset_infinite_region` is a before/after no-op
confirmation.

The sensitive parts are:

- bottom cap loop reversal (otherwise orientation is 7/1);
- exact contour from the Hybrid AOIM region, including the negative-`d`
  tangent Line–Arc segment;
- shared vertices, wire edges and edge-use selection;
- plane side faces and cylinder side faces, with no seam face.

Volume and outward-normal checks remain blocked by `SmSurface::DropCurve` and
`CalculateBoundingBox`.
See `../../BREP_REPORT_RU.md` for the detailed Russian report. If a probe
assertion fails after a kernel update, the recorded blocker or API has changed
and the report needs updating.
