//! Planar curve data used by the mathematical prototype.
//!
//! Deliberately independent of `solidcore::curve::Curve`: level A must be
//! testable without the kernel. Level B converts these specs to the real types
//! in [`crate::level_b_geometry`] (feature `solidcore`).
//!
//! All variants expose the same differential interface:
//! [`CurveSpec::natural_interval`] and [`CurveSpec::differential`]. The miter
//! solver never branches on the variant.

use core::fmt;

use crate::differential_geometry::{DifferentialData, add, cross, norm, rot90, scale, sub, unit};

/// A malformed curve or a degenerate point of it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryError {
    /// The curve data are inconsistent (degree, knots, non-finite values).
    InvalidCurve(&'static str),
    /// The derivative is zero at the requested parameter.
    ZeroSpeed,
    /// The parameter is outside the extended piece.
    OutsideCurve,
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCurve(reason) => f.write_str(reason),
            Self::ZeroSpeed => f.write_str("zero curve speed"),
            Self::OutsideCurve => f.write_str("parameter outside the piece"),
        }
    }
}

impl std::error::Error for GeometryError {}

/// A planar curve, given by control data on its natural parameter interval.
///
/// The same geometry can be represented by several variants (a line as a
/// `Line` or as a degree-1 `BSpline`, a circle as a rational B-spline). The
/// harness treats the variants as representations of a point set, not as
/// different algorithms.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveSpec {
    /// A segment from `start` to `end`.
    Line {
        /// Segment start.
        start: [f64; 2],
        /// Segment end.
        end: [f64; 2],
    },
    /// A circular arc `center + radius * (cos a, sin a)`, `a` from
    /// `start_angle` through `start_angle + sweep_angle`.
    CircleArc {
        /// Arc centre.
        center: [f64; 2],
        /// Arc radius, positive.
        radius: f64,
        /// Angle of the start point, radians.
        start_angle: f64,
        /// Signed angular span, radians; the sign gives the orientation.
        sweep_angle: f64,
    },
    /// A quadratic Bezier `p0`, `p1`, `p2`.
    QuadraticBezier {
        /// Start control point.
        p0: [f64; 2],
        /// Middle control point.
        p1: [f64; 2],
        /// End control point.
        p2: [f64; 2],
    },
    /// A cubic Bezier `p0`, `p1`, `p2`, `p3`.
    CubicBezier {
        /// First control point.
        p0: [f64; 2],
        /// Second control point.
        p1: [f64; 2],
        /// Third control point.
        p2: [f64; 2],
        /// Fourth control point.
        p3: [f64; 2],
    },
    /// A clamped B-spline: control points, clamped knot vector and degree.
    BSpline {
        /// Control points.
        control: Vec<[f64; 2]>,
        /// Knot vector, clamped: `degree + 1` copies at each end.
        knots: Vec<f64>,
        /// Polynomial degree, at least 1.
        degree: usize,
    },
}

impl CurveSpec {
    /// The natural parameter interval of the curve.
    ///
    /// # Errors
    ///
    /// [`GeometryError::InvalidCurve`] for inconsistent B-spline data.
    pub fn natural_interval(&self) -> Result<[f64; 2], GeometryError> {
        match self {
            Self::BSpline {
                control,
                knots,
                degree,
            } => {
                validate_bspline(control, knots, *degree)?;
                let lo = knot(knots, *degree)?;
                let hi = knot(knots, control.len())?;
                Ok([lo, hi])
            }
            _ => Ok([0.0, 1.0]),
        }
    }

    /// The differential data at the curve parameter `u`.
    ///
    /// The parameter is the curve's own parameter (line/Bezier/arc `[0, 1]`,
    /// B-spline its natural interval). A [`crate::rail_solver::Piece`] maps its
    /// piece parameter onto this interval.
    ///
    /// # Errors
    ///
    /// [`GeometryError::InvalidCurve`] for malformed data, and
    /// [`GeometryError::ZeroSpeed`] at a point with a zero first derivative.
    pub fn differential(&self, u: f64) -> Result<DifferentialData, GeometryError> {
        let (point, first, second) = match self {
            Self::Line { start, end } => {
                let [sx, sy] = *start;
                let [ex, ey] = *end;
                let d = [ex - sx, ey - sy];
                let p = add(*start, scale(d, u));
                (p, d, [0.0, 0.0])
            }
            Self::CircleArc {
                center,
                radius,
                start_angle,
                sweep_angle,
            } => {
                if !radius.is_finite() || *radius <= 0.0 || *sweep_angle == 0.0 {
                    return Err(GeometryError::ZeroSpeed);
                }
                if !start_angle.is_finite() || !sweep_angle.is_finite() {
                    return Err(GeometryError::InvalidCurve("non-finite arc angle"));
                }
                let [cx, cy] = *center;
                let da = *sweep_angle;
                let a = *start_angle + u * da;
                let (sa, ca) = a.sin_cos();
                let r = *radius;
                let p = [cx + r * ca, cy + r * sa];
                let d1 = [-r * da * sa, r * da * ca];
                let d2 = [-r * da * da * ca, -r * da * da * sa];
                (p, d1, d2)
            }
            Self::QuadraticBezier { p0, p1, p2 } => {
                let s = 1.0 - u;
                let p = add(
                    add(scale(*p0, s * s), scale(*p1, 2.0 * s * u)),
                    scale(*p2, u * u),
                );
                let d1 = add(scale(sub(*p1, *p0), 2.0 * s), scale(sub(*p2, *p1), 2.0 * u));
                let d2 = scale(add(add(*p2, scale(*p1, -2.0)), *p0), 2.0);
                (p, d1, d2)
            }
            Self::CubicBezier { p0, p1, p2, p3 } => {
                let s = 1.0 - u;
                let p = add(
                    add(scale(*p0, s * s * s), scale(*p1, 3.0 * s * s * u)),
                    add(scale(*p2, 3.0 * s * u * u), scale(*p3, u * u * u)),
                );
                let d1 = add(
                    add(
                        scale(sub(*p1, *p0), 3.0 * s * s),
                        scale(sub(*p2, *p1), 6.0 * s * u),
                    ),
                    scale(sub(*p3, *p2), 3.0 * u * u),
                );
                let d2 = add(
                    scale(add(add(*p2, scale(*p1, -2.0)), *p0), 6.0 * s),
                    scale(add(add(*p3, scale(*p2, -2.0)), *p1), 6.0 * u),
                );
                (p, d1, d2)
            }
            Self::BSpline {
                control,
                knots,
                degree,
            } => bspline_differential(control, knots, *degree, u)?,
        };

        let speed = norm(first);
        if !speed.is_finite() || speed <= 1.0e-15 {
            return Err(GeometryError::ZeroSpeed);
        }
        let tangent = unit(first).ok_or(GeometryError::ZeroSpeed)?;
        let normal = rot90(tangent);
        let curvature = cross(first, second) / (speed * speed * speed);
        if !curvature.is_finite() {
            return Err(GeometryError::InvalidCurve("non-finite curvature"));
        }
        Ok(DifferentialData {
            point,
            first,
            second,
            tangent,
            normal,
            curvature,
            speed,
        })
    }

    /// The same point set traversed in the opposite direction.
    ///
    /// # Errors
    ///
    /// [`GeometryError::InvalidCurve`] for malformed B-spline data.
    pub fn reversed(&self) -> Result<Self, GeometryError> {
        match self {
            Self::Line { start, end } => Ok(Self::Line {
                start: *end,
                end: *start,
            }),
            Self::CircleArc {
                center,
                radius,
                start_angle,
                sweep_angle,
            } => Ok(Self::CircleArc {
                center: *center,
                radius: *radius,
                start_angle: *start_angle + *sweep_angle,
                sweep_angle: -*sweep_angle,
            }),
            Self::QuadraticBezier { p0, p1, p2 } => Ok(Self::QuadraticBezier {
                p0: *p2,
                p1: *p1,
                p2: *p0,
            }),
            Self::CubicBezier { p0, p1, p2, p3 } => Ok(Self::CubicBezier {
                p0: *p3,
                p1: *p2,
                p2: *p1,
                p3: *p0,
            }),
            Self::BSpline {
                control,
                knots,
                degree,
            } => {
                validate_bspline(control, knots, *degree)?;
                let lo = *knots
                    .first()
                    .ok_or(GeometryError::InvalidCurve("empty B-spline knots"))?;
                let hi = *knots
                    .last()
                    .ok_or(GeometryError::InvalidCurve("empty B-spline knots"))?;
                let reversed_control = control.iter().rev().copied().collect();
                let reversed_knots = knots.iter().rev().map(|k| lo + hi - *k).collect();
                Ok(Self::BSpline {
                    control: reversed_control,
                    knots: reversed_knots,
                    degree: *degree,
                })
            }
        }
    }
}

fn knot(knots: &[f64], index: usize) -> Result<f64, GeometryError> {
    knots.get(index).copied().ok_or(GeometryError::InvalidCurve(
        "B-spline knot index out of range",
    ))
}

fn validate_bspline(
    control: &[[f64; 2]],
    knots: &[f64],
    degree: usize,
) -> Result<(), GeometryError> {
    if degree == 0 {
        return Err(GeometryError::InvalidCurve(
            "B-spline degree must be at least 1",
        ));
    }
    if control.len() < degree + 1 {
        return Err(GeometryError::InvalidCurve(
            "too few B-spline control points",
        ));
    }
    if knots.len() != control.len() + degree + 1 {
        return Err(GeometryError::InvalidCurve(
            "B-spline knot/control count mismatch",
        ));
    }
    if control.iter().any(|p| !p.iter().all(|v| v.is_finite())) {
        return Err(GeometryError::InvalidCurve(
            "non-finite B-spline control point",
        ));
    }
    if knots.iter().any(|k| !k.is_finite()) {
        return Err(GeometryError::InvalidCurve("non-finite B-spline knot"));
    }
    for pair in knots.windows(2) {
        let a = *pair
            .first()
            .ok_or(GeometryError::InvalidCurve("empty B-spline knot pair"))?;
        let b = *pair
            .get(1)
            .ok_or(GeometryError::InvalidCurve("empty B-spline knot pair"))?;
        if a > b {
            return Err(GeometryError::InvalidCurve("B-spline knots are not sorted"));
        }
    }
    let lo = knot(knots, degree)?;
    let hi = knot(knots, control.len())?;
    if hi <= lo {
        return Err(GeometryError::InvalidCurve("empty B-spline interval"));
    }
    Ok(())
}

fn find_span(n: usize, p: usize, u: f64, knots: &[f64]) -> Result<usize, GeometryError> {
    if n == 0 {
        return Ok(0);
    }
    let first = knot(knots, p)?;
    let last = knot(knots, n + 1)?;
    if u >= last {
        return Ok(n);
    }
    if u <= first {
        return Ok(p.min(n));
    }
    let mut low = p;
    let mut high = n + 1;
    let mut mid = usize::midpoint(low, high);
    let mut guard = 0usize;
    while u < knot(knots, mid)? || u >= knot(knots, mid + 1)? {
        if u < knot(knots, mid)? {
            high = mid;
        } else {
            low = mid;
        }
        mid = usize::midpoint(low, high);
        guard += 1;
        if guard > 1024 {
            return Err(GeometryError::InvalidCurve(
                "B-spline span search did not converge",
            ));
        }
    }
    Ok(mid)
}

fn eval_bspline(
    control: &[[f64; 2]],
    knots: &[f64],
    p: usize,
    u: f64,
) -> Result<[f64; 2], GeometryError> {
    let n = control
        .len()
        .checked_sub(1)
        .ok_or(GeometryError::InvalidCurve("empty B-spline control points"))?;
    let lo = knot(knots, p)?;
    let hi = knot(knots, n + 1)?;
    let u = u.clamp(lo, hi);
    let span = find_span(n, p, u, knots)?;
    let base = span
        .checked_sub(p)
        .ok_or(GeometryError::InvalidCurve("B-spline span underflow"))?;
    let mut d = Vec::with_capacity(p + 1);
    for i in 0..=p {
        let index = base.checked_add(i).ok_or(GeometryError::InvalidCurve(
            "B-spline control index overflow",
        ))?;
        d.push(*control.get(index).ok_or(GeometryError::InvalidCurve(
            "B-spline control index out of range",
        ))?);
    }
    for r in 1..=p {
        for j in (r..=p).rev() {
            let denominator = knot(knots, span + 1 + j - r)? - knot(knots, span - p + j)?;
            if denominator == 0.0 {
                return Err(GeometryError::InvalidCurve("zero B-spline knot span"));
            }
            let alpha = (u - knot(knots, span - p + j)?) / denominator;
            let previous = *d
                .get(j - 1)
                .ok_or(GeometryError::InvalidCurve("B-spline de Boor index"))?;
            let current = *d
                .get(j)
                .ok_or(GeometryError::InvalidCurve("B-spline de Boor index"))?;
            let next = add(scale(previous, 1.0 - alpha), scale(current, alpha));
            *d.get_mut(j)
                .ok_or(GeometryError::InvalidCurve("B-spline de Boor index"))? = next;
        }
    }
    d.get(p)
        .copied()
        .ok_or(GeometryError::InvalidCurve("B-spline result missing"))
}

fn derivative_curve(
    control: &[[f64; 2]],
    knots: &[f64],
    p: usize,
) -> Result<(Vec<[f64; 2]>, Vec<f64>, usize), GeometryError> {
    if p == 0 {
        return Err(GeometryError::InvalidCurve(
            "degree-zero B-spline has no derivative",
        ));
    }
    let count = control.len().saturating_sub(1);
    let mut q = Vec::with_capacity(count);
    for i in 0..count {
        let ci = *control
            .get(i)
            .ok_or(GeometryError::InvalidCurve("B-spline derivative control"))?;
        let cj = *control
            .get(i + 1)
            .ok_or(GeometryError::InvalidCurve("B-spline derivative control"))?;
        let denominator = knot(knots, i + p + 1)? - knot(knots, i + 1)?;
        if denominator == 0.0 {
            return Err(GeometryError::InvalidCurve(
                "zero B-spline derivative knot span",
            ));
        }
        q.push(scale(sub(cj, ci), p as f64 / denominator));
    }
    let k2 = knots
        .iter()
        .skip(1)
        .take(knots.len().saturating_sub(2))
        .copied()
        .collect();
    Ok((q, k2, p - 1))
}

fn bspline_differential(
    control: &[[f64; 2]],
    knots: &[f64],
    p: usize,
    u: f64,
) -> Result<([f64; 2], [f64; 2], [f64; 2]), GeometryError> {
    validate_bspline(control, knots, p)?;
    let point = eval_bspline(control, knots, p, u)?;
    let (c1, k1, p1) = derivative_curve(control, knots, p)?;
    let first = eval_bspline(&c1, &k1, p1, u)?;
    let second = if p >= 2 {
        let (c2, k2, p2) = derivative_curve(&c1, &k1, p1)?;
        eval_bspline(&c2, &k2, p2, u)?
    } else {
        [0.0, 0.0]
    };
    Ok((point, first, second))
}

impl crate::rail_solver::PlanarCurve for CurveSpec {
    fn natural_interval(&self) -> Result<[f64; 2], GeometryError> {
        CurveSpec::natural_interval(self)
    }

    fn differential(&self, u: f64) -> Result<DifferentialData, GeometryError> {
        CurveSpec::differential(self, u)
    }
}
