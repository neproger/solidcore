//! The rail system and its continuation.
//!
//! For two extended path pieces and a material offset `alpha`:
//!
//! ```text
//! G(v, w, alpha) = P1(v) + alpha n1(v) - P2(w) - alpha n2(w) = 0
//! X(alpha)       = P1(v) + alpha n1(v) = P2(w) + alpha n2(w)
//! ```
//!
//! The extension is the production `SM_CT_G1` policy: a straight tangent
//! segment at the joint. The solver traces both directions from `alpha = 0`
//! and reports folds, curvature cusps, the piece domain and non-convergence
//! explicitly; it never falls back to a straight line.

use core::fmt;

use crate::curve::GeometryError;
use crate::differential_geometry::{
    add, cross, dist, dot, norm, rot90, scale, sub, unit, DifferentialData,
};

/// A planar curve the rail solver can evaluate: the Level A `CurveSpec` or the
/// kernel `KernelCurve` adapter (feature `solidcore`).
pub trait PlanarCurve {
    /// The natural parameter interval.
    ///
    /// # Errors
    ///
    /// [`GeometryError`] for malformed curve data.
    fn natural_interval(&self) -> Result<[f64; 2], GeometryError>;
    /// Point, derivatives, tangent, normal, curvature and speed at `u`.
    ///
    /// # Errors
    ///
    /// [`GeometryError`] for malformed data or a zero-speed point.
    fn differential(&self, u: f64) -> Result<DifferentialData, GeometryError>;
}

const CUSP_TOL: f64 = 1.0e-6;
const FOLD_TOL: f64 = 1.0e-4;
const MIN_STEP: f64 = 1.0e-10;
const MAX_ITER: usize = 60;
const CLOSEST_SAMPLES: usize = 256;
const GOLDEN_ITERATIONS: usize = 100;
const TRACE_GUARD: usize = 20_000;

/// Which part of an extended piece produced a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PieceSegment {
    /// The original curve.
    Original,
    /// The straight tangent extension before the piece start.
    StartExtension,
    /// The straight tangent extension after the piece end.
    EndExtension,
}

/// One solution of the rail system at a fixed `alpha`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RailSolution {
    /// The material offset along the profile's `alpha` axis.
    pub alpha: f64,
    /// Parameter on the first piece.
    pub v: f64,
    /// Parameter on the second piece.
    pub w: f64,
    /// The miter point `X(alpha)` in the path plane.
    pub point: [f64; 2],
    /// The residual `G(v, w, alpha)`.
    pub residual: [f64; 2],
    /// The determinant of the 2x2 Jacobian at the solution.
    pub determinant: f64,
    /// Differential data at `v`.
    pub data_a: DifferentialData,
    /// Differential data at `w`.
    pub data_b: DifferentialData,
    /// Which segment of the first piece produced the point.
    pub segment_a: PieceSegment,
    /// Which segment of the second piece produced the point.
    pub segment_b: PieceSegment,
    /// Newton iterations spent on this sample (0 at the corner).
    pub iterations: usize,
    /// The adaptive alpha step that reached this sample (0 at the corner or in
    /// a uniform `solve_sequence` grid).
    pub step: f64,
}

/// Why a trace of the rail branch stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RailStop {
    /// The two offset rails became tangent (`t1` parallel to `t2`): a fold.
    Fold,
    /// The offset rail reached its curvature cusp (`1 - alpha kappa = 0`).
    CurvatureCusp,
    /// The seam left the extended piece.
    OutsidePiece,
    /// The profile's `alpha` interval ended normally.
    ReachedAlphaBound,
    /// The two pieces are tangent at the joint; no miter exists.
    TangentJoin,
    /// The Newton iteration did not converge away from a known singularity.
    NoConvergence,
}

/// A traced branch of the miter curve, both directions from `alpha = 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct RailTrace {
    /// The solutions, sorted by `alpha`.
    pub points: Vec<RailSolution>,
    /// The stop of the negative branch.
    pub negative_stop: Option<RailStop>,
    /// The stop of the positive branch.
    pub positive_stop: Option<RailStop>,
    /// Total Newton iterations spent on the trace.
    pub iterations: usize,
    /// The characteristic length used for relative residuals.
    pub characteristic_scale: f64,
    /// The smallest adaptive step (0 if no step was taken).
    pub min_step: f64,
    /// The largest adaptive step (0 if no step was taken).
    pub max_step: f64,
    /// The largest Newton iteration count of one step.
    pub max_iterations: usize,
}

/// An explicit failure of [`solve_sequence`].
#[derive(Clone, Debug, PartialEq)]
pub enum RailSolveError {
    /// Malformed curve data.
    Geometry(GeometryError),
    /// The seam left the extended piece.
    OutsidePiece,
    /// A singular Jacobian.
    Singular {
        /// The parameter where it was detected.
        alpha: f64,
        /// The determinant.
        determinant: f64,
    },
    /// The Newton iteration did not converge.
    NoConvergence {
        /// The parameter where it was detected.
        alpha: f64,
        /// The residual norm.
        residual: f64,
    },
}

impl fmt::Display for RailSolveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Geometry(error) => write!(f, "{error}"),
            Self::OutsidePiece => f.write_str("seam outside the extended piece"),
            Self::Singular { alpha, determinant } => {
                write!(
                    f,
                    "singular rail Jacobian at alpha={alpha}: det={determinant}"
                )
            }
            Self::NoConvergence { alpha, residual } => {
                write!(
                    f,
                    "rail Newton did not converge at alpha={alpha}: residual={residual}"
                )
            }
        }
    }
}

impl std::error::Error for RailSolveError {}

impl From<GeometryError> for RailSolveError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum NewtonError {
    OutsidePiece,
    Singular { determinant: f64 },
    NoConvergence { residual: f64 },
    Invalid(GeometryError),
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct NewtonOutcome {
    v: f64,
    w: f64,
    residual: [f64; 2],
    determinant: f64,
    data_a: DifferentialData,
    data_b: DifferentialData,
    segment_a: PieceSegment,
    segment_b: PieceSegment,
    iterations: usize,
}

/// One extended path piece: the original curve plus straight tangent
/// extensions at the ends selected by the caller.
///
/// The piece parameter `s` is `[param_lo, param_hi]` for the original curve.
/// `[param_lo - 1, param_lo]` is the start extension and `[param_hi,
/// param_hi + 1]` the end extension; both are straight lines of the requested
/// length. The map from `s` to the curve's own parameter is affine.
#[derive(Clone, Debug)]
pub struct Piece<C: PlanarCurve> {
    spec: C,
    curve_lo: f64,
    curve_hi: f64,
    param_lo: f64,
    param_hi: f64,
    ext_start: f64,
    ext_end: f64,
    start_point: [f64; 2],
    start_tangent: [f64; 2],
    end_point: [f64; 2],
    end_tangent: [f64; 2],
}

impl<C: PlanarCurve> Piece<C> {
    /// Build a piece on the curve's natural parameter interval.
    ///
    /// # Errors
    ///
    /// [`GeometryError`] for malformed curve data or a zero tangent at an end.
    pub fn new(spec: C, ext_start: f64, ext_end: f64) -> Result<Self, GeometryError> {
        let [lo, hi] = spec.natural_interval()?;
        Self::with_parameter_map(spec, lo, hi, ext_start, ext_end)
    }

    /// Build a piece with an explicit piece-parameter interval `[param_lo,
    /// param_hi]` mapped affinely onto the curve's natural interval.
    ///
    /// # Errors
    ///
    /// [`GeometryError`] for malformed curve data, an empty parameter interval
    /// or a negative/non-finite extension.
    pub fn with_parameter_map(
        spec: C,
        param_lo: f64,
        param_hi: f64,
        ext_start: f64,
        ext_end: f64,
    ) -> Result<Self, GeometryError> {
        if !param_lo.is_finite() || !param_hi.is_finite() || param_hi <= param_lo {
            return Err(GeometryError::InvalidCurve(
                "empty piece parameter interval",
            ));
        }
        if !ext_start.is_finite() || !ext_end.is_finite() || ext_start < 0.0 || ext_end < 0.0 {
            return Err(GeometryError::InvalidCurve("invalid piece extension"));
        }
        let [curve_lo, curve_hi] = spec.natural_interval()?;
        if curve_hi <= curve_lo {
            return Err(GeometryError::InvalidCurve("empty curve interval"));
        }
        let start = spec.differential(curve_lo)?;
        let end = spec.differential(curve_hi)?;
        Ok(Self {
            spec,
            curve_lo,
            curve_hi,
            param_lo,
            param_hi,
            ext_start,
            ext_end,
            start_point: start.point,
            start_tangent: start.tangent,
            end_point: end.point,
            end_tangent: end.tangent,
        })
    }

    /// The piece parameter at the end of the original curve (the joint for the
    /// incoming piece).
    pub fn corner_end(&self) -> f64 {
        self.param_hi
    }

    /// The piece parameter at the start of the original curve (the joint for
    /// the outgoing piece).
    pub fn corner_start(&self) -> f64 {
        self.param_lo
    }

    /// The extended parameter domain.
    pub fn domain(&self) -> [f64; 2] {
        [
            self.param_lo - if self.ext_start > 0.0 { 1.0 } else { 0.0 },
            self.param_hi + if self.ext_end > 0.0 { 1.0 } else { 0.0 },
        ]
    }

    /// Differential data at the piece parameter `s`.
    ///
    /// # Errors
    ///
    /// [`GeometryError::OutsideCurve`] outside the extended domain,
    /// [`GeometryError::ZeroSpeed`] at a degenerate point.
    pub fn differential(&self, s: f64) -> Result<(DifferentialData, PieceSegment), GeometryError> {
        if s < self.param_lo {
            return self.start_extension(s);
        }
        if s > self.param_hi {
            return self.end_extension(s);
        }
        let span = self.param_hi - self.param_lo;
        let map = (self.curve_hi - self.curve_lo) / span;
        let u = self.curve_lo + (s - self.param_lo) * map;
        let mut data = self.spec.differential(u)?;
        data.first = scale(data.first, map);
        data.second = scale(data.second, map * map);
        data.speed = norm(data.first);
        if !data.speed.is_finite() || data.speed <= 1.0e-15 {
            return Err(GeometryError::ZeroSpeed);
        }
        data.tangent = unit(data.first).ok_or(GeometryError::ZeroSpeed)?;
        data.normal = rot90(data.tangent);
        data.curvature = cross(data.first, data.second) / (data.speed * data.speed * data.speed);
        Ok((data, PieceSegment::Original))
    }

    fn start_extension(&self, s: f64) -> Result<(DifferentialData, PieceSegment), GeometryError> {
        if self.ext_start <= 0.0 {
            return Err(GeometryError::OutsideCurve);
        }
        let tau = s - (self.param_lo - 1.0);
        if !(0.0..=1.0).contains(&tau) {
            return Err(GeometryError::OutsideCurve);
        }
        let point = add(
            self.start_point,
            scale(self.start_tangent, (tau - 1.0) * self.ext_start),
        );
        let first = scale(self.start_tangent, self.ext_start);
        Ok((
            DifferentialData {
                point,
                first,
                second: [0.0, 0.0],
                tangent: self.start_tangent,
                normal: rot90(self.start_tangent),
                curvature: 0.0,
                speed: self.ext_start,
            },
            PieceSegment::StartExtension,
        ))
    }

    fn end_extension(&self, s: f64) -> Result<(DifferentialData, PieceSegment), GeometryError> {
        if self.ext_end <= 0.0 {
            return Err(GeometryError::OutsideCurve);
        }
        let tau = s - self.param_hi;
        if !(0.0..=1.0).contains(&tau) {
            return Err(GeometryError::OutsideCurve);
        }
        let point = add(self.end_point, scale(self.end_tangent, tau * self.ext_end));
        let first = scale(self.end_tangent, self.ext_end);
        Ok((
            DifferentialData {
                point,
                first,
                second: [0.0, 0.0],
                tangent: self.end_tangent,
                normal: rot90(self.end_tangent),
                curvature: 0.0,
                speed: self.ext_end,
            },
            PieceSegment::EndExtension,
        ))
    }

    /// The closest point of the extended piece to `x`, as a signed distance
    /// (positive on the left), its differential data and its segment.
    ///
    /// This is the independent check used by [`crate::validation`]: it does not
    /// use the rail system or its Jacobian.
    ///
    /// # Errors
    ///
    /// [`GeometryError`] for a malformed piece or no valid sample.
    pub fn closest(
        &self,
        x: [f64; 2],
    ) -> Result<(f64, DifferentialData, PieceSegment), GeometryError> {
        let [lo, hi] = self.domain();
        let width = hi - lo;
        if width <= 0.0 {
            return Err(GeometryError::InvalidCurve("empty extended domain"));
        }
        let mut best_s = lo;
        let mut best_d = f64::INFINITY;
        for i in 0..=CLOSEST_SAMPLES {
            let t = i as f64 / CLOSEST_SAMPLES as f64;
            let s = lo + width * t;
            if let Ok((data, _)) = self.differential(s) {
                let d = dist(data.point, x);
                if d < best_d {
                    best_d = d;
                    best_s = s;
                }
            }
        }
        if !best_d.is_finite() {
            return Err(GeometryError::InvalidCurve("no valid closest-point sample"));
        }
        let sample_step = width / CLOSEST_SAMPLES as f64;
        let a = (best_s - sample_step).max(lo);
        let b = (best_s + sample_step).min(hi);
        let mut s = golden(
            |t| {
                self.differential(t)
                    .map_or(f64::INFINITY, |(data, _)| dist(data.point, x))
            },
            a,
            b,
            GOLDEN_ITERATIONS,
        );
        for _ in 0..10 {
            let Ok((data, _)) = self.differential(s) else {
                break;
            };
            let difference = sub(data.point, x);
            let g = dot(difference, data.first);
            let gp = dot(data.first, data.first) + dot(difference, data.second);
            if gp.abs() < 1.0e-18 {
                break;
            }
            let next = (s - g / gp).clamp(lo, hi);
            if !next.is_finite() {
                break;
            }
            let change = (next - s).abs();
            s = next;
            if change < 1.0e-15 {
                break;
            }
        }
        let (data, segment) = self.differential(s)?;
        let signed = dot(sub(x, data.point), data.normal);
        Ok((signed, data, segment))
    }
}

fn golden<F: Fn(f64) -> f64>(f: F, mut a: f64, mut b: f64, iterations: usize) -> f64 {
    let ratio = (5.0_f64.sqrt() - 1.0) * 0.5;
    let mut c = b - ratio * (b - a);
    let mut d = a + ratio * (b - a);
    let mut fc = f(c);
    let mut fd = f(d);
    for _ in 0..iterations {
        if (b - a).abs() < 1.0e-15 {
            break;
        }
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - ratio * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + ratio * (b - a);
            fd = f(d);
        }
    }
    (a + b) * 0.5
}

fn newton<C1: PlanarCurve, C2: PlanarCurve>(
    p1: &Piece<C1>,
    p2: &Piece<C2>,
    alpha: f64,
    v0: f64,
    w0: f64,
    tol: f64,
    det_eps: f64,
) -> Result<NewtonOutcome, NewtonError> {
    let mut v = v0;
    let mut w = w0;
    let mut iterations = 0usize;
    for _ in 0..MAX_ITER {
        let (data_a, segment_a) = match p1.differential(v) {
            Ok(value) => value,
            Err(GeometryError::OutsideCurve) => return Err(NewtonError::OutsidePiece),
            Err(error) => return Err(NewtonError::Invalid(error)),
        };
        let (data_b, segment_b) = match p2.differential(w) {
            Ok(value) => value,
            Err(GeometryError::OutsideCurve) => return Err(NewtonError::OutsidePiece),
            Err(error) => return Err(NewtonError::Invalid(error)),
        };
        let point_a = add(data_a.point, scale(data_a.normal, alpha));
        let point_b = add(data_b.point, scale(data_b.normal, alpha));
        let residual = sub(point_a, point_b);
        let residual_norm = norm(residual);
        if residual_norm <= tol {
            let jv = scale(data_a.first, 1.0 - alpha * data_a.curvature);
            let jw = scale(data_b.first, -(1.0 - alpha * data_b.curvature));
            return Ok(NewtonOutcome {
                v,
                w,
                residual,
                determinant: cross(jv, jw),
                data_a,
                data_b,
                segment_a,
                segment_b,
                iterations,
            });
        }
        let jv = scale(data_a.first, 1.0 - alpha * data_a.curvature);
        let jw = scale(data_b.first, -(1.0 - alpha * data_b.curvature));
        let determinant = cross(jv, jw);
        if determinant.abs() <= det_eps {
            return Err(NewtonError::Singular { determinant });
        }
        let minus = scale(residual, -1.0);
        let dv = cross(minus, jw) / determinant;
        let dw = cross(jv, minus) / determinant;
        let mut step = 1.0;
        let mut accepted = false;
        for _ in 0..24 {
            let vn = v + step * dv;
            let wn = w + step * dw;
            let trial = match (p1.differential(vn), p2.differential(wn)) {
                (Ok((da, _)), Ok((db, _))) => Some((da, db)),
                _ => None,
            };
            if let Some((da, db)) = trial {
                let ga = add(da.point, scale(da.normal, alpha));
                let gb = add(db.point, scale(db.normal, alpha));
                let gn = norm(sub(ga, gb));
                if gn < residual_norm || gn <= tol {
                    v = vn;
                    w = wn;
                    accepted = true;
                    break;
                }
            }
            step *= 0.5;
        }
        iterations += 1;
        if !accepted {
            return Err(NewtonError::NoConvergence {
                residual: residual_norm,
            });
        }
    }
    Err(NewtonError::NoConvergence { residual: f64::NAN })
}

fn classify_stop(alpha: f64, data_a: &DifferentialData, data_b: &DifferentialData) -> RailStop {
    let cusp = (1.0 - alpha * data_a.curvature)
        .abs()
        .min((1.0 - alpha * data_b.curvature).abs());
    let fold = cross(data_a.tangent, data_b.tangent).abs();
    if cusp < CUSP_TOL {
        RailStop::CurvatureCusp
    } else if fold < FOLD_TOL {
        RailStop::Fold
    } else {
        RailStop::NoConvergence
    }
}

fn corner_solution<C1: PlanarCurve, C2: PlanarCurve>(
    p1: &Piece<C1>,
    p2: &Piece<C2>,
    determinant: f64,
) -> Result<RailSolution, GeometryError> {
    let (data_a, segment_a) = p1.differential(p1.corner_end())?;
    let (data_b, segment_b) = p2.differential(p2.corner_start())?;
    Ok(RailSolution {
        alpha: 0.0,
        v: p1.corner_end(),
        w: p2.corner_start(),
        point: data_a.point,
        residual: [0.0, 0.0],
        determinant,
        data_a,
        data_b,
        segment_a,
        segment_b,
        iterations: 0,
        step: 0.0,
    })
}

fn trace_direction<C1: PlanarCurve, C2: PlanarCurve>(
    p1: &Piece<C1>,
    p2: &Piece<C2>,
    sign: f64,
    alpha_min: f64,
    alpha_max: f64,
    start_v: f64,
    start_w: f64,
    dv: f64,
    dw: f64,
    tol: f64,
    det_eps: f64,
    initial_step: f64,
) -> (Vec<RailSolution>, RailStop, usize, f64, f64, usize) {
    let mut points = Vec::new();
    let mut v = start_v;
    let mut w = start_w;
    let mut alpha = 0.0;
    let mut dv = dv;
    let mut dw = dw;
    let mut step = initial_step;
    let mut iterations = 0usize;
    let mut min_step = f64::INFINITY;
    let mut max_step = 0.0_f64;
    let mut max_iterations = 0usize;
    let mut stop = RailStop::NoConvergence;
    let mut previous_data: Option<(DifferentialData, DifferentialData)> = None;
    for _ in 0..TRACE_GUARD {
        let mut target = alpha + sign * step;
        let mut clamped = false;
        if sign > 0.0 && target > alpha_max {
            target = alpha_max;
            clamped = true;
        }
        if sign < 0.0 && target < alpha_min {
            target = alpha_min;
            clamped = true;
        }
        let delta = target - alpha;
        if delta.abs() < 1.0e-15 {
            stop = RailStop::ReachedAlphaBound;
            break;
        }
        let predicted_v = v + dv * delta;
        let predicted_w = w + dw * delta;
        match newton(p1, p2, target, predicted_v, predicted_w, tol, det_eps) {
            Ok(outcome) => {
                iterations += outcome.iterations;
                let step_size = delta.abs();
                min_step = min_step.min(step_size);
                max_step = max_step.max(step_size);
                max_iterations = max_iterations.max(outcome.iterations);
                let point = add(outcome.data_a.point, scale(outcome.data_a.normal, target));
                points.push(RailSolution {
                    alpha: target,
                    v: outcome.v,
                    w: outcome.w,
                    point,
                    residual: outcome.residual,
                    determinant: outcome.determinant,
                    data_a: outcome.data_a,
                    data_b: outcome.data_b,
                    segment_a: outcome.segment_a,
                    segment_b: outcome.segment_b,
                    iterations: outcome.iterations,
                    step: step_size,
                });
                v = outcome.v;
                w = outcome.w;
                alpha = target;
                previous_data = Some((outcome.data_a, outcome.data_b));
                let cusp = (1.0 - alpha * outcome.data_a.curvature)
                    .abs()
                    .min((1.0 - alpha * outcome.data_b.curvature).abs());
                let fold = cross(outcome.data_a.tangent, outcome.data_b.tangent).abs();
                if cusp < CUSP_TOL {
                    stop = RailStop::CurvatureCusp;
                    break;
                }
                if fold < FOLD_TOL {
                    stop = RailStop::Fold;
                    break;
                }
                let jv = scale(outcome.data_a.first, 1.0 - alpha * outcome.data_a.curvature);
                let jw = scale(
                    outcome.data_b.first,
                    -(1.0 - alpha * outcome.data_b.curvature),
                );
                let determinant = cross(jv, jw);
                if determinant.abs() <= det_eps {
                    stop = classify_stop(alpha, &outcome.data_a, &outcome.data_b);
                    break;
                }
                let rhs = sub(outcome.data_b.normal, outcome.data_a.normal);
                dv = cross(rhs, jw) / determinant;
                dw = cross(jv, rhs) / determinant;
                if clamped {
                    stop = RailStop::ReachedAlphaBound;
                    break;
                }
                step = (step * 1.5).min((alpha_max - alpha_min).abs() * 0.25 + 1.0e-6);
            }
            Err(NewtonError::OutsidePiece) => {
                if step > MIN_STEP {
                    step *= 0.5;
                    continue;
                }
                stop = if let Some((data_a, data_b)) = previous_data {
                    match classify_stop(alpha, &data_a, &data_b) {
                        RailStop::Fold => RailStop::Fold,
                        RailStop::CurvatureCusp => RailStop::CurvatureCusp,
                        _ => RailStop::OutsidePiece,
                    }
                } else {
                    RailStop::OutsidePiece
                };
                break;
            }
            Err(NewtonError::Invalid(_)) => {
                stop = RailStop::NoConvergence;
                break;
            }
            Err(
                NewtonError::Singular { determinant: _ }
                | NewtonError::NoConvergence { residual: _ },
            ) => {
                if step > MIN_STEP {
                    step *= 0.5;
                    continue;
                }
                stop = if let Some((data_a, data_b)) = previous_data {
                    classify_stop(alpha, &data_a, &data_b)
                } else {
                    RailStop::NoConvergence
                };
                break;
            }
        }
    }
    (
        points,
        stop,
        iterations,
        if min_step.is_finite() { min_step } else { 0.0 },
        max_step,
        max_iterations,
    )
}

/// Trace the miter curve from `alpha = 0` to both bounds.
///
/// Each direction stops independently; a fold or an out-of-domain stop on one
/// side does not abort the other.
///
/// # Errors
///
/// [`GeometryError`] for malformed pieces, a corner where the pieces do not
/// meet, or invalid alpha bounds.
pub fn trace<C1: PlanarCurve, C2: PlanarCurve>(
    p1: &Piece<C1>,
    p2: &Piece<C2>,
    alpha_min: f64,
    alpha_max: f64,
) -> Result<RailTrace, GeometryError> {
    if !alpha_min.is_finite() || !alpha_max.is_finite() || alpha_min > 0.0 || alpha_max < 0.0 {
        return Err(GeometryError::InvalidCurve(
            "trace bounds must contain alpha = 0",
        ));
    }
    let corner_a = p1.differential(p1.corner_end())?.0.point;
    let corner_b = p2.differential(p2.corner_start())?.0.point;
    let corner_scale = norm(corner_a).max(1.0);
    if dist(corner_a, corner_b) > 1.0e-9 * corner_scale {
        return Err(GeometryError::InvalidCurve(
            "the pieces do not meet at the corner",
        ));
    }
    let magnitude = corner_scale.max(alpha_min.abs()).max(alpha_max.abs());
    let tol = 1.0e-13 * magnitude;
    let det_eps = 1.0e-14 * magnitude * magnitude;
    let (start_a, _) = p1.differential(p1.corner_end())?;
    let (start_b, _) = p2.differential(p2.corner_start())?;
    let jv = start_a.first;
    let jw = scale(start_b.first, -1.0);
    let rhs = sub(start_b.normal, start_a.normal);
    let determinant = cross(jv, jw);
    let initial = corner_solution(p1, p2, determinant)?;
    if determinant.abs() <= det_eps {
        return Ok(RailTrace {
            points: vec![initial],
            negative_stop: Some(RailStop::TangentJoin),
            positive_stop: Some(RailStop::TangentJoin),
            iterations: 0,
            characteristic_scale: magnitude,
            min_step: 0.0,
            max_step: 0.0,
            max_iterations: 0,
        });
    }
    let dv = cross(rhs, jw) / determinant;
    let dw = cross(jv, rhs) / determinant;
    let range = alpha_max - alpha_min;
    let initial_step = (range / 50.0).clamp(1.0e-6, 0.05);
    let (
        negative_points,
        negative_stop,
        negative_iterations,
        negative_min_step,
        negative_max_step,
        negative_max_iterations,
    ) = trace_direction(
        p1,
        p2,
        -1.0,
        alpha_min,
        alpha_max,
        p1.corner_end(),
        p2.corner_start(),
        dv,
        dw,
        tol,
        det_eps,
        initial_step,
    );
    let (
        positive_points,
        positive_stop,
        positive_iterations,
        positive_min_step,
        positive_max_step,
        positive_max_iterations,
    ) = trace_direction(
        p1,
        p2,
        1.0,
        alpha_min,
        alpha_max,
        p1.corner_end(),
        p2.corner_start(),
        dv,
        dw,
        tol,
        det_eps,
        initial_step,
    );
    let mut points = Vec::with_capacity(1 + negative_points.len() + positive_points.len());
    points.push(initial);
    points.extend(negative_points);
    points.extend(positive_points);
    points.sort_by(|a, b| a.alpha.total_cmp(&b.alpha));
    Ok(RailTrace {
        points,
        negative_stop: Some(negative_stop),
        positive_stop: Some(positive_stop),
        iterations: negative_iterations + positive_iterations,
        characteristic_scale: magnitude,
        min_step: if negative_min_step > 0.0 && positive_min_step > 0.0 {
            negative_min_step.min(positive_min_step)
        } else if negative_min_step > 0.0 {
            negative_min_step
        } else if positive_min_step > 0.0 {
            positive_min_step
        } else {
            0.0
        },
        max_step: negative_max_step.max(positive_max_step),
        max_iterations: negative_max_iterations.max(positive_max_iterations),
    })
}

/// Solve the rail system at a fixed list of `alpha` values, using the previous
/// solution of the same sign as the initial guess.
///
/// This is the uniform-grid path used by the invariance tests.
///
/// # Errors
///
/// [`RailSolveError`] for a malformed piece or a point that cannot be solved.
pub fn solve_sequence<C1: PlanarCurve, C2: PlanarCurve>(
    p1: &Piece<C1>,
    p2: &Piece<C2>,
    alphas: &[f64],
) -> Result<Vec<RailSolution>, RailSolveError> {
    let corner_a = p1.differential(p1.corner_end())?.0.point;
    let corner_b = p2.differential(p2.corner_start())?.0.point;
    let corner_scale = norm(corner_a).max(1.0);
    if dist(corner_a, corner_b) > 1.0e-9 * corner_scale {
        return Err(RailSolveError::Geometry(GeometryError::InvalidCurve(
            "the pieces do not meet at the corner",
        )));
    }
    let max_alpha = alphas.iter().map(|a| a.abs()).fold(0.0_f64, f64::max);
    let magnitude = corner_scale.max(max_alpha);
    let tol = 1.0e-13 * magnitude;
    let det_eps = 1.0e-14 * magnitude * magnitude;
    let mut result = Vec::new();
    for sign in [-1.0, 1.0] {
        let mut items: Vec<f64> = alphas.iter().copied().filter(|a| a * sign > 0.0).collect();
        items.sort_by(|a, b| a.abs().total_cmp(&b.abs()));
        let mut v = p1.corner_end();
        let mut w = p2.corner_start();
        for alpha in items {
            let outcome =
                newton(p1, p2, alpha, v, w, tol, det_eps).map_err(|error| match error {
                    NewtonError::OutsidePiece => RailSolveError::OutsidePiece,
                    NewtonError::Singular { determinant } => {
                        RailSolveError::Singular { alpha, determinant }
                    }
                    NewtonError::NoConvergence { residual } => {
                        RailSolveError::NoConvergence { alpha, residual }
                    }
                    NewtonError::Invalid(error) => RailSolveError::Geometry(error),
                })?;
            v = outcome.v;
            w = outcome.w;
            result.push(RailSolution {
                alpha,
                v: outcome.v,
                w: outcome.w,
                point: add(outcome.data_a.point, scale(outcome.data_a.normal, alpha)),
                residual: outcome.residual,
                determinant: outcome.determinant,
                data_a: outcome.data_a,
                data_b: outcome.data_b,
                segment_a: outcome.segment_a,
                segment_b: outcome.segment_b,
                iterations: outcome.iterations,
                step: 0.0,
            });
        }
    }
    if alphas.contains(&0.0) {
        result.push(corner_solution(p1, p2, 0.0)?);
    }
    result.sort_by(|a, b| a.alpha.total_cmp(&b.alpha));
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn data(curvature: f64, tangent: [f64; 2]) -> DifferentialData {
        DifferentialData {
            point: [0.0, 0.0],
            first: tangent,
            second: [0.0, 0.0],
            tangent,
            normal: rot90(tangent),
            curvature,
            speed: 1.0,
        }
    }

    #[test]
    fn stop_classifier_distinguishes_cusp_fold_and_non_convergence() {
        let tangent_a = data(1.0, [1.0, 0.0]);
        let tangent_b = data(1.0, [0.0, 1.0]);
        assert_eq!(
            classify_stop(1.0, &tangent_a, &tangent_b),
            RailStop::CurvatureCusp
        );
        let parallel_b = data(0.0, [1.0, 0.0]);
        assert_eq!(classify_stop(0.5, &tangent_a, &parallel_b), RailStop::Fold);
        assert_eq!(
            classify_stop(0.5, &tangent_a, &tangent_b),
            RailStop::NoConvergence
        );
    }
}
