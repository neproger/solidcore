//! Planar curve data used by the mathematical prototype.
//!
//! Deliberately independent of `solidcore::curve::Curve`: level A must be
//! testable without the kernel. Level B converts these specs to the real types
//! in [`crate::solidcore_api`] (feature `solidcore`).

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
        /// Arc radius.
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
    ///
    /// A rational representation (NURBS) is intentionally not a separate
    /// variant yet. The model only needs point, first and second derivatives;
    /// a rational curve can be added when the integration level needs it.
    BSpline {
        /// Control points.
        control: Vec<[f64; 2]>,
        /// Knot vector, clamped: `degree + 1` copies at each end.
        knots: Vec<f64>,
        /// Polynomial degree.
        degree: usize,
    },
}
