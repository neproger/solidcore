//! Differential data of a planar curve at one parameter.
//!
//! The rail system needs only the point, the first and second derivatives, the
//! unit tangent, the left normal and the signed curvature. Every curve family
//! in the test matrix provides these through its generic evaluator; no branch
//! on the segment type is needed.

/// The data the rail solver needs at one curve parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DifferentialData {
    /// The curve point `P(v)`.
    pub point: [f64; 2],
    /// The first derivative `P'(v)`; not normalised, so the parameter speed is
    /// preserved for the Jacobian.
    pub first: [f64; 2],
    /// The second derivative `P''(v)`.
    pub second: [f64; 2],
    /// The unit tangent `P'/|P'|`.
    pub tangent: [f64; 2],
    /// The left normal `N x tangent` in the path plane (`N = +z`).
    pub normal: [f64; 2],
    /// The signed curvature `cross(P', P'') / |P'|^3`, so that `t' = kappa n`.
    pub curvature: f64,
    /// The parameter speed `|P'|`.
    pub speed: f64,
}
