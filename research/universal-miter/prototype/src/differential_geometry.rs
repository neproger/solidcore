//! Differential data of a planar curve at one parameter.
//!
//! The rail system needs only the point, the first and second derivatives, the
//! unit tangent, the left normal and the signed curvature. Every curve family
//! in the test matrix provides these through its generic evaluator; no branch
//! on the segment type is needed by the miter solver.

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

// `[f64; 2]` is the public point type; these helpers keep the implementation
// free of unchecked indexing.

pub(crate) fn add(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let [ax, ay] = a;
    let [bx, by] = b;
    [ax + bx, ay + by]
}

pub(crate) fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let [ax, ay] = a;
    let [bx, by] = b;
    [ax - bx, ay - by]
}

pub(crate) fn scale(a: [f64; 2], s: f64) -> [f64; 2] {
    let [x, y] = a;
    [x * s, y * s]
}

pub(crate) fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    let [ax, ay] = a;
    let [bx, by] = b;
    ax * bx + ay * by
}

pub(crate) fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    let [ax, ay] = a;
    let [bx, by] = b;
    ax * by - ay * bx
}

pub(crate) fn norm(a: [f64; 2]) -> f64 {
    dot(a, a).sqrt()
}

pub(crate) fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    norm(sub(a, b))
}

pub(crate) fn unit(a: [f64; 2]) -> Option<[f64; 2]> {
    let n = norm(a);
    let [x, y] = a;
    if n > 1.0e-15 {
        Some([x / n, y / n])
    } else {
        None
    }
}

pub(crate) fn rot90(a: [f64; 2]) -> [f64; 2] {
    let [x, y] = a;
    [-y, x]
}
