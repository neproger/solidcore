//! The miter curve and its material coordinate.

/// One point of the miter curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MiterSample {
    /// The profile's material offset along the path-plane normal.
    pub alpha: f64,
    /// The point `X(alpha)` in the path plane.
    pub point: [f64; 2],
}

/// A sampled miter curve in the path plane.
///
/// With the WR-407 frame the full miter surface is this curve extruded along
/// the path-plane normal `N`; `beta` is a common translation.
#[derive(Clone, Debug, PartialEq)]
pub struct MiterCurve {
    /// The samples, sorted by `alpha`.
    pub samples: Vec<MiterSample>,
}
