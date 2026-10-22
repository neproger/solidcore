//! Independent AOIM Line–Arc analytical geometry experiment.
//!
//! This is NOT connected to solidcore's B-rep builder. It studies the
//! supporting-curve algebra behind the proposed Analytical Offset-Intersection
//! Miter (AOIM).
//!
//! # Conventions
//!
//! * The joint is the origin. The incoming segment travels along `+x` and is
//!   supported by the line `y = 0`.
//! * The outgoing arc starts at the joint with tangent `(cos α, -sin α)`, so
//!   the path turns clockwise (to the right) by `α` at the joint.
//! * The arc has radius `R` and center `C = (R sin α, R cos α)`. With this
//!   center the arc curves counter-clockwise (to the left of its travel), so
//!   the junction is *inflected*: a right kink followed by a left-curving arc.
//!   The continuing-turn fillet has the opposite center and is not modelled
//!   here.
//! * `d` is a signed offset to the right of travel for both segments:
//!   the line becomes `y = -d` and the circle radius becomes `R + d`.
//!
//! For finite path segments, `d > 0` lies on the incoming ray and on the arc
//! span; `d < 0` lies on both supporting extensions. See [`LineArc::seam_location`].
//!
//! Deviations from the first prototype: the discriminant uses a stable
//! half-angle form, non-finite/overflowed results are rejected, and seam-domain
//! helpers were added. The upstream B-rep and Sweep code is untouched.

/// A line–arc corner with a circular arc of radius `radius` and kink angle
/// `corner_angle_rad`.
#[derive(Debug, Clone, Copy)]
pub struct LineArc {
    /// Arc radius `R`. Positive and finite.
    pub radius: f64,
    /// Kink angle `α` in radians. Must be in `(0, π)`.
    pub corner_angle_rad: f64,
}

/// A point in the path plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
}

impl Point2 {
    /// Euclidean distance between two points.
    pub fn distance_to(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Whether both coordinates are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

/// Why a requested offset intersection or seam point is unavailable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeometryError {
    /// An input or intermediate value is non-finite, invalid, or overflowed.
    InvalidParameters,
    /// The offset circle has non-positive radius (`d <= -R`).
    CollapsedOffsetCircle,
    /// The corresponding offset line and circle do not intersect.
    NoRealIntersection,
    /// The algebra completed but produced a non-finite point.
    NonFiniteResult,
}

/// Where a seam point lies relative to the finite incoming ray and arc span.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamLocation {
    /// The corresponding offset-intersection point.
    pub point: Point2,
    /// Line parameter `x` (joint at `0`, incoming ray at `x <= 0`).
    pub line_parameter: f64,
    /// Arc parameter in radians from the arc start (`0..=α` inside the span;
    /// negative on the extension before the joint).
    pub arc_parameter: f64,
    /// Whether the point is on the finite incoming ray (`x <= 0`).
    pub on_incoming_ray: bool,
    /// Whether the point is inside the finite arc span (`0 <= param <= α`).
    pub on_arc_span: bool,
}

impl LineArc {
    /// Build a line–arc model, rejecting non-finite values and invalid ranges.
    pub fn new(radius: f64, corner_angle_rad: f64) -> Result<Self, GeometryError> {
        if !radius.is_finite()
            || radius <= 0.0
            || !corner_angle_rad.is_finite()
            || corner_angle_rad <= 0.0
            || corner_angle_rad >= std::f64::consts::PI
        {
            return Err(GeometryError::InvalidParameters);
        }
        Ok(Self {
            radius,
            corner_angle_rad,
        })
    }

    /// Center `C = (R sin α, R cos α)` of the supporting circle.
    pub fn center(&self) -> Point2 {
        Point2 {
            x: self.radius * self.corner_angle_rad.sin(),
            y: self.radius * self.corner_angle_rad.cos(),
        }
    }

    /// Vertex `(R sin α, R (1 + cos α) / 2)` of the miter parabola.
    pub fn vertex(&self) -> Point2 {
        let half = 0.5 * self.corner_angle_rad;
        let cos_half = half.cos();
        Point2 {
            x: self.radius * self.corner_angle_rad.sin(),
            y: self.radius * cos_half * cos_half,
        }
    }

    /// Smallest offset with a real corresponding-offset intersection.
    ///
    /// `d_min = -R (1 + cos α) / 2`; it corresponds to the parabola vertex.
    pub fn min_offset(&self) -> f64 {
        let half = 0.5 * self.corner_angle_rad;
        let cos_half = half.cos();
        -self.radius * cos_half * cos_half
    }

    /// Stable discriminant of the offset line/circle intersection.
    ///
    /// `D = R² sin²α + 2 R d (1 - cos α)`
    ///   `= 4 R sin²(α/2) (R cos²(α/2) + d)`.
    fn discriminant(&self, d: f64) -> Result<f64, GeometryError> {
        let half = 0.5 * self.corner_angle_rad;
        let sin_half = half.sin();
        let cos_half = half.cos();
        let discriminant =
            4.0 * self.radius * sin_half * sin_half * (self.radius * cos_half * cos_half + d);
        if !discriminant.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        Ok(discriminant)
    }

    /// Intersection of the supporting offset line and supporting offset circle.
    ///
    /// The selected branch is the one passing through the joint at `d = 0`.
    /// This does not require the point to lie on the finite line or arc; use
    /// [`Self::seam_location`] for that classification.
    pub fn intersect_offsets(&self, d: f64) -> Result<Point2, GeometryError> {
        if !d.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        let effective_radius = self.radius + d;
        if !effective_radius.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        if effective_radius <= 0.0 {
            return Err(GeometryError::CollapsedOffsetCircle);
        }
        let discriminant = self.discriminant(d)?;
        if discriminant < 0.0 {
            return Err(GeometryError::NoRealIntersection);
        }
        let center = self.center();
        let point = Point2 {
            x: center.x - discriminant.sqrt(),
            y: -d,
        };
        if !point.is_finite() {
            return Err(GeometryError::NonFiniteResult);
        }
        Ok(point)
    }

    /// Intersection plus its position on the finite incoming ray and arc span.
    pub fn seam_location(&self, d: f64) -> Result<SeamLocation, GeometryError> {
        let point = self.intersect_offsets(d)?;
        let line_parameter = point.x;
        let arc_parameter = self.arc_parameter(point);
        let scale = self.radius.max(1.0).max(d.abs());
        let tolerance = 1e-12 * scale;
        Ok(SeamLocation {
            point,
            line_parameter,
            arc_parameter,
            on_incoming_ray: line_parameter <= tolerance,
            on_arc_span: arc_parameter >= -tolerance
                && arc_parameter <= self.corner_angle_rad + tolerance,
        })
    }

    /// Standard angle of the arc start on the supporting circle.
    ///
    /// `3π/2 - α`; the arc ends at `3π/2`.
    pub fn arc_start_angle(&self) -> f64 {
        1.5 * std::f64::consts::PI - self.corner_angle_rad
    }

    /// Standard angle of the arc end on the supporting circle (`3π/2`).
    pub fn arc_end_angle(&self) -> f64 {
        1.5 * std::f64::consts::PI
    }

    /// Signed angle of `p` from the arc start, in `(-π, π]`.
    ///
    /// Values in `0..=α` are on the finite arc span; negative values are on the
    /// supporting-circle extension before the joint.
    pub fn arc_parameter(&self, p: Point2) -> f64 {
        let center = self.center();
        let mut phi = (p.y - center.y).atan2(p.x - center.x);
        if phi < 0.0 {
            phi += std::f64::consts::TAU;
        }
        let mut parameter = phi - self.arc_start_angle();
        if parameter > std::f64::consts::PI {
            parameter -= std::f64::consts::TAU;
        } else if parameter < -std::f64::consts::PI {
            parameter += std::f64::consts::TAU;
        }
        parameter
    }

    /// Implicit equation `F(x, y) = 0` of the local miter parabola.
    ///
    /// `x² - 2 R sin α x + 2 R (1 - cos α) y = 0`.
    pub fn implicit_miter(&self, p: Point2) -> f64 {
        let half = 0.5 * self.corner_angle_rad;
        let sin_half = half.sin();
        let cos_half = half.cos();
        let sin_alpha = 2.0 * sin_half * cos_half;
        let one_minus_cos_alpha = 2.0 * sin_half * sin_half;
        p.x * p.x - 2.0 * self.radius * sin_alpha * p.x
            + 2.0 * self.radius * one_minus_cos_alpha * p.y
    }

    /// `dy/dx` of the miter at the junction: the *external* angle bisector.
    ///
    /// This is `cot(α/2)`; the other (internal) bisector is
    /// [`Self::internal_bisector_slope_at_joint`].
    pub fn tangent_slope_at_joint(&self) -> f64 {
        let half = 0.5 * self.corner_angle_rad;
        half.cos() / half.sin()
    }

    /// `dy/dx` of the internal angle bisector of the two tangent directions.
    ///
    /// This is `-tan(α/2)` and is *not* the tangent of this miter branch.
    pub fn internal_bisector_slope_at_joint(&self) -> f64 {
        let half = 0.5 * self.corner_angle_rad;
        -half.sin() / half.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// Independent solver: parameterize the offset circle by angle and bisect
    /// `C_y + (R + d) sin φ + d = 0`. It uses no discriminant, square root, or
    /// parabola equation.
    fn bisection_intersection(model: &LineArc, d: f64) -> Option<Point2> {
        if !d.is_finite() {
            return None;
        }
        let effective_radius = model.radius + d;
        if !effective_radius.is_finite() || effective_radius <= 0.0 {
            return None;
        }
        let center = model.center();
        let residual = |phi: f64| center.y + effective_radius * phi.sin() + d;
        let low = FRAC_PI_2;
        let high = 1.5 * PI;
        if residual(low) < 0.0 || residual(high) > 0.0 {
            return None;
        }
        let mut a = low;
        let mut b = high;
        for _ in 0..200 {
            let mid = 0.5 * (a + b);
            if residual(mid) > 0.0 {
                a = mid;
            } else {
                b = mid;
            }
        }
        let phi = 0.5 * (a + b);
        Some(Point2 {
            x: center.x + effective_radius * phi.cos(),
            y: center.y + effective_radius * phi.sin(),
        })
    }

    #[test]
    fn zero_offset_starts_at_joint() {
        for degrees in [20.0_f64, 45.0, 75.0, 110.0, 150.0] {
            let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
            let p = model.intersect_offsets(0.0).unwrap();
            assert!(p.x.abs() < 1e-12, "{degrees}: {p:?}");
            assert!(p.y.abs() < 1e-12, "{degrees}: {p:?}");
        }
    }

    #[test]
    fn offset_intersections_lie_on_parabola_and_both_loci() {
        for degrees in [30.0_f64, 45.0, 75.0, 110.0, 150.0] {
            for radius in [0.5_f64, 4.0, 100.0] {
                let model = LineArc::new(radius, degrees.to_radians()).unwrap();
                let center = model.center();
                for index in -60..=80 {
                    let d = radius * index as f64 / 200.0;
                    if let Ok(p) = model.intersect_offsets(d) {
                        let scale = radius * radius;
                        assert!(
                            model.implicit_miter(p).abs() <= 1e-9 * scale,
                            "miter: angle={degrees} r={radius} d={d} p={p:?}"
                        );
                        assert!((p.y + d).abs() <= 1e-12 * radius);
                        let distance_squared = (p.x - center.x).powi(2) + (p.y - center.y).powi(2);
                        assert!((distance_squared - (radius + d).powi(2)).abs() <= 1e-9 * scale);
                    }
                }
            }
        }
    }

    #[test]
    fn intersections_are_equidistant_from_center_and_directrix() {
        for degrees in [30.0_f64, 90.0, 150.0] {
            for radius in [0.5_f64, 4.0, 100.0] {
                let model = LineArc::new(radius, degrees.to_radians()).unwrap();
                let center = model.center();
                let minimum = model.min_offset();
                for step in 0..=100 {
                    let d = minimum + (radius - minimum) * step as f64 / 100.0;
                    let p = model.intersect_offsets(d).unwrap();
                    let scale = radius.max(1.0);
                    // Parabola focus/directrix: |P - C| = R - y (directrix y=R).
                    let focus_distance = p.distance_to(center);
                    let directrix_distance = radius - p.y;
                    assert!(
                        (focus_distance - directrix_distance).abs() <= 1e-9 * scale,
                        "focus/directrix: angle={degrees} r={radius} d={d} p={p:?}"
                    );
                    // Both supporting loci, checked independently.
                    assert!((p.y + d).abs() <= 1e-10 * scale);
                    let circle_distance = p.distance_to(center);
                    assert!((circle_distance - (radius + d)).abs() <= 1e-9 * scale);
                }
            }
        }
    }

    #[test]
    fn independent_bisection_matches_closed_form() {
        for degrees in [1.0_f64, 30.0, 89.0, 90.0, 110.0, 150.0, 179.0] {
            let radius = 4.0;
            let model = LineArc::new(radius, degrees.to_radians()).unwrap();
            let minimum = model.min_offset();
            let ratios = [0.0_f64, 1e-6, -1e-6, 0.25, -0.25, 1.0, -1.0, 10.0, -10.0];
            for ratio in ratios {
                let d = ratio * radius;
                if d < minimum {
                    continue;
                }
                let closed = model.intersect_offsets(d).unwrap();
                let independent = bisection_intersection(&model, d)
                    .unwrap_or_else(|| panic!("no bisection root: angle={degrees} d={d}"));
                let scale = closed.x.abs().max(closed.y.abs()).max(radius);
                assert!(
                    (closed.x - independent.x).abs() <= 1e-9 * scale,
                    "x: angle={degrees} d={d} closed={closed:?} independent={independent:?}"
                );
                assert!(
                    (closed.y - independent.y).abs() <= 1e-9 * scale,
                    "y: angle={degrees} d={d} closed={closed:?} independent={independent:?}"
                );
            }
        }
    }

    #[test]
    fn finite_domain_membership_splits_at_zero_offset() {
        for degrees in [30.0_f64, 90.0, 150.0] {
            let radius = 4.0;
            let model = LineArc::new(radius, degrees.to_radians()).unwrap();
            let minimum = model.min_offset();

            let joint = model.seam_location(0.0).unwrap();
            assert!(joint.on_incoming_ray, "angle={degrees}");
            assert!(joint.on_arc_span, "angle={degrees}");

            for ratio in [0.01_f64, 0.1, 0.5, 1.0, 10.0] {
                let location = model.seam_location(ratio * radius).unwrap();
                assert!(location.on_incoming_ray, "angle={degrees} d={ratio}R");
                assert!(location.on_arc_span, "angle={degrees} d={ratio}R");
                assert!(location.line_parameter < 0.0);
                assert!(location.arc_parameter > 0.0);
            }

            let d = 0.5 * minimum;
            assert!(d > minimum && d < 0.0, "angle={degrees}");
            let location = model.seam_location(d).unwrap();
            assert!(!location.on_incoming_ray, "angle={degrees} d={d}");
            assert!(!location.on_arc_span, "angle={degrees} d={d}");
            assert!(location.line_parameter > 0.0);
            assert!(location.arc_parameter < 0.0);
        }
    }

    #[test]
    fn vertex_is_the_existence_limit() {
        for degrees in [30.0_f64, 90.0, 150.0] {
            let radius = 4.0;
            let model = LineArc::new(radius, degrees.to_radians()).unwrap();
            let minimum = model.min_offset();
            let vertex = model.vertex();
            let at_limit = model.intersect_offsets(minimum).unwrap();
            assert!((at_limit.x - vertex.x).abs() <= 1e-12 * radius);
            assert!((at_limit.y - vertex.y).abs() <= 1e-12 * radius);
            assert_eq!(
                model.intersect_offsets(minimum - 1e-9 * radius),
                Err(GeometryError::NoRealIntersection)
            );
        }
    }

    #[test]
    fn near_tangency_is_stable() {
        let radius = 4.0;
        let model = LineArc::new(radius, FRAC_PI_2).unwrap();
        let minimum = model.min_offset();
        let vertex = model.vertex();
        for epsilon in [1e-3_f64, 1e-6, 1e-9, 1e-12] {
            let d = minimum + epsilon * radius;
            let p = model.intersect_offsets(d).unwrap();
            assert!(p.is_finite());
            assert!(model.implicit_miter(p).abs() <= 1e-8 * radius * radius);
            let independent = bisection_intersection(&model, d).unwrap();
            assert!((p.x - independent.x).abs() <= 1e-6 * radius);
            assert!((p.y - independent.y).abs() <= 1e-12 * radius);
            if epsilon <= 1e-9 {
                assert!((p.x - vertex.x).abs() <= 1e-4 * radius);
            }
        }
    }

    #[test]
    fn offset_solution_scales_with_radius() {
        let corner = FRAC_PI_2;
        let base_radius = 4.0;
        let base = LineArc::new(base_radius, corner).unwrap();
        for scale in [1e-6_f64, 1e-3, 1.0, 1e3, 1e6] {
            let scaled = LineArc::new(scale * base_radius, corner).unwrap();
            for ratio in [0.001_f64, 0.1, -0.1, 1.0, 10.0] {
                let d = ratio * base_radius;
                let p = base.intersect_offsets(d).unwrap();
                let q = scaled.intersect_offsets(scale * d).unwrap();
                let expected_x = scale * p.x;
                let expected_y = scale * p.y;
                let tolerance = 1e-9 * expected_x.abs().max(expected_y.abs()).max(scale);
                assert!(
                    (q.x - expected_x).abs() <= tolerance,
                    "x: scale={scale} d={d} q={q:?} expected={expected_x}"
                );
                assert!(
                    (q.y - expected_y).abs() <= tolerance,
                    "y: scale={scale} d={d} q={q:?} expected={expected_y}"
                );
            }
        }
    }

    #[test]
    fn sharp_corner_leaves_outer_profile_uncovered() {
        let radius = 4.0;
        let model = LineArc::new(radius, 150.0_f64.to_radians()).unwrap();
        let minimum = model.min_offset();
        assert!(minimum < 0.0);
        assert!(minimum > -0.1 * radius);

        // A symmetric profile spans d in [-w, w]; the outer part below d_min
        // has no corresponding-offset intersection at all.
        let low = -0.5 * radius;
        let high = 0.5 * radius;
        let samples = 2001;
        let mut covered = 0;
        for index in 0..samples {
            let d = low + (high - low) * index as f64 / (samples - 1) as f64;
            if model.intersect_offsets(d).is_ok() {
                covered += 1;
            }
        }
        let uncovered = samples - covered;
        assert!(
            uncovered > samples * 4 / 10,
            "expected a large uncovered outer band, covered={covered} of {samples}"
        );
        assert_eq!(
            model.intersect_offsets(-1.0),
            Err(GeometryError::NoRealIntersection)
        );
    }

    #[test]
    fn tangent_is_the_external_bisector() {
        for degrees in [20.0_f64, 45.0, 90.0, 150.0] {
            let angle = degrees.to_radians();
            let model = LineArc::new(4.0, angle).unwrap();
            let incoming = (1.0_f64, 0.0_f64);
            let outgoing = (angle.cos(), -angle.sin());
            let external = (incoming.0 - outgoing.0, incoming.1 - outgoing.1);
            let internal = (incoming.0 + outgoing.0, incoming.1 + outgoing.1);
            let external_slope = external.1 / external.0;
            let internal_slope = internal.1 / internal.0;
            assert!(
                (model.tangent_slope_at_joint() - external_slope).abs() < 1e-12,
                "angle={degrees}"
            );
            assert!(
                (model.internal_bisector_slope_at_joint() - internal_slope).abs() < 1e-12,
                "angle={degrees}"
            );
            assert!((external_slope - internal_slope).abs() > 1e-6);
        }
    }

    #[test]
    fn model_is_an_inflected_junction() {
        for degrees in [30.0_f64, 90.0, 150.0] {
            let angle = degrees.to_radians();
            let model = LineArc::new(4.0, angle).unwrap();
            let center = model.center();
            let outgoing = (angle.cos(), -angle.sin());
            // cross(outgoing, center) > 0 => center is left of arc travel,
            // so the arc curves left while the joint itself turns right.
            let center_side = outgoing.0 * center.y - outgoing.1 * center.x;
            assert!(center_side > 0.0);
            assert!((center_side - model.radius).abs() <= 1e-12 * model.radius);
            // cross((1,0), outgoing) = -sin(alpha) < 0 => clockwise kink.
            assert!(outgoing.1 < 0.0);
        }
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        assert!(LineArc::new(0.0, 0.5).is_err());
        assert!(LineArc::new(-1.0, 0.5).is_err());
        assert!(LineArc::new(f64::NAN, 0.5).is_err());
        assert!(LineArc::new(f64::INFINITY, 0.5).is_err());
        assert!(LineArc::new(4.0, 0.0).is_err());
        assert!(LineArc::new(4.0, PI).is_err());
        assert!(LineArc::new(4.0, f64::NAN).is_err());

        let model = LineArc::new(4.0, std::f64::consts::FRAC_PI_4).unwrap();
        assert_eq!(
            model.intersect_offsets(-4.0),
            Err(GeometryError::CollapsedOffsetCircle)
        );
        assert_eq!(
            model.intersect_offsets(-5.0),
            Err(GeometryError::CollapsedOffsetCircle)
        );
        assert_eq!(
            model.intersect_offsets(f64::NAN),
            Err(GeometryError::InvalidParameters)
        );
        assert_eq!(
            model.intersect_offsets(f64::INFINITY),
            Err(GeometryError::InvalidParameters)
        );

        // R + d overflows to infinity: reject instead of returning NaNs.
        let huge = LineArc::new(f64::MAX, 0.5).unwrap();
        assert_eq!(
            huge.intersect_offsets(f64::MAX),
            Err(GeometryError::InvalidParameters)
        );
    }
}
