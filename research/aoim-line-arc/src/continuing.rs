//! Continuing-turn (right-curving) Line–Arc mirror.
//!
//! The inflected model in [`crate::LineArc`] has a right kink followed by a
//! left-curving arc. This module is the opposite-curvature control: the joint
//! still turns right, but the arc continues that turn (its center is on the
//! right of the outgoing tangent).
//!
//! Offsets are still signed to the right of travel. Consequently the offset
//! circle radius is `R - d`, and the AOIM existence interval is the opposite
//! side: `d <= d_max = R cos²(α/2)`. The missing side is the concave/inside
//! side of the turn, where the junction tangent continuation points forward
//! into the arc rather than backward into the corner.

use crate::{GeometryError, Point2};

/// Right-curving (continuing-turn) Line–Arc corner.
#[derive(Debug, Clone, Copy)]
pub struct ContinuingLineArc {
    /// Arc radius `R`.
    pub radius: f64,
    /// Kink angle `α` in `(0, π)`.
    pub corner_angle_rad: f64,
}

impl ContinuingLineArc {
    /// Build the mirror model, rejecting invalid ranges.
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

    /// Center `C = (-R sin α, -R cos α)`: right of the outgoing tangent.
    pub fn center(&self) -> Point2 {
        Point2 {
            x: -self.radius * self.corner_angle_rad.sin(),
            y: -self.radius * self.corner_angle_rad.cos(),
        }
    }

    /// Profile point on the arc at the joint for offset `d`.
    pub fn joint_arc_point(&self, d: f64) -> Result<Point2, GeometryError> {
        if !d.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        if self.radius - d <= 0.0 {
            return Err(GeometryError::CollapsedOffsetCircle);
        }
        Ok(Point2 {
            x: -d * self.corner_angle_rad.sin(),
            y: -d * self.corner_angle_rad.cos(),
        })
    }

    /// Unit tangent at the joint, `(cos α, -sin α)`.
    pub fn joint_tangent(&self) -> Point2 {
        Point2 {
            x: self.corner_angle_rad.cos(),
            y: -self.corner_angle_rad.sin(),
        }
    }

    /// Existence limit `d_max = R cos²(α/2)`.
    pub fn max_offset(&self) -> f64 {
        let half = 0.5 * self.corner_angle_rad;
        let cos_half = half.cos();
        self.radius * cos_half * cos_half
    }

    /// Vertex of the continuing-turn miter parabola.
    pub fn vertex(&self) -> Point2 {
        let half = 0.5 * self.corner_angle_rad;
        let cos_half = half.cos();
        Point2 {
            x: -self.radius * self.corner_angle_rad.sin(),
            y: -self.radius * cos_half * cos_half,
        }
    }

    /// AOIM intersection on the continuing-turn branch.
    ///
    /// Exists for `d <= d_max`; the branch passes through the joint at `d = 0`.
    pub fn intersect_offsets(&self, d: f64) -> Result<Point2, GeometryError> {
        if !d.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        let radius = self.radius - d;
        if !radius.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        if radius <= 0.0 {
            return Err(GeometryError::CollapsedOffsetCircle);
        }
        let half = 0.5 * self.corner_angle_rad;
        let sin_half = half.sin();
        let cos_half = half.cos();
        let discriminant =
            4.0 * self.radius * sin_half * sin_half * (self.radius * cos_half * cos_half - d);
        if discriminant < 0.0 {
            return Err(GeometryError::NoRealIntersection);
        }
        let center = self.center();
        Ok(Point2 {
            x: center.x + discriminant.sqrt(),
            y: -d,
        })
    }

    /// Intersection of the line offset with the junction tangent continuation.
    ///
    /// This is the same external bisector as in the inflected model:
    /// `Q(d) = (-d tan(α/2), -d)`. The signed tangent parameter is
    /// `t = d tan(α/2)`: positive here means the continuation runs *forward*
    /// along the arc, not backward into the corner.
    pub fn junction_linear_point(&self, d: f64) -> Result<Point2, GeometryError> {
        if !d.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        if self.radius - d <= 0.0 {
            return Err(GeometryError::CollapsedOffsetCircle);
        }
        let half = 0.5 * self.corner_angle_rad;
        let tangent = half.sin() / half.cos();
        Ok(Point2 {
            x: -d * tangent,
            y: -d,
        })
    }

    /// Signed parameter of the junction tangent intersection.
    pub fn tangent_parameter(&self, d: f64) -> f64 {
        let half = 0.5 * self.corner_angle_rad;
        d * (half.sin() / half.cos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> ContinuingLineArc {
        ContinuingLineArc::new(4.0, 150.0_f64.to_radians()).unwrap()
    }

    #[test]
    fn existence_is_on_the_positive_side() {
        let model = model();
        let maximum = model.max_offset();
        assert!((maximum - 0.267_949_2).abs() < 1e-6);
        assert!(model.intersect_offsets(0.0).is_ok());
        assert!(model.intersect_offsets(-2.0).is_ok());
        assert!(model.intersect_offsets(maximum).is_ok());
        assert_eq!(
            model.intersect_offsets(maximum + 1e-6),
            Err(GeometryError::NoRealIntersection)
        );
        assert_eq!(
            model.intersect_offsets(2.0),
            Err(GeometryError::NoRealIntersection)
        );
    }

    #[test]
    fn missing_side_is_the_forward_tangent_side() {
        let model = model();
        let maximum = model.max_offset();
        for d in [maximum + 0.01, 0.5, 1.0, 2.0] {
            let parameter = model.tangent_parameter(d);
            assert!(parameter > 0.0, "d={d} parameter={parameter}");
        }
        // In the inflected model the missing side has t < 0.
        for d in [-2.0, -1.0, model.max_offset() - 2.0] {
            assert!(model.tangent_parameter(d) < 0.0);
        }
    }

    #[test]
    fn backward_tangent_ray_moves_away_from_the_line_offsets() {
        let model = model();
        let tangent = model.joint_tangent();
        for d in [0.5_f64, 1.0, 2.0] {
            let joint = model.joint_arc_point(d).unwrap();
            let line_y = -d;
            for step in 1..=20 {
                let t = -step as f64 * 0.1;
                let y = joint.y + t * tangent.y;
                assert!(y > line_y, "d={d} t={t} y={y} line={line_y}");
            }
        }
    }

    #[test]
    fn junction_line_intersection_is_unique() {
        let model = model();
        let tangent = model.joint_tangent();
        for d in [0.5_f64, 1.0, 2.0] {
            let joint = model.joint_arc_point(d).unwrap();
            let seam = model.junction_linear_point(d).unwrap();
            let parameter = model.tangent_parameter(d);
            let expected = Point2 {
                x: joint.x + parameter * tangent.x,
                y: joint.y + parameter * tangent.y,
            };
            assert!(seam.distance_to(expected) < 1e-12);
            assert!((seam.y + d).abs() < 1e-12);
        }
    }

    #[test]
    fn forward_extension_is_inside_the_line_body() {
        let model = model();
        let d = 1.0;
        let joint = model.joint_arc_point(d).unwrap();
        let seam = model.junction_linear_point(d).unwrap();
        // Symmetric profile d in [-2, 2]: line strip x <= 0, |y| <= 2.
        assert!(joint.x < 0.0 && seam.x < 0.0);
        assert!(joint.y.abs() <= 2.0 && seam.y.abs() <= 2.0);
        let midpoint = Point2 {
            x: 0.5 * (joint.x + seam.x),
            y: 0.5 * (joint.y + seam.y),
        };
        assert!(midpoint.x < 0.0 && midpoint.y.abs() <= 2.0);
    }

    #[test]
    fn inflected_missing_side_extension_is_outside_the_line_body() {
        let inflected = crate::LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let d = -2.0;
        let joint = inflected.joint_arc_point(d).unwrap();
        let seam = inflected.junction_linear_point(d).unwrap();
        // In the inflected case the tangent extension is on the forward line
        // extension, outside the finite line strip x <= 0.
        assert!(joint.x > 0.0 && seam.x > 0.0);
    }
}
