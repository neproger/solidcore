//! Independent AOIM Line–Arc analytical geometry experiment.
//! This is NOT connected to solidcore's B-rep builder.
//! Coordinates: incoming line y=0, arc circle center (R sin(a), R cos(a)).
//! Corresponding signed offsets: line y=-d; circle radius R+d.

#[derive(Debug, Clone, Copy)]
pub struct LineArc {
    pub radius: f64,
    pub corner_angle_rad: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeometryError {
    InvalidParameters,
    CollapsedOffsetCircle,
    NoRealIntersection,
}

impl LineArc {
    pub fn new(radius: f64, corner_angle_rad: f64) -> Result<Self, GeometryError> {
        if !radius.is_finite() || radius <= 0.0
            || !corner_angle_rad.is_finite()
            || corner_angle_rad <= 0.0
            || corner_angle_rad >= std::f64::consts::PI
        {
            return Err(GeometryError::InvalidParameters);
        }
        Ok(Self { radius, corner_angle_rad })
    }

    pub fn center(&self) -> Point2 {
        Point2 {
            x: self.radius * self.corner_angle_rad.sin(),
            y: self.radius * self.corner_angle_rad.cos(),
        }
    }

    /// Local intersection branch passing through (0,0) at d=0.
    /// This uses the *supporting* offset circle, not necessarily a finite arc.
    pub fn intersect_offsets(&self, d: f64) -> Result<Point2, GeometryError> {
        if !d.is_finite() {
            return Err(GeometryError::InvalidParameters);
        }
        let effective_radius = self.radius + d;
        if effective_radius <= 0.0 {
            return Err(GeometryError::CollapsedOffsetCircle);
        }
        let center = self.center();
        let y = -d;
        let discriminant = effective_radius * effective_radius - (y - center.y).powi(2);
        if discriminant < 0.0 {
            return Err(GeometryError::NoRealIntersection);
        }
        Ok(Point2 { x: center.x - discriminant.sqrt(), y })
    }

    /// Implicit equation F(x,y)=0 for the local parabola.
    pub fn implicit_miter(&self, p: Point2) -> f64 {
        let a = self.corner_angle_rad;
        p.x * p.x - 2.0 * self.radius * a.sin() * p.x
            + 2.0 * self.radius * (1.0 - a.cos()) * p.y
    }

    /// dy/dx of the miter at the original path junction.
    pub fn tangent_slope_at_joint(&self) -> f64 {
        self.corner_angle_rad.sin() / (1.0 - self.corner_angle_rad.cos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
                let c = model.center();
                // Scaling offset with radius keeps this test far from degenerate radii.
                for index in -60..=80 {
                    let d = radius * index as f64 / 200.0;
                    if let Ok(p) = model.intersect_offsets(d) {
                        let scale = radius * radius;
                        assert!(model.implicit_miter(p).abs() <= 1e-9 * scale,
                            "miter: angle={degrees} r={radius} d={d} p={p:?}");
                        assert!((p.y + d).abs() <= 1e-12 * radius);
                        let distance_squared = (p.x - c.x).powi(2) + (p.y - c.y).powi(2);
                        assert!((distance_squared - (radius + d).powi(2)).abs()
                            <= 1e-9 * scale);
                    }
                }
            }
        }
    }

    #[test]
    fn parabola_tangent_is_angle_bisector() {
        for degrees in [20.0_f64, 45.0, 90.0, 150.0] {
            let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
            let expected = (degrees.to_radians() / 2.0).tan().recip();
            assert!((model.tangent_slope_at_joint() - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        assert!(LineArc::new(0.0, 0.5).is_err());
        assert!(LineArc::new(4.0, 0.0).is_err());
        assert!(LineArc::new(4.0, std::f64::consts::PI).is_err());
        let model = LineArc::new(4.0, std::f64::consts::FRAC_PI_4).unwrap();
        assert_eq!(model.intersect_offsets(-4.0), Err(GeometryError::CollapsedOffsetCircle));
    }
}
