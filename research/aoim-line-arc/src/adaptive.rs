//! Adaptive path regularization for the AOIM Line–Arc experiment.
//!
//! The strategy shortens the incoming line to a point `A`, inserts a straight
//! segment `A -> Q` tangent to the original arc at `Q`, and keeps the remaining
//! arc. The original corner at `P` is replaced by:
//!
//! * a line–line corner at `A`, handled by the usual straight miter;
//! * a tangent line–arc junction at `Q`, where no miter is needed.
//!
//! The cut is chosen so that the new miter length and the path deviation both
//! stay below caller-supplied bounds. The original analytical and hybrid
//! strategies are not modified.

use crate::sweep::{raster_compare, Polygon, RasterMetrics, RegionError, SeamSample};
use crate::{GeometryError, LineArc, Point2};

/// Bounds for the adaptive path cut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptiveParams {
    /// Maximum allowed straight miter seam length across the profile.
    pub max_miter_length: f64,
    /// Maximum allowed Hausdorff deviation from the original path.
    pub max_path_deviation: f64,
    /// Require the far endpoints (line start and arc end) to remain.
    ///
    /// The straight-transition family only shortens both segments, so it
    /// satisfies this automatically; `false` does not enable moving the far
    /// endpoints (that would require a different construction).
    pub preserve_endpoints: bool,
    /// Require G1 at every transition of the modified path.
    pub preserve_tangency: bool,
}

/// Why an adaptive cut could not be produced.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AdaptiveError {
    /// A primitive failed.
    Geometry(GeometryError),
    /// A region construction failed.
    Region(RegionError),
    /// Parameters are non-finite or outside their valid range.
    InvalidParameters,
    /// A straight transition cannot satisfy `preserve_tangency`.
    TangencyRequired,
    /// The original line–line miter is already inside the requested bound.
    NoCutNeeded {
        /// Miter chord of the unmodified line–line corner.
        original_miter: f64,
    },
    /// No cut satisfies both bounds.
    NoFeasibleCut {
        /// Smallest miter length seen in the scan.
        min_miter: f64,
        /// Smallest path deviation seen in the scan.
        min_deviation: f64,
    },
}

impl From<GeometryError> for AdaptiveError {
    fn from(error: GeometryError) -> Self {
        Self::Geometry(error)
    }
}

impl From<RegionError> for AdaptiveError {
    fn from(error: RegionError) -> Self {
        Self::Region(error)
    }
}

/// Geometric description of the adaptive cut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptiveCut {
    /// Turn angle at the new line–line corner `A` (negative for a right turn).
    pub theta: f64,
    /// New line end / line–line corner `A`.
    pub cut_point: Point2,
    /// Tangency point `Q` on the original arc.
    pub tangent_point: Point2,
    /// Length of the tangent segment `A -> Q`.
    pub tangent_length: f64,
    /// Part of the original arc replaced by the straight segment.
    pub removed_arc_angle: f64,
    /// Original arc angle that remains after `Q`.
    pub remaining_arc_angle: f64,
    /// Length of the straight line–line miter across the profile.
    pub miter_length: f64,
    /// Maximum distance from `A` to a miter point.
    pub miter_extent: f64,
    /// Hausdorff deviation between the original and modified local paths.
    pub path_deviation: f64,
}

/// Solved adaptive path.
#[derive(Debug, Clone, Copy)]
pub struct AdaptivePath {
    /// Original model.
    pub model: LineArc,
    /// Selected cut.
    pub cut: AdaptiveCut,
    /// Incoming line length (joint at `x = 0`, start at `x = -line_length`).
    pub line_length: f64,
    /// Profile lower offset.
    pub d0: f64,
    /// Profile upper offset.
    pub d1: f64,
}

/// Trimmed regions for the modified path.
#[derive(Debug, Clone)]
pub struct AdaptiveRegions {
    /// Trimmed incoming line strip.
    pub line: Polygon,
    /// Straight transition strip.
    pub straight: Polygon,
    /// Remaining arc sector.
    pub arc: Polygon,
    /// Expected outer contour.
    pub expected: Polygon,
    /// Sampled line–line miter seam.
    pub seam: Vec<SeamSample>,
    /// Selected cut and its metrics.
    pub cut: AdaptiveCut,
    /// `line + straight + arc - expected`.
    pub area_balance: f64,
    /// `area_balance / expected.area()`.
    pub area_balance_fraction: f64,
    /// Whether any part or the expected contour self-intersects.
    pub self_intersecting: bool,
    /// Raster comparison of the three parts against the expected contour.
    pub raster: RasterMetrics,
}

impl AdaptivePath {
    /// Search for the least-deviating cut that satisfies both bounds.
    ///
    /// `preserve_tangency = true` is reported as infeasible for this
    /// straight-transition construction: the line–straight corner at `A` is a
    /// genuine G0 corner.
    pub fn solve(
        model: LineArc,
        d0: f64,
        d1: f64,
        line_length: f64,
        params: AdaptiveParams,
    ) -> Result<Self, AdaptiveError> {
        if !d0.is_finite()
            || !d1.is_finite()
            || !line_length.is_finite()
            || d0 >= d1
            || line_length <= 0.0
            || model.radius + d0 <= 0.0
            || !params.max_miter_length.is_finite()
            || !params.max_path_deviation.is_finite()
            || params.max_miter_length < 0.0
            || params.max_path_deviation < 0.0
        {
            return Err(AdaptiveError::InvalidParameters);
        }
        if params.preserve_tangency {
            return Err(AdaptiveError::TangencyRequired);
        }

        let alpha = model.corner_angle_rad;
        let untouched = AdaptiveCut {
            theta: -alpha,
            cut_point: Point2 { x: 0.0, y: 0.0 },
            tangent_point: Point2 { x: 0.0, y: 0.0 },
            tangent_length: 0.0,
            removed_arc_angle: 0.0,
            remaining_arc_angle: alpha,
            miter_length: 0.0,
            miter_extent: 0.0,
            path_deviation: 0.0,
        };
        let untouched0 = line_line_seam(untouched, d0)?;
        let untouched1 = line_line_seam(untouched, d1)?;
        let original_miter = untouched0.distance_to(untouched1);
        if original_miter <= params.max_miter_length + 1e-12 {
            return Err(AdaptiveError::NoCutNeeded { original_miter });
        }
        let steps = 1600;
        let low = -alpha + 1e-6;
        let high = -1e-6;
        let mut best: Option<AdaptiveCut> = None;
        let mut min_miter = f64::INFINITY;
        let mut min_deviation = f64::INFINITY;

        for index in 0..=steps {
            let t = index as f64 / steps as f64;
            let theta = low + (high - low) * t;
            let Some(geometry) = cut_geometry(&model, theta) else {
                continue;
            };
            if geometry.cut_point.x < -line_length {
                continue;
            }
            if geometry.removed_arc_angle <= 1e-2 || geometry.remaining_arc_angle <= 1e-6 {
                continue;
            }
            let deviation = path_deviation(
                &model,
                geometry.cut_point,
                geometry.tangent_point,
                geometry.removed_arc_angle,
            );
            let mut cut = AdaptiveCut {
                theta,
                cut_point: geometry.cut_point,
                tangent_point: geometry.tangent_point,
                tangent_length: geometry.tangent_length,
                removed_arc_angle: geometry.removed_arc_angle,
                remaining_arc_angle: geometry.remaining_arc_angle,
                miter_length: 0.0,
                miter_extent: 0.0,
                path_deviation: deviation,
            };
            let p0 = match line_line_seam(cut, d0) {
                Ok(point) => point,
                Err(_) => continue,
            };
            let p1 = match line_line_seam(cut, d1) {
                Ok(point) => point,
                Err(_) => continue,
            };
            let miter_length = p0.distance_to(p1);
            let miter_extent = p0
                .distance_to(geometry.cut_point)
                .max(p1.distance_to(geometry.cut_point));
            min_miter = min_miter.min(miter_length);
            min_deviation = min_deviation.min(deviation);
            if miter_length > params.max_miter_length + 1e-12
                || deviation > params.max_path_deviation + 1e-12
            {
                continue;
            }
            let seam = [
                SeamSample { d: d0, point: p0 },
                SeamSample { d: d1, point: p1 },
            ];
            let straight = match straight_polygon(&seam, cut, d0, d1) {
                Ok(polygon) => polygon,
                Err(_) => continue,
            };
            if straight.is_self_intersecting() {
                continue;
            }
            cut.miter_length = miter_length;
            cut.miter_extent = miter_extent;
            let replace = match best {
                None => true,
                Some(current) => {
                    deviation < current.path_deviation - 1e-12
                        || ((deviation - current.path_deviation).abs() <= 1e-12
                            && cut.theta < current.theta)
                }
            };
            if replace {
                best = Some(cut);
            }
        }

        match best {
            Some(cut) => Ok(Self {
                model,
                cut,
                line_length,
                d0,
                d1,
            }),
            None => Err(AdaptiveError::NoFeasibleCut {
                min_miter,
                min_deviation,
            }),
        }
    }

    /// Line–line miter point for offset `d` on the modified path.
    pub fn seam_point(&self, d: f64) -> Result<Point2, GeometryError> {
        line_line_seam(self.cut, d)
    }

    /// Build the three trimmed parts, the expected contour, and metrics.
    pub fn build_regions(
        &self,
        seam_samples: usize,
        arc_steps: usize,
        raster_resolution: usize,
    ) -> Result<AdaptiveRegions, AdaptiveError> {
        if seam_samples < 2 || arc_steps < 1 {
            return Err(AdaptiveError::InvalidParameters);
        }
        let mut seam = Vec::with_capacity(seam_samples);
        for index in 0..seam_samples {
            let t = index as f64 / (seam_samples - 1) as f64;
            let d = self.d0 + (self.d1 - self.d0) * t;
            seam.push(SeamSample {
                d,
                point: line_line_seam(self.cut, d)?,
            });
        }
        for sample in &seam {
            if sample.point.x < -self.line_length {
                return Err(AdaptiveError::Region(RegionError::LineTooShort));
            }
        }

        let line = line_polygon(&seam, self.d0, self.d1, self.line_length)?;
        let straight = straight_polygon(&seam, self.cut, self.d0, self.d1)?;
        let arc = arc_polygon(&self.model, self.cut, self.d0, self.d1, arc_steps)?;
        let expected = expected_polygon(
            &self.model,
            self.cut,
            &seam,
            self.d0,
            self.d1,
            self.line_length,
            arc_steps,
        )?;

        let line_area = line.area();
        let straight_area = straight.area();
        let arc_area = arc.area();
        let expected_area = expected.area();
        let area_balance = line_area + straight_area + arc_area - expected_area;
        let area_balance_fraction = if expected_area > 0.0 {
            area_balance / expected_area
        } else {
            0.0
        };
        let self_intersecting = line.is_self_intersecting()
            || straight.is_self_intersecting()
            || arc.is_self_intersecting()
            || expected.is_self_intersecting();
        let raster = raster_compare(&[&line, &straight, &arc], &expected, raster_resolution)?;

        Ok(AdaptiveRegions {
            line,
            straight,
            arc,
            expected,
            seam,
            cut: self.cut,
            area_balance,
            area_balance_fraction,
            self_intersecting,
            raster,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct CutGeometry {
    cut_point: Point2,
    tangent_point: Point2,
    tangent_length: f64,
    removed_arc_angle: f64,
    remaining_arc_angle: f64,
}

fn cut_geometry(model: &LineArc, theta: f64) -> Option<CutGeometry> {
    let ct = theta.cos();
    let st = theta.sin();
    if st.abs() < 1e-14 {
        return None;
    }
    let center = model.center();
    let s = (ct * center.y - st * center.x - model.radius) / st;
    if !s.is_finite() || s < 0.0 {
        return None;
    }
    let cut_point = Point2 { x: -s, y: 0.0 };
    let tangent_length = (center.x + s) * ct + center.y * st;
    if !tangent_length.is_finite() || tangent_length < 0.0 {
        return None;
    }
    let tangent_point = Point2 {
        x: cut_point.x + tangent_length * ct,
        y: cut_point.y + tangent_length * st,
    };
    let removed = model.arc_parameter(tangent_point);
    let alpha = model.corner_angle_rad;
    let tolerance = 1e-10;
    if removed < -tolerance || removed > alpha + tolerance {
        return None;
    }
    Some(CutGeometry {
        cut_point,
        tangent_point,
        tangent_length,
        removed_arc_angle: removed.max(0.0).min(alpha),
        remaining_arc_angle: (alpha - removed).max(0.0).min(alpha),
    })
}

fn line_line_seam(cut: AdaptiveCut, d: f64) -> Result<Point2, GeometryError> {
    if !d.is_finite() {
        return Err(GeometryError::InvalidParameters);
    }
    let ct = cut.theta.cos();
    let st = cut.theta.sin();
    let tangent_half = st / (1.0 + ct);
    let along = -d * tangent_half;
    let point = Point2 {
        x: cut.cut_point.x + along * ct + d * st,
        y: -d,
    };
    if !point.is_finite() {
        return Err(GeometryError::NonFiniteResult);
    }
    Ok(point)
}

fn path_deviation(
    model: &LineArc,
    cut_point: Point2,
    tangent_point: Point2,
    removed_arc_angle: f64,
) -> f64 {
    let steps = 64;
    let start = model.arc_start_angle();
    let center = model.center();
    let mut original = Vec::with_capacity(2 * steps + 2);
    for index in 0..=steps {
        let t = index as f64 / steps as f64;
        original.push(Point2 {
            x: cut_point.x * (1.0 - t),
            y: 0.0,
        });
    }
    for index in 1..=steps {
        let t = index as f64 / steps as f64;
        let angle = start + removed_arc_angle * t;
        original.push(Point2 {
            x: center.x + model.radius * angle.cos(),
            y: center.y + model.radius * angle.sin(),
        });
    }

    let mut deviation = 0.0_f64;
    for point in &original {
        deviation = deviation.max(point_segment_distance(*point, cut_point, tangent_point));
    }
    for index in 0..=steps {
        let t = index as f64 / steps as f64;
        let point = Point2 {
            x: cut_point.x + (tangent_point.x - cut_point.x) * t,
            y: cut_point.y + (tangent_point.y - cut_point.y) * t,
        };
        let mut distance = f64::INFINITY;
        for pair in original.windows(2) {
            distance = distance.min(point_segment_distance(point, pair[0], pair[1]));
        }
        deviation = deviation.max(distance);
    }
    deviation
}

fn line_polygon(
    seam: &[SeamSample],
    d0: f64,
    d1: f64,
    line_length: f64,
) -> Result<Polygon, AdaptiveError> {
    let mut points = Vec::with_capacity(seam.len() + 2);
    points.push(Point2 {
        x: -line_length,
        y: -d1,
    });
    for sample in seam.iter().rev() {
        push_unique(&mut points, sample.point);
    }
    push_unique(
        &mut points,
        Point2 {
            x: -line_length,
            y: -d0,
        },
    );
    Polygon::new(points).map_err(AdaptiveError::Region)
}

fn straight_polygon(
    seam: &[SeamSample],
    cut: AdaptiveCut,
    d0: f64,
    d1: f64,
) -> Result<Polygon, AdaptiveError> {
    let normal = Point2 {
        x: cut.theta.sin(),
        y: -cut.theta.cos(),
    };
    let q0 = Point2 {
        x: cut.tangent_point.x + d0 * normal.x,
        y: cut.tangent_point.y + d0 * normal.y,
    };
    let q1 = Point2 {
        x: cut.tangent_point.x + d1 * normal.x,
        y: cut.tangent_point.y + d1 * normal.y,
    };
    let first = seam
        .first()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;
    let last = seam
        .last()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;
    let mut points = vec![first.point, q0, q1, last.point];
    dedup(&mut points);
    Polygon::new(points).map_err(AdaptiveError::Region)
}

fn arc_polygon(
    model: &LineArc,
    cut: AdaptiveCut,
    d0: f64,
    d1: f64,
    arc_steps: usize,
) -> Result<Polygon, AdaptiveError> {
    let start = model.arc_start_angle() + cut.removed_arc_angle;
    let end = model.arc_end_angle();
    let outer = arc_points(model, model.radius + d1, start, end, arc_steps);
    let inner = arc_points(model, model.radius + d0, start, end, arc_steps);
    let inner_end = *inner
        .last()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;
    let mut points = Vec::with_capacity(2 * arc_steps + 2);
    for point in &outer {
        push_unique(&mut points, *point);
    }
    push_unique(&mut points, inner_end);
    for point in inner.iter().rev().skip(1) {
        push_unique(&mut points, *point);
    }
    dedup(&mut points);
    Polygon::new(points).map_err(AdaptiveError::Region)
}

fn expected_polygon(
    model: &LineArc,
    cut: AdaptiveCut,
    seam: &[SeamSample],
    d0: f64,
    d1: f64,
    line_length: f64,
    arc_steps: usize,
) -> Result<Polygon, AdaptiveError> {
    let normal = Point2 {
        x: cut.theta.sin(),
        y: -cut.theta.cos(),
    };
    let q0 = Point2 {
        x: cut.tangent_point.x + d0 * normal.x,
        y: cut.tangent_point.y + d0 * normal.y,
    };
    let q1 = Point2 {
        x: cut.tangent_point.x + d1 * normal.x,
        y: cut.tangent_point.y + d1 * normal.y,
    };
    let start = model.arc_start_angle() + cut.removed_arc_angle;
    let end = model.arc_end_angle();
    let outer = arc_points(model, model.radius + d1, start, end, arc_steps);
    let inner = arc_points(model, model.radius + d0, start, end, arc_steps);
    let outer_end = *outer
        .last()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;
    let inner_end = *inner
        .last()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;
    let first = seam
        .first()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;
    let last = seam
        .last()
        .ok_or(AdaptiveError::Region(RegionError::EmptyPolygon))?;

    let mut points = Vec::with_capacity(2 * arc_steps + 8);
    points.push(Point2 {
        x: -line_length,
        y: -d1,
    });
    push_unique(&mut points, last.point);
    push_unique(&mut points, q1);
    for point in &outer {
        push_unique(&mut points, *point);
    }
    push_unique(&mut points, outer_end);
    push_unique(&mut points, inner_end);
    for point in inner.iter().rev().skip(1) {
        push_unique(&mut points, *point);
    }
    push_unique(&mut points, q0);
    push_unique(&mut points, first.point);
    push_unique(
        &mut points,
        Point2 {
            x: -line_length,
            y: -d0,
        },
    );
    dedup(&mut points);
    Polygon::new(points).map_err(AdaptiveError::Region)
}

fn arc_points(model: &LineArc, radius: f64, start: f64, end: f64, steps: usize) -> Vec<Point2> {
    let center = model.center();
    (0..=steps)
        .map(|index| {
            let t = start + (end - start) * index as f64 / steps as f64;
            Point2 {
                x: center.x + radius * t.cos(),
                y: center.y + radius * t.sin(),
            }
        })
        .collect()
}

fn push_unique(points: &mut Vec<Point2>, point: Point2) {
    if let Some(last) = points.last() {
        if last.distance_to(point) <= 1e-12 * scale_of(*last, point) {
            return;
        }
    }
    points.push(point);
}

fn dedup(points: &mut Vec<Point2>) {
    if points.len() > 2 {
        let first = points[0];
        let last = points[points.len() - 1];
        if first.distance_to(last) <= 1e-12 * scale_of(first, last) {
            points.pop();
        }
    }
}

fn scale_of(a: Point2, b: Point2) -> f64 {
    1.0 + a.x.abs() + a.y.abs() + b.x.abs() + b.y.abs()
}

fn point_segment_distance(p: Point2, a: Point2, b: Point2) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let length_squared = dx * dx + dy * dy;
    if length_squared <= 0.0 {
        return p.distance_to(a);
    }
    let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared;
    let t = t.clamp(0.0, 1.0);
    p.distance_to(Point2 {
        x: a.x + t * dx,
        y: a.y + t * dy,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sweep::{build_regions, Strategy};

    fn params(max_miter: f64, max_deviation: f64) -> AdaptiveParams {
        AdaptiveParams {
            max_miter_length: max_miter,
            max_path_deviation: max_deviation,
            preserve_endpoints: true,
            preserve_tangency: false,
        }
    }

    fn assert_valid(path: &AdaptivePath, line_length: f64, max_miter: f64, max_deviation: f64) {
        let cut = path.cut;
        assert!(cut.cut_point.x <= 0.0 && cut.cut_point.x >= -line_length);
        assert!(cut.removed_arc_angle > 0.0);
        assert!(cut.remaining_arc_angle > 0.0);
        assert!(cut.miter_length <= max_miter + 1e-9);
        assert!(cut.path_deviation <= max_deviation + 1e-9);
        let regions = path.build_regions(401, 96, 600).unwrap();
        assert!(
            !regions.self_intersecting,
            "line={:?} straight={:?} arc={:?} expected={:?}",
            regions.line.self_intersection_pairs(),
            regions.straight.self_intersection_pairs(),
            regions.arc.self_intersection_pairs(),
            regions.expected.self_intersection_pairs()
        );
        assert!(regions.area_balance_fraction.abs() < 1e-9);
        let expected = regions.raster.expected_area;
        assert!(expected > 0.0);
        assert!(regions.raster.overlap_area / expected < 1e-4);
        assert!(regions.raster.gap_area / expected < 1e-4);
        assert!(regions.raster.symmetric_difference_fraction < 1e-4);
    }

    #[test]
    fn adaptive_150_is_bounded_and_valid() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let path = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(6.5, 1.5)).unwrap();
        assert!(path.cut.miter_length <= 6.5 + 1e-9);
        assert!(path.cut.path_deviation <= 1.5 + 1e-9);
        assert!(path.cut.removed_arc_angle > 0.5);
        assert_valid(&path, 20.0, 6.5, 1.5);

        let regions = path.build_regions(401, 96, 600).unwrap();
        assert!(regions.expected.area() > 100.0);
    }

    #[test]
    fn adaptive_30_and_90_are_valid() {
        for (degrees, max_miter, max_deviation) in [(30.0_f64, 4.14, 0.5), (90.0, 5.0, 0.5)] {
            let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
            let path =
                AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(max_miter, max_deviation))
                    .unwrap_or_else(|error| panic!("{degrees}: {error:?}"));
            assert!(path.cut.miter_length <= max_miter + 1e-9);
            assert!(path.cut.path_deviation <= max_deviation + 1e-9);
            assert_valid(&path, 20.0, max_miter, max_deviation);
        }
    }

    #[test]
    fn adaptive_handles_near_opposite_tangents() {
        for (degrees, line_length, max_deviation) in [
            (170.0_f64, 100.0_f64, 2.6_f64),
            (175.0, 500.0, 3.0),
            (179.0, 500.0, 3.2),
        ] {
            let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
            let path =
                AdaptivePath::solve(model, -2.0, 2.0, line_length, params(6.5, max_deviation))
                    .unwrap_or_else(|error| panic!("{degrees}: {error:?}"));
            assert!(path.cut.miter_length <= 6.5 + 1e-9);
            assert!(path.cut.path_deviation <= max_deviation + 1e-9);
            assert!(path.cut.removed_arc_angle > 1.0);
            assert_valid(&path, line_length, 6.5, max_deviation);
        }
    }

    #[test]
    fn no_cut_is_reported_when_the_original_miter_fits() {
        let model = LineArc::new(4.0, 30.0_f64.to_radians()).unwrap();
        let error = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(6.5, 0.5)).unwrap_err();
        match error {
            AdaptiveError::NoCutNeeded { original_miter } => {
                assert!((original_miter - 4.141).abs() < 1e-3);
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn incompatible_bounds_are_reported() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let error = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(4.5, 1.0)).unwrap_err();
        match error {
            AdaptiveError::NoFeasibleCut {
                min_miter,
                min_deviation,
            } => {
                assert!(min_miter.is_finite());
                assert!(min_deviation.is_finite());
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn tangency_requirement_rejects_straight_transition() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let mut request = params(6.5, 1.5);
        request.preserve_tangency = true;
        assert!(matches!(
            AdaptivePath::solve(model, -2.0, 2.0, 20.0, request),
            Err(AdaptiveError::TangencyRequired)
        ));
    }

    #[test]
    fn adaptive_path_is_g1_at_the_arc_transition() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let path = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(6.5, 1.5)).unwrap();
        let cut = path.cut;
        let straight = Point2 {
            x: cut.theta.cos(),
            y: cut.theta.sin(),
        };
        let angle = model.arc_start_angle() + cut.removed_arc_angle;
        let arc_tangent = Point2 {
            x: -angle.sin(),
            y: angle.cos(),
        };
        let cross = straight.x * arc_tangent.y - straight.y * arc_tangent.x;
        let dot = straight.x * arc_tangent.x + straight.y * arc_tangent.y;
        assert!(cross.abs() < 1e-12, "cross={cross}");
        assert!(dot > 0.0, "tangent direction reversed");
    }

    #[test]
    fn adaptive_seam_is_collinear_and_on_the_offsets() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let path = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(6.5, 1.5)).unwrap();
        let p0 = path.seam_point(-2.0).unwrap();
        let p1 = path.seam_point(2.0).unwrap();
        let a = path.cut.cut_point;
        let cross = (p0.x - a.x) * (p1.y - a.y) - (p0.y - a.y) * (p1.x - a.x);
        assert!(cross.abs() < 1e-10, "seam not through A: {cross}");
        assert!((p0.y - 2.0).abs() < 1e-12);
        assert!((p1.y + 2.0).abs() < 1e-12);
    }

    #[test]
    fn adaptive_miter_is_shorter_than_the_hybrid_wedge() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let path = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(6.5, 1.5)).unwrap();
        let hybrid =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        let original_extent = hybrid
            .seam
            .iter()
            .map(|sample| sample.point.distance_to(Point2 { x: 0.0, y: 0.0 }))
            .fold(0.0_f64, f64::max);
        let original_chord = hybrid
            .seam
            .first()
            .unwrap()
            .point
            .distance_to(hybrid.seam.last().unwrap().point);
        assert!(path.cut.miter_extent < original_extent);
        assert!(path.cut.miter_length < original_chord);

        let adaptive = path.build_regions(401, 96, 600).unwrap();
        let area_difference = adaptive.expected.area() - hybrid.expected.area();
        assert!(area_difference < -1.0, "area difference={area_difference}");
    }

    #[test]
    fn dense_path_deviation_confirms_the_solver_metric() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let path = AdaptivePath::solve(model, -2.0, 2.0, 20.0, params(6.5, 1.5)).unwrap();
        let cut = path.cut;
        let dense = dense_deviation(&model, cut);
        assert!(
            (dense - cut.path_deviation).abs() < 1e-3,
            "dense={dense} cut={cut:?}"
        );
    }

    fn dense_deviation(model: &LineArc, cut: AdaptiveCut) -> f64 {
        let steps = 400;
        let center = model.center();
        let start = model.arc_start_angle();
        let a = cut.cut_point;
        let q = cut.tangent_point;
        let mut original = Vec::new();
        for index in 0..=steps {
            let t = index as f64 / steps as f64;
            original.push(Point2 {
                x: a.x * (1.0 - t),
                y: 0.0,
            });
        }
        for index in 1..=steps {
            let t = index as f64 / steps as f64;
            let angle = start + cut.removed_arc_angle * t;
            original.push(Point2 {
                x: center.x + model.radius * angle.cos(),
                y: center.y + model.radius * angle.sin(),
            });
        }
        let mut deviation = 0.0_f64;
        for point in &original {
            deviation = deviation.max(point_segment_distance(*point, a, q));
        }
        for index in 0..=steps {
            let t = index as f64 / steps as f64;
            let point = Point2 {
                x: a.x + (q.x - a.x) * t,
                y: a.y + (q.y - a.y) * t,
            };
            let mut distance = f64::INFINITY;
            for pair in original.windows(2) {
                distance = distance.min(point_segment_distance(point, pair[0], pair[1]));
            }
            deviation = deviation.max(distance);
        }
        deviation
    }
}
