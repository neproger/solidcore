//! 2D sweep-region construction and verification for the AOIM experiments.
//!
//! The module is deliberately independent of the SolidCore kernel. It builds
//! the line-strip and arc-annulus regions from the *actual prolonged profile
//! trajectories*, then checks whether the proposed seam is a common boundary
//! with no area overlap or gap.

use crate::{GeometryError, LineArc, Point2};

/// Seam construction being evaluated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Strategy {
    /// AOIM only; requires a real corresponding-offset intersection.
    Analytical,
    /// AOIM on `d >= 0`, tangent-line continuation on `d < 0`.
    HybridAtJunction,
    /// AOIM up to the existence limit, then the tangent at the vertex.
    ///
    /// The vertex tangent is horizontal, so it never meets `y = -d` below the
    /// limit. This variant is a negative control.
    HybridAtLimitTangent,
    /// AOIM up to the existence limit, then a line through the vertex parallel
    /// to the junction bisector. The connector is not tangent to the arc; this
    /// variant is also a negative control.
    HybridAtLimitBisector,
}

/// Failure to build or validate a pair of sweep regions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegionError {
    /// A geometric primitive failed.
    Geometry(GeometryError),
    /// The profile reaches offsets for which this strategy has no seam.
    UncoveredProfile {
        /// First sampled offset without a seam.
        first_missing: f64,
    },
    /// The profile interval or sampling parameters are invalid.
    InvalidProfile,
    /// The finite line is too short to contain the seam.
    LineTooShort,
    /// A polygon had fewer than three distinct points.
    EmptyPolygon,
}

impl From<GeometryError> for RegionError {
    fn from(error: GeometryError) -> Self {
        Self::Geometry(error)
    }
}

/// A sampled seam point and its signed profile offset.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamSample {
    /// Signed offset to the right of travel.
    pub d: f64,
    /// Seam point on both prolonged trajectories.
    pub point: Point2,
}

/// A closed 2D polygon with sampled arcs.
#[derive(Debug, Clone)]
pub struct Polygon {
    /// Vertices in order (the closing edge is implicit).
    pub points: Vec<Point2>,
}

impl Polygon {
    /// Build a polygon, rejecting fewer than three points.
    pub fn new(mut points: Vec<Point2>) -> Result<Self, RegionError> {
        if points.len() > 2 {
            let first = points[0];
            let last = points[points.len() - 1];
            if first.distance_to(last) <= 1e-12 * scale_of(first, last) {
                points.pop();
            }
        }
        if points.len() < 3 {
            return Err(RegionError::EmptyPolygon);
        }
        Ok(Self { points })
    }

    /// Absolute area by the shoelace formula.
    pub fn area(&self) -> f64 {
        let n = self.points.len();
        if n < 3 {
            return 0.0;
        }
        let mut twice = 0.0;
        for index in 0..n {
            let a = self.points[index];
            let b = self.points[(index + 1) % n];
            twice += a.x * b.y - b.x * a.y;
        }
        0.5 * twice.abs()
    }

    /// Point-in-polygon test using the even-odd rule.
    pub fn contains(&self, p: Point2) -> bool {
        let n = self.points.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        for index in 0..n {
            let a = self.points[index];
            let b = self.points[(index + 1) % n];
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if p.x < x {
                    inside = !inside;
                }
            }
        }
        inside
    }

    /// Shortest distance from `p` to the polygon boundary.
    pub fn boundary_distance(&self, p: Point2) -> f64 {
        let n = self.points.len();
        if n < 2 {
            return f64::INFINITY;
        }
        let mut best = f64::INFINITY;
        for index in 0..n {
            let a = self.points[index];
            let b = self.points[(index + 1) % n];
            best = best.min(point_segment_distance(p, a, b));
        }
        best
    }

    /// Pairs of non-adjacent edges that cross properly.
    pub fn self_intersection_pairs(&self) -> Vec<(usize, usize)> {
        let n = self.points.len();
        let mut pairs = Vec::new();
        if n < 4 {
            return pairs;
        }
        for first in 0..n {
            let a = self.points[first];
            let b = self.points[(first + 1) % n];
            for second in (first + 1)..n {
                if second == first + 1 || (first == 0 && second == n - 1) {
                    continue;
                }
                let c = self.points[second];
                let d = self.points[(second + 1) % n];
                if points_close(a, c)
                    || points_close(a, d)
                    || points_close(b, c)
                    || points_close(b, d)
                {
                    continue;
                }
                if segments_cross(a, b, c, d) {
                    pairs.push((first, second));
                }
            }
        }
        pairs
    }

    /// Whether two non-adjacent edges cross properly.
    pub fn is_self_intersecting(&self) -> bool {
        !self.self_intersection_pairs().is_empty()
    }
}

/// Area metrics from a raster comparison of the two trimmed regions and the
/// expected outer contour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RasterMetrics {
    /// Grid resolution per side.
    pub resolution: usize,
    /// Area of one cell.
    pub cell_area: f64,
    /// Area covered by the expected outer contour.
    pub expected_area: f64,
    /// Area covered by the union of the two trimmed regions.
    pub union_area: f64,
    /// Area covered by both trimmed regions.
    pub overlap_area: f64,
    /// Area inside the expected contour but outside both regions.
    pub gap_area: f64,
    /// Area outside the expected contour but inside a trimmed region.
    pub extra_area: f64,
    /// `gap_area + extra_area`.
    pub symmetric_difference_area: f64,
    /// `symmetric_difference_area / expected_area`.
    pub symmetric_difference_fraction: f64,
}

/// Regions plus seam and consistency metrics.
#[derive(Debug, Clone)]
pub struct BuiltRegions {
    /// Trimmed line-sweep region.
    pub line: Polygon,
    /// Trimmed arc-sweep region, including any local prolongation.
    pub arc: Polygon,
    /// Expected outer contour, built from the same prolonged trajectories but
    /// without using the seam as an outer edge.
    pub expected: Polygon,
    /// Seam samples from `d0` to `d1`.
    pub seam: Vec<SeamSample>,
    /// `line.area() + arc.area() - expected.area()`.
    pub area_balance: f64,
    /// `area_balance / expected.area()`.
    pub area_balance_fraction: f64,
    /// Whether either trimmed polygon self-intersects.
    pub self_intersecting: bool,
    /// Raster comparison metrics.
    pub raster: RasterMetrics,
}

/// Seam point for a strategy at offset `d`.
pub fn seam_point(model: &LineArc, strategy: Strategy, d: f64) -> Result<Point2, GeometryError> {
    match strategy {
        Strategy::Analytical => model.intersect_offsets(d),
        Strategy::HybridAtJunction => {
            if d >= 0.0 {
                model.intersect_offsets(d)
            } else {
                model.junction_linear_point(d)
            }
        }
        Strategy::HybridAtLimitTangent => {
            if d >= model.min_offset() {
                model.intersect_offsets(d)
            } else {
                Err(GeometryError::NoRealIntersection)
            }
        }
        Strategy::HybridAtLimitBisector => {
            if d >= model.min_offset() {
                model.intersect_offsets(d)
            } else {
                model.limit_linear_point(d)
            }
        }
    }
}

/// Arc-side boundary for one profile offset, from the seam to the arc end.
///
/// For `d < 0` under [`Strategy::HybridAtJunction`] the path is a straight
/// tangent segment from the seam to the joint cross-section, followed by the
/// finite arc. For [`Strategy::Analytical`] the path stays on the supporting
/// circle, i.e. it uses a circular extension before the joint.
pub fn arc_boundary(
    model: &LineArc,
    strategy: Strategy,
    d: f64,
    steps: usize,
) -> Result<Vec<Point2>, GeometryError> {
    let seam = seam_point(model, strategy, d)?;
    let radius = model.radius + d;
    if radius <= 0.0 {
        return Err(GeometryError::CollapsedOffsetCircle);
    }
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();

    let analytic = || {
        let parameter = model.arc_parameter(seam);
        arc_points(model, radius, start + parameter, end, steps)
    };

    match strategy {
        Strategy::Analytical => Ok(analytic()),
        Strategy::HybridAtJunction => {
            if d >= 0.0 {
                Ok(analytic())
            } else {
                let joint = model.joint_arc_point(d)?;
                let mut points = Vec::with_capacity(steps + 3);
                push_unique(&mut points, seam);
                push_unique(&mut points, joint);
                for point in arc_points(model, radius, start, end, steps) {
                    push_unique(&mut points, point);
                }
                Ok(points)
            }
        }
        Strategy::HybridAtLimitTangent => {
            if d >= model.min_offset() {
                Ok(analytic())
            } else {
                Err(GeometryError::NoRealIntersection)
            }
        }
        Strategy::HybridAtLimitBisector => {
            if d >= model.min_offset() {
                Ok(analytic())
            } else {
                let joint = model.joint_arc_point(d)?;
                let mut points = Vec::with_capacity(steps + 3);
                push_unique(&mut points, seam);
                push_unique(&mut points, joint);
                for point in arc_points(model, radius, start, end, steps) {
                    push_unique(&mut points, point);
                }
                Ok(points)
            }
        }
    }
}

/// Build both trimmed 2D regions, the expected outer contour, and metrics.
///
/// `line_length` is the distance from the joint back along the incoming line
/// (`x = -line_length`). It must be large enough to contain every seam point.
pub fn build_regions(
    model: &LineArc,
    strategy: Strategy,
    d0: f64,
    d1: f64,
    line_length: f64,
    seam_samples: usize,
    arc_steps: usize,
) -> Result<BuiltRegions, RegionError> {
    if !d0.is_finite()
        || !d1.is_finite()
        || !line_length.is_finite()
        || d0 >= d1
        || line_length <= 0.0
        || seam_samples < 2
        || arc_steps < 1
    {
        return Err(RegionError::InvalidProfile);
    }
    if model.radius + d0 <= 0.0 || model.radius + d1 <= 0.0 {
        return Err(RegionError::Geometry(GeometryError::CollapsedOffsetCircle));
    }

    let mut seam = Vec::with_capacity(seam_samples);
    for index in 0..seam_samples {
        let t = index as f64 / (seam_samples - 1) as f64;
        let d = d0 + (d1 - d0) * t;
        match seam_point(model, strategy, d) {
            Ok(point) => seam.push(SeamSample { d, point }),
            Err(GeometryError::NoRealIntersection) => {
                return Err(RegionError::UncoveredProfile { first_missing: d });
            }
            Err(error) => return Err(RegionError::Geometry(error)),
        }
    }

    for sample in &seam {
        if sample.point.x < -line_length {
            return Err(RegionError::LineTooShort);
        }
    }

    let line = line_polygon(&seam, d0, d1, line_length)?;
    let arc = arc_polygon(model, strategy, d0, d1, &seam, arc_steps)?;
    let expected = expected_polygon(model, strategy, d0, d1, &seam, line_length, arc_steps)?;

    let line_area = line.area();
    let arc_area = arc.area();
    let expected_area = expected.area();
    let area_balance = line_area + arc_area - expected_area;
    let area_balance_fraction = if expected_area > 0.0 {
        area_balance / expected_area
    } else {
        0.0
    };
    let self_intersecting = line.is_self_intersecting() || arc.is_self_intersecting();
    let raster = raster_metrics(&line, &arc, &expected, 600)?;

    Ok(BuiltRegions {
        line,
        arc,
        expected,
        seam,
        area_balance,
        area_balance_fraction,
        self_intersecting,
        raster,
    })
}

fn line_polygon(
    seam: &[SeamSample],
    d0: f64,
    d1: f64,
    line_length: f64,
) -> Result<Polygon, RegionError> {
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
    Polygon::new(points)
}

fn arc_polygon(
    model: &LineArc,
    strategy: Strategy,
    d0: f64,
    d1: f64,
    seam: &[SeamSample],
    arc_steps: usize,
) -> Result<Polygon, RegionError> {
    let outer = arc_boundary(model, strategy, d1, arc_steps)?;
    let inner = arc_boundary(model, strategy, d0, arc_steps)?;
    let inner_end = *inner.last().ok_or(RegionError::EmptyPolygon)?;

    let mut points = Vec::with_capacity(2 * arc_steps + seam.len() + 4);
    for point in &outer {
        push_unique(&mut points, *point);
    }
    push_unique(&mut points, inner_end);
    for point in inner.iter().rev().skip(1) {
        push_unique(&mut points, *point);
    }
    for sample in seam {
        push_unique(&mut points, sample.point);
    }
    Polygon::new(points)
}

fn expected_polygon(
    model: &LineArc,
    strategy: Strategy,
    d0: f64,
    d1: f64,
    seam: &[SeamSample],
    line_length: f64,
    arc_steps: usize,
) -> Result<Polygon, RegionError> {
    let outer = arc_boundary(model, strategy, d1, arc_steps)?;
    let inner = arc_boundary(model, strategy, d0, arc_steps)?;
    let outer_end = *outer.last().ok_or(RegionError::EmptyPolygon)?;
    let inner_end = *inner.last().ok_or(RegionError::EmptyPolygon)?;
    let seam_end = seam.last().ok_or(RegionError::EmptyPolygon)?.point;

    let mut points = Vec::with_capacity(2 * arc_steps + 4);
    points.push(Point2 {
        x: -line_length,
        y: -d1,
    });
    push_unique(&mut points, seam_end);
    for point in &outer {
        push_unique(&mut points, *point);
    }
    push_unique(&mut points, outer_end);
    push_unique(&mut points, inner_end);
    for point in inner.iter().rev().skip(1) {
        push_unique(&mut points, *point);
    }
    push_unique(
        &mut points,
        Point2 {
            x: -line_length,
            y: -d0,
        },
    );
    Polygon::new(points)
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

fn points_close(a: Point2, b: Point2) -> bool {
    a.distance_to(b) <= 1e-12 * scale_of(a, b)
}

fn push_unique(points: &mut Vec<Point2>, point: Point2) {
    if let Some(last) = points.last() {
        if last.distance_to(point) <= 1e-12 * scale_of(*last, point) {
            return;
        }
    }
    points.push(point);
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
    let closest = Point2 {
        x: a.x + t * dx,
        y: a.y + t * dy,
    };
    p.distance_to(closest)
}

fn orient(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn segments_cross(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    o1 * o2 < 0.0 && o3 * o4 < 0.0
}

fn raster_metrics(
    line: &Polygon,
    arc: &Polygon,
    expected: &Polygon,
    resolution: usize,
) -> Result<RasterMetrics, RegionError> {
    let resolution = resolution.max(16);
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for polygon in [line, arc, expected] {
        for point in &polygon.points {
            min_x = min_x.min(point.x);
            max_x = max_x.max(point.x);
            min_y = min_y.min(point.y);
            max_y = max_y.max(point.y);
        }
    }
    if !min_x.is_finite() || !max_x.is_finite() || max_x <= min_x || max_y <= min_y {
        return Err(RegionError::EmptyPolygon);
    }
    let pad = 0.01 * (max_x - min_x).max(max_y - min_y);
    min_x -= pad;
    max_x += pad;
    min_y -= pad;
    max_y += pad;

    let line_cells = rasterize(line, min_x, max_x, min_y, max_y, resolution);
    let arc_cells = rasterize(arc, min_x, max_x, min_y, max_y, resolution);
    let expected_cells = rasterize(expected, min_x, max_x, min_y, max_y, resolution);

    let mut expected_count = 0usize;
    let mut union_count = 0usize;
    let mut overlap_count = 0usize;
    let mut gap_count = 0usize;
    let mut extra_count = 0usize;
    for index in 0..line_cells.len() {
        let in_line = line_cells[index];
        let in_arc = arc_cells[index];
        let in_expected = expected_cells[index];
        let in_union = in_line || in_arc;
        if in_expected {
            expected_count += 1;
        }
        if in_union {
            union_count += 1;
        }
        if in_line && in_arc {
            overlap_count += 1;
        }
        if in_expected && !in_union {
            gap_count += 1;
        }
        if in_union && !in_expected {
            extra_count += 1;
        }
    }

    let cell_area = (max_x - min_x) * (max_y - min_y) / (resolution * resolution) as f64;
    let expected_area = expected_count as f64 * cell_area;
    let symmetric_difference_area = (gap_count + extra_count) as f64 * cell_area;
    let symmetric_difference_fraction = if expected_area > 0.0 {
        symmetric_difference_area / expected_area
    } else {
        0.0
    };

    Ok(RasterMetrics {
        resolution,
        cell_area,
        expected_area,
        union_area: union_count as f64 * cell_area,
        overlap_area: overlap_count as f64 * cell_area,
        gap_area: gap_count as f64 * cell_area,
        extra_area: extra_count as f64 * cell_area,
        symmetric_difference_area,
        symmetric_difference_fraction,
    })
}

fn rasterize(
    polygon: &Polygon,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    resolution: usize,
) -> Vec<bool> {
    let mut cells = vec![false; resolution * resolution];
    let cell_width = (max_x - min_x) / resolution as f64;
    let cell_height = (max_y - min_y) / resolution as f64;
    let n = polygon.points.len();
    if n < 3 {
        return cells;
    }

    for row in 0..resolution {
        let y = min_y + (row as f64 + 0.5) * cell_height;
        let mut intersections = Vec::new();
        for index in 0..n {
            let a = polygon.points[index];
            let b = polygon.points[(index + 1) % n];
            if (a.y > y) != (b.y > y) {
                let t = (y - a.y) / (b.y - a.y);
                intersections.push(a.x + t * (b.x - a.x));
            }
        }
        intersections.sort_by(f64::total_cmp);
        for pair in intersections.as_chunks::<2>().0 {
            let first = pair[0];
            let second = pair[1];
            let low = first.min(second);
            let high = first.max(second);
            let start = (((low - min_x) / cell_width) - 0.5).ceil() as isize;
            let end = (((high - min_x) / cell_width) - 0.5).floor() as isize;
            let start = start.max(0);
            let end = end.min(resolution as isize - 1);
            for column in start..=end {
                cells[row * resolution + column as usize] = true;
            }
        }
    }
    cells
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn sharp() -> LineArc {
        LineArc::new(4.0, 150.0_f64.to_radians()).unwrap()
    }

    fn assert_hybrid_ok(model: &LineArc, d0: f64, d1: f64) {
        let built = build_regions(model, Strategy::HybridAtJunction, d0, d1, 20.0, 401, 96)
            .unwrap_or_else(|error| panic!("build failed: {error:?}"));
        assert!(
            !built.self_intersecting,
            "self-intersection: line={:?} arc={:?}",
            built.line.self_intersection_pairs(),
            built.arc.self_intersection_pairs()
        );
        assert!(
            built.area_balance_fraction.abs() < 1e-9,
            "area balance: {} ({})",
            built.area_balance,
            built.area_balance_fraction
        );
        let expected = built.raster.expected_area;
        assert!(expected > 0.0);
        assert!(
            built.raster.overlap_area / expected < 1e-4,
            "overlap: {}",
            built.raster.overlap_area / expected
        );
        assert!(
            built.raster.gap_area / expected < 1e-4,
            "gap: {}",
            built.raster.gap_area / expected
        );
        assert!(
            built.raster.symmetric_difference_fraction < 1e-4,
            "symmetric difference: {}",
            built.raster.symmetric_difference_fraction
        );
    }

    #[test]
    fn hybrid_junction_covers_the_sharp_corner() {
        let model = sharp();
        assert_hybrid_ok(&model, -2.0, 2.0);
        assert_hybrid_ok(&model, -2.0, -0.5);
        assert_hybrid_ok(&model, 0.5, 2.0);
    }

    #[test]
    fn analytical_cannot_cover_the_sharp_corner() {
        let model = sharp();
        let error =
            build_regions(&model, Strategy::Analytical, -2.0, 2.0, 20.0, 401, 96).unwrap_err();
        match error {
            RegionError::UncoveredProfile { first_missing } => {
                assert!((first_missing + 2.0).abs() < 1e-12);
            }
            other => panic!("unexpected error: {other:?}"),
        }
        let minimum = model.min_offset();
        let covered_fraction = (2.0 - minimum) / 4.0;
        assert!((covered_fraction - 0.566_987_3).abs() < 1e-6);
        assert_eq!(
            seam_point(&model, Strategy::Analytical, minimum - 1e-6),
            Err(GeometryError::NoRealIntersection)
        );
    }

    #[test]
    fn hybrid_matches_analytical_on_a_positive_profile() {
        let model = sharp();
        let analytical =
            build_regions(&model, Strategy::Analytical, 0.5, 2.0, 20.0, 401, 96).unwrap();
        let hybrid =
            build_regions(&model, Strategy::HybridAtJunction, 0.5, 2.0, 20.0, 401, 96).unwrap();
        assert!((analytical.line.area() - hybrid.line.area()).abs() < 1e-10);
        assert!((analytical.arc.area() - hybrid.arc.area()).abs() < 1e-10);
        assert!((analytical.expected.area() - hybrid.expected.area()).abs() < 1e-10);
        for (a, b) in analytical.seam.iter().zip(hybrid.seam.iter()) {
            assert!(a.point.distance_to(b.point) < 1e-12);
        }
    }

    #[test]
    fn junction_connector_is_the_arc_tangent() {
        let model = sharp();
        let tangent = model.joint_tangent();
        let half = 0.5 * model.corner_angle_rad;
        let expected_parameter = half.sin() / half.cos();
        for d in [-0.01_f64, -0.1, -0.5, -1.0, -2.0] {
            let seam = model.junction_linear_point(d).unwrap();
            let joint = model.joint_arc_point(d).unwrap();
            let delta = Point2 {
                x: seam.x - joint.x,
                y: seam.y - joint.y,
            };
            let cross = delta.x * tangent.y - delta.y * tangent.x;
            let along = delta.x * tangent.x + delta.y * tangent.y;
            let scale = 1.0 + delta.x.abs() + delta.y.abs();
            assert!(cross.abs() <= 1e-12 * scale, "d={d} cross={cross}");
            assert!(
                (along - d * expected_parameter).abs() <= 1e-12 * scale,
                "d={d} along={along}"
            );
            assert!(along < 0.0, "d={d} must extend backwards");
        }
    }

    #[test]
    fn internal_bisector_is_not_tangent_to_the_arc() {
        let model = sharp();
        let tangent = model.joint_tangent();
        let mut max_error = 0.0_f64;
        for d in [-0.1_f64, -0.5, -1.0, -2.0] {
            let internal = model.internal_junction_point(d).unwrap();
            let joint = model.joint_arc_point(d).unwrap();
            let delta = Point2 {
                x: internal.x - joint.x,
                y: internal.y - joint.y,
            };
            let cross = delta.x * tangent.y - delta.y * tangent.x;
            max_error = max_error.max(cross.abs());
        }
        assert!(max_error > 1e-3, "internal connector is tangent");
    }

    #[test]
    fn limit_bisector_closes_but_is_not_a_linear_sweep_extension() {
        let model = sharp();
        let built = build_regions(
            &model,
            Strategy::HybridAtLimitBisector,
            -2.0,
            2.0,
            20.0,
            401,
            96,
        )
        .unwrap();
        // The area construction can be closed...
        assert!(built.raster.symmetric_difference_fraction < 1e-4);
        assert!(!built.self_intersecting);

        // ...but the connector direction is not the arc tangent and varies with d.
        let tangent = model.joint_tangent();
        let mut min_angle = f64::INFINITY;
        let mut max_angle = f64::NEG_INFINITY;
        for d in [-2.0_f64, -1.5, -1.0, -0.5, model.min_offset() - 1e-6] {
            let seam = model.limit_linear_point(d).unwrap();
            let joint = model.joint_arc_point(d).unwrap();
            let delta = Point2 {
                x: seam.x - joint.x,
                y: seam.y - joint.y,
            };
            let cross = delta.x * tangent.y - delta.y * tangent.x;
            assert!(cross.abs() > 1e-6, "d={d} unexpectedly tangent");
            let angle = delta.y.atan2(delta.x);
            min_angle = min_angle.min(angle);
            max_angle = max_angle.max(angle);
        }
        assert!(
            max_angle - min_angle > 1e-3,
            "connector direction should vary: {min_angle}..{max_angle}"
        );
    }

    #[test]
    fn limit_tangent_has_no_seam_below_the_limit() {
        let model = sharp();
        let minimum = model.min_offset();
        assert_eq!(
            seam_point(&model, Strategy::HybridAtLimitTangent, minimum - 1e-6),
            Err(GeometryError::NoRealIntersection)
        );
        assert_eq!(
            seam_point(&model, Strategy::HybridAtLimitTangent, -2.0),
            Err(GeometryError::NoRealIntersection)
        );
    }

    #[test]
    fn hybrid_seam_is_g1_at_the_junction() {
        let model = sharp();
        let epsilon = 1e-7;
        let left = seam_point(&model, Strategy::HybridAtJunction, -epsilon).unwrap();
        let right = seam_point(&model, Strategy::HybridAtJunction, epsilon).unwrap();
        let left_slope = left.y / left.x;
        let right_slope = right.y / right.x;
        let expected = model.tangent_slope_at_joint();
        assert!((left_slope - expected).abs() < 1e-6);
        assert!((right_slope - expected).abs() < 1e-6);
        assert!((left_slope - right_slope).abs() < 1e-6);
    }

    #[test]
    fn hybrid_seam_lies_on_both_trimmed_boundaries() {
        let model = sharp();
        let built =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        for sample in built.seam.iter().step_by(10) {
            assert!(
                built.line.boundary_distance(sample.point) < 1e-9,
                "line boundary at d={}",
                sample.d
            );
            assert!(
                built.arc.boundary_distance(sample.point) < 1e-9,
                "arc boundary at d={}",
                sample.d
            );
        }
    }

    #[test]
    fn hybrid_regions_are_on_opposite_sides_of_the_seam() {
        let model = sharp();
        let built =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        let mut checked = 0;
        for window in built.seam.windows(3).step_by(20) {
            let before = window[0].point;
            let at = window[1].point;
            let after = window[2].point;
            let tangent = Point2 {
                x: after.x - before.x,
                y: after.y - before.y,
            };
            let length = (tangent.x * tangent.x + tangent.y * tangent.y).sqrt();
            let normal = Point2 {
                x: -tangent.y / length,
                y: tangent.x / length,
            };
            let offset = 1e-4 * model.radius;
            let first = Point2 {
                x: at.x + normal.x * offset,
                y: at.y + normal.y * offset,
            };
            let second = Point2 {
                x: at.x - normal.x * offset,
                y: at.y - normal.y * offset,
            };
            let first_line = built.line.contains(first);
            let first_arc = built.arc.contains(first);
            let second_line = built.line.contains(second);
            let second_arc = built.arc.contains(second);
            assert!(
                (first_line && second_arc) || (first_arc && second_line),
                "d={} first=({first_line},{first_arc}) second=({second_line},{second_arc})",
                window[1].d
            );
            checked += 1;
        }
        assert!(checked > 0);
    }

    #[test]
    fn ninety_degree_hybrid_and_analytical_are_both_closed() {
        let model = LineArc::new(4.0, FRAC_PI_2).unwrap();
        let hybrid =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        assert!(!hybrid.self_intersecting);
        assert!(hybrid.area_balance_fraction.abs() < 1e-9);
        assert!(hybrid.raster.symmetric_difference_fraction < 1e-4);

        let analytical =
            build_regions(&model, Strategy::Analytical, -2.0, 2.0, 20.0, 401, 96).unwrap();
        // The circular extension touches the seam tangentially at d_min; the
        // sampled polygon reports a tangential self-contact. The hybrid avoids
        // that degeneracy.
        assert!(analytical.self_intersecting);
        assert!(analytical.area_balance_fraction.abs() < 1e-9);
    }

    #[test]
    fn hybrid_junction_works_for_thirty_degrees() {
        let model = LineArc::new(4.0, 30.0_f64.to_radians()).unwrap();
        assert_hybrid_ok(&model, -2.0, 2.0);
        assert_hybrid_ok(&model, -2.0, -0.5);
    }
}

/// Metrics for the original (untrimmed) line strip and finite arc sector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UntrimmedMetrics {
    /// Grid resolution per side.
    pub resolution: usize,
    /// Area of one cell.
    pub cell_area: f64,
    /// Line-strip area.
    pub line_area: f64,
    /// Finite arc-sector area.
    pub arc_area: f64,
    /// Area where the two original regions overlap.
    pub overlap_area: f64,
    /// Area of their union.
    pub union_area: f64,
}

/// Build the original untrimmed line strip and finite arc sector.
///
/// These are the overlapping bodies *before* the seam is applied. They are
/// kept separate from [`BuiltRegions`] so the initial overlap can be compared
/// with the trimmed result.
pub fn build_untrimmed(
    model: &LineArc,
    d0: f64,
    d1: f64,
    line_length: f64,
    arc_steps: usize,
    resolution: usize,
) -> Result<(Polygon, Polygon, UntrimmedMetrics), RegionError> {
    if !d0.is_finite()
        || !d1.is_finite()
        || !line_length.is_finite()
        || d0 >= d1
        || line_length <= 0.0
        || arc_steps < 1
    {
        return Err(RegionError::InvalidProfile);
    }
    if model.radius + d0 <= 0.0 || model.radius + d1 <= 0.0 {
        return Err(RegionError::Geometry(GeometryError::CollapsedOffsetCircle));
    }

    let line = Polygon::new(vec![
        Point2 {
            x: -line_length,
            y: -d1,
        },
        Point2 { x: 0.0, y: -d1 },
        Point2 { x: 0.0, y: -d0 },
        Point2 {
            x: -line_length,
            y: -d0,
        },
    ])?;

    let start = model.arc_start_angle();
    let end = model.arc_end_angle();
    let outer = arc_points(model, model.radius + d1, start, end, arc_steps);
    let inner = arc_points(model, model.radius + d0, start, end, arc_steps);
    let mut points = outer;
    for point in inner.iter().rev() {
        push_unique(&mut points, *point);
    }
    let arc = Polygon::new(points)?;

    let resolution = resolution.max(16);
    let (min_x, max_x, min_y, max_y) = polygon_bounds(&[&line, &arc]);
    let line_cells = rasterize(&line, min_x, max_x, min_y, max_y, resolution);
    let arc_cells = rasterize(&arc, min_x, max_x, min_y, max_y, resolution);
    let mut line_count = 0usize;
    let mut arc_count = 0usize;
    let mut overlap_count = 0usize;
    let mut union_count = 0usize;
    for index in 0..line_cells.len() {
        let in_line = line_cells[index];
        let in_arc = arc_cells[index];
        if in_line {
            line_count += 1;
        }
        if in_arc {
            arc_count += 1;
        }
        if in_line && in_arc {
            overlap_count += 1;
        }
        if in_line || in_arc {
            union_count += 1;
        }
    }
    let cell_area = (max_x - min_x) * (max_y - min_y) / (resolution * resolution) as f64;
    Ok((
        line,
        arc,
        UntrimmedMetrics {
            resolution,
            cell_area,
            line_area: line_count as f64 * cell_area,
            arc_area: arc_count as f64 * cell_area,
            overlap_area: overlap_count as f64 * cell_area,
            union_area: union_count as f64 * cell_area,
        },
    ))
}

fn polygon_bounds(polygons: &[&Polygon]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for polygon in polygons {
        for point in &polygon.points {
            min_x = min_x.min(point.x);
            max_x = max_x.max(point.x);
            min_y = min_y.min(point.y);
            max_y = max_y.max(point.y);
        }
    }
    let pad = 0.01 * (max_x - min_x).max(max_y - min_y);
    (min_x - pad, max_x + pad, min_y - pad, max_y + pad)
}

#[cfg(test)]
mod extra_tests {
    use super::*;

    #[test]
    fn untrimmed_overlap_is_resolved_by_the_hybrid_seam() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let (_, _, untrimmed) = build_untrimmed(&model, -2.0, 2.0, 20.0, 96, 600).unwrap();
        assert!(untrimmed.overlap_area > 0.1, "no initial overlap");
        let trimmed =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        assert!(trimmed.raster.overlap_area / trimmed.raster.expected_area < 1e-4);
        assert!(trimmed.raster.gap_area / trimmed.raster.expected_area < 1e-4);
    }

    #[test]
    fn hybrid_handles_wide_and_near_collapse_profiles() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        for (d0, d1, line_length) in [
            (-0.5_f64, 0.5_f64, 20.0_f64),
            (-3.5, 3.5, 30.0),
            (-3.99, 3.99, 30.0),
        ] {
            let built = build_regions(
                &model,
                Strategy::HybridAtJunction,
                d0,
                d1,
                line_length,
                401,
                96,
            )
            .unwrap();
            assert!(!built.self_intersecting, "d=[{d0},{d1}]");
            assert!(
                built.area_balance_fraction.abs() < 1e-9,
                "d=[{d0},{d1}] balance={}",
                built.area_balance_fraction
            );
        }
    }

    #[test]
    fn hybrid_handles_steep_and_shallow_angles() {
        for (degrees, line_length) in [
            (1.0_f64, 20.0_f64),
            (30.0, 20.0),
            (90.0, 20.0),
            (150.0, 20.0),
            (170.0, 100.0),
            (179.0, 300.0),
        ] {
            let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
            let built = build_regions(
                &model,
                Strategy::HybridAtJunction,
                -2.0,
                2.0,
                line_length,
                401,
                96,
            )
            .unwrap_or_else(|error| panic!("{degrees} deg: {error:?}"));
            assert!(!built.self_intersecting, "{degrees} deg");
            assert!(
                built.area_balance_fraction.abs() < 1e-9,
                "{degrees} deg balance={}",
                built.area_balance_fraction
            );
        }
    }
}
/// Compare any number of trimmed parts against one expected outer contour.
///
/// A raster cell counts as overlap when it is inside more than one part. This
/// is the generic form used by the three-part adaptive-path builder.
pub fn raster_compare(
    parts: &[&Polygon],
    expected: &Polygon,
    resolution: usize,
) -> Result<RasterMetrics, RegionError> {
    let resolution = resolution.max(16);
    let mut all = parts.to_vec();
    all.push(expected);
    let (min_x, max_x, min_y, max_y) = polygon_bounds(&all);
    if !min_x.is_finite() || !max_x.is_finite() || max_x <= min_x || max_y <= min_y {
        return Err(RegionError::EmptyPolygon);
    }

    let cells: Vec<Vec<bool>> = parts
        .iter()
        .map(|polygon| rasterize(polygon, min_x, max_x, min_y, max_y, resolution))
        .collect();
    let expected_cells = rasterize(expected, min_x, max_x, min_y, max_y, resolution);

    let mut expected_count = 0usize;
    let mut union_count = 0usize;
    let mut overlap_count = 0usize;
    let mut gap_count = 0usize;
    let mut extra_count = 0usize;
    for index in 0..expected_cells.len() {
        let inside = cells.iter().filter(|cells| cells[index]).count();
        let in_union = inside > 0;
        let in_expected = expected_cells[index];
        if in_expected {
            expected_count += 1;
        }
        if in_union {
            union_count += 1;
        }
        if inside > 1 {
            overlap_count += 1;
        }
        if in_expected && !in_union {
            gap_count += 1;
        }
        if in_union && !in_expected {
            extra_count += 1;
        }
    }

    let cell_area = (max_x - min_x) * (max_y - min_y) / (resolution * resolution) as f64;
    let expected_area = expected_count as f64 * cell_area;
    let symmetric_difference_area = (gap_count + extra_count) as f64 * cell_area;
    let symmetric_difference_fraction = if expected_area > 0.0 {
        symmetric_difference_area / expected_area
    } else {
        0.0
    };

    Ok(RasterMetrics {
        resolution,
        cell_area,
        expected_area,
        union_area: union_count as f64 * cell_area,
        overlap_area: overlap_count as f64 * cell_area,
        gap_area: gap_count as f64 * cell_area,
        extra_area: extra_count as f64 * cell_area,
        symmetric_difference_area,
        symmetric_difference_fraction,
    })
}
