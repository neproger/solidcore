//! Experimental 3D extrusion of the trimmed AOIM 2D regions.
//!
//! The first 3D step is deliberately narrow: a rectangular profile in the
//! `(offset, height)` plane, a planar Line–Arc path, and a fixed global `Z`
//! section direction. The resulting bodies are prisms over the already
//! validated 2D regions. The seam surface is built once from the 2D seam and
//! referenced by both bodies with opposite orientations; the combined shell is
//! obtained by removing the internal seam faces, not by a Boolean.

use crate::sweep::{build_regions, RegionError, Strategy};
use crate::{LineArc, Point2};
use std::collections::{BTreeMap, BTreeSet};

type GridPoint = (i64, i64);
type SegmentKey = (GridPoint, GridPoint);
type SegmentSet = BTreeSet<SegmentKey>;

/// A point in the sweep space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
    /// Z coordinate (profile height direction).
    pub z: f64,
}

impl Point3 {
    /// Euclidean distance.
    pub fn distance_to(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Role of a 3D face.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceKind {
    /// Top profile face (`z = z1`).
    Top,
    /// Bottom profile face (`z = z0`).
    Bottom,
    /// Outer side wall.
    Side,
    /// Internal seam face shared by the two bodies.
    Seam,
}

/// An oriented polygon face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Face3 {
    /// Vertex indices in oriented order.
    pub vertices: Vec<usize>,
    /// Face role.
    pub kind: FaceKind,
}

/// A closed body face list.
#[derive(Debug, Clone)]
pub struct Body3 {
    /// Oriented faces.
    pub faces: Vec<Face3>,
}

/// Failure of the 3D construction or validation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Solid3Error {
    /// 2D region construction failed.
    Region(RegionError),
    /// Profile height or sampling parameters are invalid.
    InvalidProfile,
    /// A face is degenerate.
    Degenerate,
    /// An edge is not used exactly twice, or is used twice in the same direction.
    NotClosed,
    /// The two bodies do not expose the same seam face geometry.
    SeamMismatch,
    /// Volume or seam area does not match the 2D reference.
    MetricMismatch,
}

impl From<RegionError> for Solid3Error {
    fn from(error: RegionError) -> Self {
        Self::Region(error)
    }
}

/// Validation and comparison metrics for the two-body 3D pair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Solid3Metrics {
    /// Volume of the line body.
    pub line_volume: f64,
    /// Volume of the arc body.
    pub arc_volume: f64,
    /// Volume of the combined shell after removing the seam faces.
    pub combined_volume: f64,
    /// Reference `line_area * height`.
    pub expected_line_volume: f64,
    /// Reference `arc_area * height`.
    pub expected_arc_volume: f64,
    /// Reference `(line_area + arc_area) * height`.
    pub expected_combined_volume: f64,
    /// Number of seam quads (one orientation).
    pub seam_face_count: usize,
    /// Area of the seam surface.
    pub seam_area: f64,
    /// Reference seam area from the 2D seam length times height.
    pub expected_seam_area: f64,
}

/// Two neighbouring 3D bodies with a shared seam surface.
#[derive(Debug, Clone)]
pub struct SweepPair3 {
    /// Shared vertex pool.
    pub vertices: Vec<Point3>,
    /// Line-side body.
    pub line: Body3,
    /// Arc-side body.
    pub arc: Body3,
    /// Validation metrics.
    pub metrics: Solid3Metrics,
}

impl SweepPair3 {
    /// Faces that remain after deleting the internal seam faces.
    pub fn combined_body(&self) -> Body3 {
        let mut faces = Vec::with_capacity(self.line.faces.len() + self.arc.faces.len());
        faces.extend(
            self.line
                .faces
                .iter()
                .filter(|face| face.kind != FaceKind::Seam)
                .cloned(),
        );
        faces.extend(
            self.arc
                .faces
                .iter()
                .filter(|face| face.kind != FaceKind::Seam)
                .cloned(),
        );
        Body3 { faces }
    }

    /// Faces tagged as seam faces, in both orientations.
    pub fn seam_faces(&self) -> Vec<&Face3> {
        self.line
            .faces
            .iter()
            .chain(self.arc.faces.iter())
            .filter(|face| face.kind == FaceKind::Seam)
            .collect()
    }

    /// Re-run closure, seam-sharing and volume checks.
    pub fn validate(&self) -> Result<Solid3Metrics, Solid3Error> {
        check_closed(&self.line)?;
        check_closed(&self.arc)?;
        let combined = self.combined_body();
        check_closed(&combined)?;

        let line_seam: Vec<&Face3> = self
            .line
            .faces
            .iter()
            .filter(|face| face.kind == FaceKind::Seam)
            .collect();
        let arc_seam: Vec<&Face3> = self
            .arc
            .faces
            .iter()
            .filter(|face| face.kind == FaceKind::Seam)
            .collect();
        if line_seam.len() != arc_seam.len() || line_seam.is_empty() {
            return Err(Solid3Error::SeamMismatch);
        }
        let mut arc_sets: BTreeMap<Vec<usize>, Vec<&Face3>> = BTreeMap::new();
        for face in &arc_seam {
            let mut key = face.vertices.clone();
            key.sort_unstable();
            arc_sets.entry(key).or_default().push(face);
        }
        let mut seam_area = 0.0;
        for face in &line_seam {
            let mut key = face.vertices.clone();
            key.sort_unstable();
            let matches = arc_sets.get(&key).ok_or(Solid3Error::SeamMismatch)?;
            if matches.is_empty() {
                return Err(Solid3Error::SeamMismatch);
            }
            seam_area += face_area(&self.vertices, face);
        }

        let line_volume = body_volume(&self.vertices, &self.line);
        let arc_volume = body_volume(&self.vertices, &self.arc);
        let combined_volume = body_volume(&self.vertices, &combined);
        let scale = self
            .metrics
            .expected_combined_volume
            .abs()
            .max(self.metrics.expected_seam_area.abs())
            .max(1.0);
        if (line_volume - self.metrics.expected_line_volume).abs() > 1e-8 * scale
            || (arc_volume - self.metrics.expected_arc_volume).abs() > 1e-8 * scale
            || (combined_volume - self.metrics.expected_combined_volume).abs() > 1e-8 * scale
            || (seam_area - self.metrics.expected_seam_area).abs() > 1e-8 * scale
        {
            return Err(Solid3Error::MetricMismatch);
        }
        if (combined_volume - (line_volume + arc_volume)).abs() > 1e-8 * scale {
            return Err(Solid3Error::MetricMismatch);
        }

        Ok(Solid3Metrics {
            line_volume,
            arc_volume,
            combined_volume,
            expected_line_volume: self.metrics.expected_line_volume,
            expected_arc_volume: self.metrics.expected_arc_volume,
            expected_combined_volume: self.metrics.expected_combined_volume,
            seam_face_count: line_seam.len(),
            seam_area,
            expected_seam_area: self.metrics.expected_seam_area,
        })
    }
}

/// Parameters for the rectangular-profile 3D sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sweep3Params {
    /// Lower in-plane profile offset.
    pub d0: f64,
    /// Upper in-plane profile offset.
    pub d1: f64,
    /// Bottom height.
    pub z0: f64,
    /// Top height.
    pub z1: f64,
    /// Incoming line length (joint at x=0, start at x=-line_length).
    pub line_length: f64,
    /// Number of seam samples.
    pub seam_samples: usize,
    /// Number of arc samples per boundary.
    pub arc_steps: usize,
}

/// Build the two-body 3D sweep for a rectangular profile.
///
/// The profile is `offset in [d0, d1]` and `height in [z0, z1]`. The 2D
/// strategy must produce two parts sharing one seam; [`Strategy::HybridAtJunction`]
/// is the intended first case.
pub fn build_pair(
    model: LineArc,
    strategy: Strategy,
    params: Sweep3Params,
) -> Result<SweepPair3, Solid3Error> {
    if !params.d0.is_finite()
        || !params.d1.is_finite()
        || !params.z0.is_finite()
        || !params.z1.is_finite()
        || params.d0 >= params.d1
        || params.z0 >= params.z1
        || params.line_length <= 0.0
        || params.seam_samples < 2
        || params.arc_steps < 1
    {
        return Err(Solid3Error::InvalidProfile);
    }
    let built = build_regions(
        &model,
        strategy,
        params.d0,
        params.d1,
        params.line_length,
        params.seam_samples,
        params.arc_steps,
    )?;
    let height = params.z1 - params.z0;

    let mut mesh = MeshBuilder::new();
    let seam_segments = seam_segment_set(&built.seam);
    let line_polygon = ensure_ccw(built.line.points.clone());
    let arc_polygon = ensure_ccw(built.arc.points.clone());

    let line = extrude_body(
        &mut mesh,
        &line_polygon,
        params.z0,
        params.z1,
        &seam_segments,
    )?;
    let arc = extrude_body(
        &mut mesh,
        &arc_polygon,
        params.z0,
        params.z1,
        &seam_segments,
    )?;

    let seam_length = built
        .seam
        .windows(2)
        .map(|pair| pair[0].point.distance_to(pair[1].point))
        .sum::<f64>();
    let expected_line_volume = built.line.area() * height;
    let expected_arc_volume = built.arc.area() * height;
    let metrics = Solid3Metrics {
        line_volume: 0.0,
        arc_volume: 0.0,
        combined_volume: 0.0,
        expected_line_volume,
        expected_arc_volume,
        expected_combined_volume: expected_line_volume + expected_arc_volume,
        seam_face_count: 0,
        seam_area: 0.0,
        expected_seam_area: seam_length * height,
    };

    let pair = SweepPair3 {
        vertices: mesh.vertices,
        line,
        arc,
        metrics,
    };
    let metrics = pair.validate()?;
    Ok(SweepPair3 { metrics, ..pair })
}

struct MeshBuilder {
    vertices: Vec<Point3>,
    map: BTreeMap<(i64, i64, i64), usize>,
}

impl MeshBuilder {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            map: BTreeMap::new(),
        }
    }

    fn vertex(&mut self, point: Point2, z: f64) -> usize {
        let key = (q(point.x), q(point.y), q(z));
        if let Some(&index) = self.map.get(&key) {
            return index;
        }
        let index = self.vertices.len();
        self.vertices.push(Point3 {
            x: point.x,
            y: point.y,
            z,
        });
        self.map.insert(key, index);
        index
    }
}

fn extrude_body(
    mesh: &mut MeshBuilder,
    polygon: &[Point2],
    z0: f64,
    z1: f64,
    seam_segments: &SegmentSet,
) -> Result<Body3, Solid3Error> {
    if polygon.len() < 3 {
        return Err(Solid3Error::Degenerate);
    }
    let top: Vec<usize> = polygon
        .iter()
        .map(|point| mesh.vertex(*point, z1))
        .collect();
    let bottom: Vec<usize> = polygon
        .iter()
        .map(|point| mesh.vertex(*point, z0))
        .collect();
    let mut faces = Vec::with_capacity(polygon.len() + 2);
    faces.push(Face3 {
        vertices: top.clone(),
        kind: FaceKind::Top,
    });
    let mut reversed_bottom = bottom.clone();
    reversed_bottom.reverse();
    faces.push(Face3 {
        vertices: reversed_bottom,
        kind: FaceKind::Bottom,
    });

    for index in 0..polygon.len() {
        let next = (index + 1) % polygon.len();
        let a = polygon[index];
        let b = polygon[next];
        let kind = if seam_segments.contains(&segment_key(a, b)) {
            FaceKind::Seam
        } else {
            FaceKind::Side
        };
        let face = Face3 {
            vertices: vec![bottom[index], bottom[next], top[next], top[index]],
            kind,
        };
        faces.push(face);
    }
    Ok(Body3 { faces })
}

fn ensure_ccw(mut polygon: Vec<Point2>) -> Vec<Point2> {
    let area = signed_area(&polygon);
    if area < 0.0 {
        polygon.reverse();
    }
    polygon
}

fn signed_area(points: &[Point2]) -> f64 {
    let n = points.len();
    let mut twice = 0.0;
    for index in 0..n {
        let a = points[index];
        let b = points[(index + 1) % n];
        twice += a.x * b.y - b.x * a.y;
    }
    0.5 * twice
}

fn seam_segment_set(seam: &[crate::sweep::SeamSample]) -> SegmentSet {
    seam.windows(2)
        .map(|pair| segment_key(pair[0].point, pair[1].point))
        .collect()
}

fn segment_key(a: Point2, b: Point2) -> SegmentKey {
    let first = (q(a.x), q(a.y));
    let second = (q(b.x), q(b.y));
    if first <= second {
        (first, second)
    } else {
        (second, first)
    }
}

fn q(value: f64) -> i64 {
    (value * 1e12).round() as i64
}

fn check_closed(body: &Body3) -> Result<(), Solid3Error> {
    let mut edges: BTreeMap<(usize, usize), (usize, i32)> = BTreeMap::new();
    for face in &body.faces {
        if face.vertices.len() < 3 {
            return Err(Solid3Error::Degenerate);
        }
        for index in 0..face.vertices.len() {
            let a = face.vertices[index];
            let b = face.vertices[(index + 1) % face.vertices.len()];
            if a == b {
                return Err(Solid3Error::Degenerate);
            }
            let key = if a < b { (a, b) } else { (b, a) };
            let entry = edges.entry(key).or_insert((0, 0));
            entry.0 += 1;
            entry.1 += if a < b { 1 } else { -1 };
        }
    }
    for (_, (count, direction)) in edges {
        if count != 2 || direction != 0 {
            return Err(Solid3Error::NotClosed);
        }
    }
    Ok(())
}

fn body_volume(vertices: &[Point3], body: &Body3) -> f64 {
    let mut six_volume = 0.0;
    for face in &body.faces {
        if face.vertices.len() < 3 {
            continue;
        }
        let a = vertices[face.vertices[0]];
        for index in 1..face.vertices.len() - 1 {
            let b = vertices[face.vertices[index]];
            let c = vertices[face.vertices[index + 1]];
            six_volume += dot(a, cross(b, c));
        }
    }
    six_volume / 6.0
}

fn face_area(vertices: &[Point3], face: &Face3) -> f64 {
    if face.vertices.len() < 3 {
        return 0.0;
    }
    let a = vertices[face.vertices[0]];
    let mut sum = 0.0;
    for index in 1..face.vertices.len() - 1 {
        let b = vertices[face.vertices[index]];
        let c = vertices[face.vertices[index + 1]];
        sum += 0.5
            * cross(sub(b, a), sub(c, a)).distance_to(Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            });
    }
    sum
}

fn sub(a: Point3, b: Point3) -> Point3 {
    Point3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

fn cross(a: Point3, b: Point3) -> Point3 {
    Point3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

fn dot(a: Point3, b: Point3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sweep::build_regions;

    fn p(d0: f64, d1: f64, z0: f64, z1: f64, line_length: f64) -> Sweep3Params {
        Sweep3Params {
            d0,
            d1,
            z0,
            z1,
            line_length,
            seam_samples: 401,
            arc_steps: 96,
        }
    }

    fn edge_pairs(body: &Body3) -> BTreeMap<(usize, usize), (usize, i32)> {
        let mut edges = BTreeMap::new();
        for face in &body.faces {
            for index in 0..face.vertices.len() {
                let a = face.vertices[index];
                let b = face.vertices[(index + 1) % face.vertices.len()];
                let key = if a < b { (a, b) } else { (b, a) };
                let entry = edges.entry(key).or_insert((0, 0));
                entry.0 += 1;
                entry.1 += if a < b { 1 } else { -1 };
            }
        }
        edges
    }

    #[test]
    fn square_prism_is_closed() {
        let polygon = vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 1.0, y: 1.0 },
            Point2 { x: 0.0, y: 1.0 },
        ];
        let mut mesh = MeshBuilder::new();
        let body = extrude_body(&mut mesh, &polygon, -1.0, 1.0, &BTreeSet::new()).unwrap();
        let issues: Vec<_> = edge_pairs(&body)
            .into_iter()
            .filter(|(_, (count, direction))| *count != 2 || *direction != 0)
            .collect();
        assert!(issues.is_empty(), "issues={issues:?}");
        assert!(check_closed(&body).is_ok());
    }

    #[test]
    fn built_hybrid_prism_is_closed() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let built =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        let polygon = ensure_ccw(built.line.points.clone());
        let seam = seam_segment_set(&built.seam);
        let mut mesh = MeshBuilder::new();
        let body = extrude_body(&mut mesh, &polygon, -1.0, 1.0, &seam).unwrap();
        let issues: Vec<_> = edge_pairs(&body)
            .into_iter()
            .filter(|(_, (count, direction))| *count != 2 || *direction != 0)
            .collect();
        assert!(issues.is_empty(), "issues={issues:?}");
        assert!(check_closed(&body).is_ok());
    }

    #[test]
    fn built_hybrid_arc_prism_is_closed() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let built =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        let polygon = ensure_ccw(built.arc.points.clone());
        let seam = seam_segment_set(&built.seam);
        let mut mesh = MeshBuilder::new();
        let body = extrude_body(&mut mesh, &polygon, -1.0, 1.0, &seam).unwrap();
        let issues: Vec<_> = edge_pairs(&body)
            .into_iter()
            .filter(|(_, (count, direction))| *count != 2 || *direction != 0)
            .collect();
        assert!(issues.is_empty(), "issues={issues:?}");
        assert!(check_closed(&body).is_ok());
    }

    #[test]
    fn built_hybrid_combined_prism_is_closed() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let built =
            build_regions(&model, Strategy::HybridAtJunction, -2.0, 2.0, 20.0, 401, 96).unwrap();
        let seam = seam_segment_set(&built.seam);
        let mut mesh = MeshBuilder::new();
        let line = extrude_body(
            &mut mesh,
            &ensure_ccw(built.line.points.clone()),
            -1.0,
            1.0,
            &seam,
        )
        .unwrap();
        let arc = extrude_body(
            &mut mesh,
            &ensure_ccw(built.arc.points.clone()),
            -1.0,
            1.0,
            &seam,
        )
        .unwrap();
        let mut faces = Vec::new();
        faces.extend(
            line.faces
                .iter()
                .filter(|f| f.kind != FaceKind::Seam)
                .cloned(),
        );
        faces.extend(
            arc.faces
                .iter()
                .filter(|f| f.kind != FaceKind::Seam)
                .cloned(),
        );
        let combined = Body3 { faces };
        let issues: Vec<_> = edge_pairs(&combined)
            .into_iter()
            .filter(|(_, (count, direction))| *count != 2 || *direction != 0)
            .collect();
        assert!(issues.is_empty(), "issues={issues:?}");
        assert!(check_closed(&combined).is_ok());
    }
    #[test]
    fn hybrid_pair_150_metrics_match_2d_reference() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let pair = build_pair(
            model,
            Strategy::HybridAtJunction,
            p(-2.0, 2.0, -1.0, 1.0, 20.0),
        )
        .unwrap();
        let metrics = pair.validate().unwrap();
        assert!((metrics.line_volume - metrics.expected_line_volume).abs() < 1e-9);
        assert!((metrics.arc_volume - metrics.expected_arc_volume).abs() < 1e-9);
        assert!((metrics.combined_volume - metrics.expected_combined_volume).abs() < 1e-9);
        assert!((metrics.seam_area - metrics.expected_seam_area).abs() < 1e-9);
        assert_eq!(metrics.seam_face_count, 400);

        let combined = pair.combined_body();
        assert!(check_closed(&combined).is_ok());
        assert!(combined
            .faces
            .iter()
            .all(|face| face.kind != FaceKind::Seam));
        let line_seams = pair
            .line
            .faces
            .iter()
            .filter(|face| face.kind == FaceKind::Seam)
            .count();
        let arc_seams = pair
            .arc
            .faces
            .iter()
            .filter(|face| face.kind == FaceKind::Seam)
            .count();
        assert_eq!(line_seams, arc_seams);
        assert_eq!(line_seams, metrics.seam_face_count);
    }

    #[test]
    fn profile_height_scales_the_volume() {
        let model = LineArc::new(4.0, 90.0_f64.to_radians()).unwrap();
        let unit = build_pair(
            model,
            Strategy::HybridAtJunction,
            p(-2.0, 2.0, 0.0, 1.0, 20.0),
        )
        .unwrap();
        let triple = build_pair(
            model,
            Strategy::HybridAtJunction,
            p(-2.0, 2.0, 0.0, 3.0, 20.0),
        )
        .unwrap();
        assert!((triple.metrics.line_volume - 3.0 * unit.metrics.line_volume).abs() < 1e-9);
        assert!((triple.metrics.arc_volume - 3.0 * unit.metrics.arc_volume).abs() < 1e-9);
        assert!((triple.metrics.combined_volume - 3.0 * unit.metrics.combined_volume).abs() < 1e-9);
    }

    #[test]
    fn top_and_bottom_faces_are_at_profile_heights() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let pair = build_pair(
            model,
            Strategy::HybridAtJunction,
            p(-2.0, 2.0, -1.5, 2.5, 20.0),
        )
        .unwrap();
        for body in [&pair.line, &pair.arc] {
            for face in &body.faces {
                match face.kind {
                    FaceKind::Top => {
                        for &vertex in &face.vertices {
                            assert!((pair.vertices[vertex].z - 2.5).abs() < 1e-12);
                        }
                    }
                    FaceKind::Bottom => {
                        for &vertex in &face.vertices {
                            assert!((pair.vertices[vertex].z - (-1.5)).abs() < 1e-12);
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    #[test]
    fn invalid_profile_heights_are_rejected() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        assert!(matches!(
            build_pair(
                model,
                Strategy::HybridAtJunction,
                p(-2.0, 2.0, 1.0, 1.0, 20.0)
            ),
            Err(Solid3Error::InvalidProfile)
        ));
    }

    #[test]
    fn near_opposite_tangents_build_a_closed_pair() {
        for degrees in [170.0_f64, 175.0, 179.0] {
            let line_length = if degrees >= 175.0 { 500.0 } else { 100.0 };
            let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
            let pair = build_pair(
                model,
                Strategy::HybridAtJunction,
                p(-2.0, 2.0, -1.0, 1.0, line_length),
            )
            .unwrap_or_else(|error| panic!("{degrees}: {error:?}"));
            assert!(
                (pair.metrics.combined_volume - pair.metrics.expected_combined_volume).abs() < 1e-9
            );
            assert_eq!(pair.metrics.seam_face_count, 400);
        }
    }
}
