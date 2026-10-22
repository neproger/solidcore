//! Experimental 3D extrusion of the trimmed AOIM 2D regions.
//!
//! The first 3D step is deliberately narrow: a rectangular profile in the
//! `(offset, height)` plane, a planar Line–Arc path, and a fixed global `Z`
//! section direction. The resulting bodies are prisms over the already
//! validated 2D regions. The seam surface is built once from the 2D seam and
//! referenced by both bodies with opposite orientations; the combined shell is
//! obtained by removing the internal seam faces, not by a Boolean.

use crate::sweep::{build_regions, RegionError, Strategy};
use crate::{GeometryError, LineArc, Point2};
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
    /// Start cap of a general profile sweep.
    CapStart,
    /// End cap of a general profile sweep.
    CapEnd,
    /// Lateral surface of a general profile sweep.
    Lateral,
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
    /// A geometry primitive failed.
    Geometry(GeometryError),
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

impl From<GeometryError> for Solid3Error {
    fn from(error: GeometryError) -> Self {
        Self::Geometry(error)
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
        self.validate_with_tolerance(1e-8)
    }

    /// Validation with a caller-supplied relative tolerance.
    pub fn validate_with_tolerance(&self, tolerance: f64) -> Result<Solid3Metrics, Solid3Error> {
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
        if (line_volume - self.metrics.expected_line_volume).abs() > tolerance * scale
            || (arc_volume - self.metrics.expected_arc_volume).abs() > tolerance * scale
            || (combined_volume - self.metrics.expected_combined_volume).abs() > tolerance * scale
            || (seam_area - self.metrics.expected_seam_area).abs() > tolerance * scale
        {
            return Err(Solid3Error::MetricMismatch);
        }
        if (combined_volume - (line_volume + arc_volume)).abs() > tolerance * scale {
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
            Sweep3Params {
                d0: -2.0,
                d1: 2.0,
                z0: -1.0,
                z1: 1.0,
                line_length: 20.0,
                seam_samples: 401,
                arc_steps: 96,
            },
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

/// Independent volume reference for a general profile sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VolumeReference {
    /// Reference line-body volume from profile quadrature.
    pub line_volume: f64,
    /// Reference arc-body volume from profile quadrature.
    pub arc_volume: f64,
    /// `line_volume + arc_volume`.
    pub combined_volume: f64,
    /// Area of the profile polygon in `(d, z)`.
    pub profile_area: f64,
}

/// Result of the general simple-polygon sweep.
#[derive(Debug, Clone)]
pub struct ProfileSweep3 {
    /// Topology and mesh metrics.
    pub pair: SweepPair3,
    /// Independent volume reference.
    pub reference: VolumeReference,
}

/// Build a two-body 3D sweep for a simple closed profile polygon in `(d, z)`.
///
/// The profile must be simple, counter-clockwise (it is reversed if needed),
/// and must stay above `d = -R`. The seam is the hybrid 2D seam; the seam
/// surface is a faceted copy of the profile under `(d, z) -> (P(d), z)`.
pub fn build_profile_pair(
    model: LineArc,
    profile: &[Point2],
    line_length: f64,
    profile_steps: usize,
    arc_steps: usize,
) -> Result<ProfileSweep3, Solid3Error> {
    if profile.len() < 3
        || !line_length.is_finite()
        || line_length <= 0.0
        || profile_steps < 1
        || arc_steps < 1
    {
        return Err(Solid3Error::InvalidProfile);
    }
    for point in profile {
        if !point.x.is_finite() || !point.y.is_finite() || point.x <= -model.radius {
            return Err(Solid3Error::InvalidProfile);
        }
    }
    if !is_simple(profile) {
        return Err(Solid3Error::InvalidProfile);
    }
    let mut base = profile.to_vec();
    if signed_area(&base) < 0.0 {
        base.reverse();
    }
    let sampled = sample_profile(&base, profile_steps);
    let corner_triangles = ear_clip(&base)?;
    let (profile_points, triangles) =
        refine_triangles(&base, &sampled, profile_steps, &corner_triangles)?;
    let profile_area = signed_area(&sampled).abs();
    if profile_area <= 0.0 {
        return Err(Solid3Error::InvalidProfile);
    }

    let mut mesh = MeshBuilder::new();
    let mut start = Vec::with_capacity(profile_points.len());
    let mut end = Vec::with_capacity(profile_points.len());
    let mut seam = Vec::with_capacity(profile_points.len());
    for point in &profile_points {
        let d = point.x;
        let z = point.y;
        let seam2 = crate::sweep::seam_point(&model, Strategy::HybridAtJunction, d)
            .map_err(Solid3Error::from)?;
        start.push(mesh.vertex(
            Point2 {
                x: -line_length,
                y: -d,
            },
            z,
        ));
        end.push(mesh.vertex(
            Point2 {
                x: model.radius * model.corner_angle_rad.sin(),
                y: model.radius * model.corner_angle_rad.cos() - model.radius - d,
            },
            z,
        ));
        seam.push(mesh.vertex(seam2, z));
    }

    let mut line_faces = Vec::new();
    let mut arc_faces = Vec::new();
    for triangle in &triangles {
        let [a, b, c] = *triangle;
        line_faces.push(Face3 {
            vertices: vec![seam[c], seam[b], seam[a]],
            kind: FaceKind::Seam,
        });
        arc_faces.push(Face3 {
            vertices: vec![seam[a], seam[b], seam[c]],
            kind: FaceKind::Seam,
        });
        line_faces.push(Face3 {
            vertices: vec![start[a], start[b], start[c]],
            kind: FaceKind::CapStart,
        });
        arc_faces.push(Face3 {
            vertices: vec![end[c], end[b], end[a]],
            kind: FaceKind::CapEnd,
        });
    }
    for index in 0..sampled.len() {
        let next = (index + 1) % sampled.len();
        let a = start[index];
        let b = start[next];
        let sa = seam[index];
        let sb = seam[next];
        line_faces.push(quad_face(a, sa, sb, b, FaceKind::Lateral));
        let _ = (a, b, sa, sb);
    }

    // Arc lateral patches: one sampled strip per profile edge.
    for index in 0..sampled.len() {
        let next = (index + 1) % sampled.len();
        let seam_a = sampled[index];
        let seam_b = sampled[next];
        for step in 0..arc_steps {
            let t0 = step as f64 / arc_steps as f64;
            let t1 = (step + 1) as f64 / arc_steps as f64;
            let a0 = if step == 0 {
                seam[index]
            } else {
                let point = hybrid_arc_point(&model, seam_a.x, t0)?;
                mesh.vertex(point, seam_a.y)
            };
            let a1 = if step + 1 == arc_steps {
                end[index]
            } else {
                let point = hybrid_arc_point(&model, seam_a.x, t1)?;
                mesh.vertex(point, seam_a.y)
            };
            let b0 = if step == 0 {
                seam[next]
            } else {
                let point = hybrid_arc_point(&model, seam_b.x, t0)?;
                mesh.vertex(point, seam_b.y)
            };
            let b1 = if step + 1 == arc_steps {
                end[next]
            } else {
                let point = hybrid_arc_point(&model, seam_b.x, t1)?;
                mesh.vertex(point, seam_b.y)
            };
            arc_faces.push(triangle_face(a0, a1, b1, FaceKind::Lateral));
            arc_faces.push(triangle_face(a0, b1, b0, FaceKind::Lateral));
        }
    }

    let line = Body3 { faces: line_faces };
    let arc = Body3 { faces: arc_faces };
    let reference = profile_reference(
        &model,
        &profile_points,
        &triangles,
        line_length,
        profile_area,
    )?;
    let seam_area = body_seam_area(&mesh.vertices, &line);
    let metrics = Solid3Metrics {
        line_volume: 0.0,
        arc_volume: 0.0,
        combined_volume: 0.0,
        expected_line_volume: reference.line_volume,
        expected_arc_volume: reference.arc_volume,
        expected_combined_volume: reference.combined_volume,
        seam_face_count: 0,
        seam_area: 0.0,
        expected_seam_area: seam_area,
    };
    let pair = SweepPair3 {
        vertices: mesh.vertices,
        line,
        arc,
        metrics,
    };
    let metrics = pair.validate_with_tolerance(1e9)?;
    Ok(ProfileSweep3 {
        pair: SweepPair3 { metrics, ..pair },
        reference,
    })
}

fn quad_face(a: usize, b: usize, c: usize, d: usize, kind: FaceKind) -> Face3 {
    Face3 {
        vertices: vec![a, b, c, d],
        kind,
    }
}

fn triangle_face(a: usize, b: usize, c: usize, kind: FaceKind) -> Face3 {
    Face3 {
        vertices: vec![a, b, c],
        kind,
    }
}

fn sample_profile(profile: &[Point2], steps: usize) -> Vec<Point2> {
    let mut sampled = Vec::with_capacity(profile.len() * steps);
    for index in 0..profile.len() {
        let a = profile[index];
        let b = profile[(index + 1) % profile.len()];
        for step in 0..steps {
            let t = step as f64 / steps as f64;
            sampled.push(Point2 {
                x: a.x + (b.x - a.x) * t,
                y: a.y + (b.y - a.y) * t,
            });
        }
    }
    sampled
}

fn ear_clip(polygon: &[Point2]) -> Result<Vec<[usize; 3]>, Solid3Error> {
    if polygon.len() < 3 {
        return Err(Solid3Error::InvalidProfile);
    }
    let mut indices: Vec<usize> = (0..polygon.len()).collect();
    let mut triangles = Vec::new();
    let mut guard = 0;
    while indices.len() > 3 && guard < polygon.len() * polygon.len() {
        guard += 1;
        let count = indices.len();
        let mut clipped = false;
        for position in 0..count {
            let previous = indices[(position + count - 1) % count];
            let current = indices[position];
            let next = indices[(position + 1) % count];
            let cross = orient2(polygon[previous], polygon[current], polygon[next]);
            if cross < -1e-12 {
                continue;
            }
            let mut blocked = false;
            for &other in &indices {
                if other == previous || other == current || other == next {
                    continue;
                }
                if point_in_triangle(
                    polygon[other],
                    polygon[previous],
                    polygon[current],
                    polygon[next],
                ) {
                    blocked = true;
                    break;
                }
            }
            if blocked {
                continue;
            }
            triangles.push([previous, current, next]);
            indices.remove(position);
            clipped = true;
            break;
        }
        if !clipped {
            return Err(Solid3Error::InvalidProfile);
        }
    }
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }
    Ok(triangles)
}

fn point_in_triangle(p: Point2, a: Point2, b: Point2, c: Point2) -> bool {
    let d1 = orient2(a, b, p);
    let d2 = orient2(b, c, p);
    let d3 = orient2(c, a, p);
    (d1 > 1e-12 && d2 > 1e-12 && d3 > 1e-12) || (d1 < -1e-12 && d2 < -1e-12 && d3 < -1e-12)
}

fn orient2(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn is_simple(profile: &[Point2]) -> bool {
    let n = profile.len();
    for first in 0..n {
        let a = profile[first];
        let b = profile[(first + 1) % n];
        for second in (first + 1)..n {
            if second == first + 1 || (first == 0 && second == n - 1) {
                continue;
            }
            let c = profile[second];
            let d = profile[(second + 1) % n];
            if segments_intersect(a, b, c, d) {
                return false;
            }
        }
    }
    true
}

fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let o1 = orient2(a, b, c);
    let o2 = orient2(a, b, d);
    let o3 = orient2(c, d, a);
    let o4 = orient2(c, d, b);
    o1 * o2 < 0.0 && o3 * o4 < 0.0
}

fn hybrid_arc_length(model: &LineArc, d: f64) -> Result<f64, Solid3Error> {
    let seam = crate::sweep::seam_point(model, Strategy::HybridAtJunction, d)
        .map_err(Solid3Error::from)?;
    if d < 0.0 {
        let joint = model.joint_arc_point(d)?;
        let tangent = seam.distance_to(joint);
        Ok(tangent + (model.radius + d) * model.corner_angle_rad)
    } else {
        let parameter = model.arc_parameter(seam);
        Ok((model.radius + d) * (model.corner_angle_rad - parameter))
    }
}

fn hybrid_arc_point(model: &LineArc, d: f64, t: f64) -> Result<Point2, Solid3Error> {
    let seam = crate::sweep::seam_point(model, Strategy::HybridAtJunction, d)
        .map_err(Solid3Error::from)?;
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();
    let t = t.clamp(0.0, 1.0);
    if d >= 0.0 {
        let parameter = model.arc_parameter(seam);
        let angle = start + parameter + (end - start - parameter) * t;
        return Ok(Point2 {
            x: model.center().x + (model.radius + d) * angle.cos(),
            y: model.center().y + (model.radius + d) * angle.sin(),
        });
    }
    let joint = model.joint_arc_point(d)?;
    let tangent = seam.distance_to(joint);
    let arc = (model.radius + d) * model.corner_angle_rad;
    let total = tangent + arc;
    if total <= 0.0 {
        return Err(Solid3Error::InvalidProfile);
    }
    let distance = t * total;
    if tangent > 0.0 && distance <= tangent {
        let ratio = distance / tangent;
        return Ok(Point2 {
            x: seam.x + (joint.x - seam.x) * ratio,
            y: seam.y + (joint.y - seam.y) * ratio,
        });
    }
    let angle = start + (distance - tangent) / (model.radius + d);
    Ok(Point2 {
        x: model.center().x + (model.radius + d) * angle.cos(),
        y: model.center().y + (model.radius + d) * angle.sin(),
    })
}

fn body_seam_area(vertices: &[Point3], body: &Body3) -> f64 {
    body.faces
        .iter()
        .filter(|face| face.kind == FaceKind::Seam)
        .map(|face| face_area(vertices, face))
        .sum()
}

fn profile_reference(
    model: &LineArc,
    sampled: &[Point2],
    triangles: &[[usize; 3]],
    line_length: f64,
    profile_area: f64,
) -> Result<VolumeReference, Solid3Error> {
    let weights = [
        (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0, 0.225),
        (
            0.059_715_871_7,
            0.470_142_064_1,
            0.470_142_064_1,
            0.132_394_152_7,
        ),
        (
            0.470_142_064_1,
            0.059_715_871_7,
            0.470_142_064_1,
            0.132_394_152_7,
        ),
        (
            0.470_142_064_1,
            0.470_142_064_1,
            0.059_715_871_7,
            0.132_394_152_7,
        ),
        (
            0.797_426_985_3,
            0.101_286_507_3,
            0.101_286_507_3,
            0.125_939_180_5,
        ),
        (
            0.101_286_507_3,
            0.797_426_985_3,
            0.101_286_507_3,
            0.125_939_180_5,
        ),
        (
            0.101_286_507_3,
            0.101_286_507_3,
            0.797_426_985_3,
            0.125_939_180_5,
        ),
    ];
    let mut line_volume = 0.0;
    let mut arc_volume = 0.0;
    for triangle in triangles {
        let a = sampled[triangle[0]];
        let b = sampled[triangle[1]];
        let c = sampled[triangle[2]];
        let area = 0.5 * (orient2(a, b, c)).abs();
        for (l1, l2, l3, weight) in weights {
            let d = a.x * l1 + b.x * l2 + c.x * l3;
            let seam = crate::sweep::seam_point(model, Strategy::HybridAtJunction, d)
                .map_err(Solid3Error::from)?;
            let line = seam.x + line_length;
            let arc = hybrid_arc_length(model, d)?;
            line_volume += area * weight * line;
            arc_volume += area * weight * arc;
        }
    }
    Ok(VolumeReference {
        line_volume,
        arc_volume,
        combined_volume: line_volume + arc_volume,
        profile_area,
    })
}

fn refine_triangles(
    base: &[Point2],
    sampled: &[Point2],
    steps: usize,
    corner_triangles: &[[usize; 3]],
) -> Result<(Vec<Point2>, Vec<[usize; 3]>), Solid3Error> {
    let n = base.len();
    let sample_count = sampled.len();
    let mut points = sampled.to_vec();
    let mut triangles = Vec::new();
    for triangle in corner_triangles {
        let [a, b, c] = *triangle;
        let centroid = Point2 {
            x: (base[a].x + base[b].x + base[c].x) / 3.0,
            y: (base[a].y + base[b].y + base[c].y) / 3.0,
        };
        let centroid_index = points.len();
        points.push(centroid);

        let chain = |from: usize, to: usize| -> Vec<usize> {
            if to == (from + 1) % n {
                (0..=steps)
                    .map(|k| (from * steps + k) % sample_count)
                    .collect()
            } else if from == (to + 1) % n {
                let mut backward: Vec<usize> = (0..=steps)
                    .map(|k| (to * steps + k) % sample_count)
                    .collect();
                backward.reverse();
                backward
            } else {
                vec![from * steps, to * steps]
            }
        };

        let mut boundary: Vec<usize> = Vec::new();
        for (from, to) in [(a, b), (b, c), (c, a)] {
            for index in chain(from, to) {
                if boundary.last() != Some(&index) {
                    boundary.push(index);
                }
            }
        }
        if boundary.len() > 1 && boundary.first() == boundary.last() {
            boundary.pop();
        }
        if boundary.len() < 3 {
            return Err(Solid3Error::InvalidProfile);
        }
        for index in 0..boundary.len() {
            let first = boundary[index];
            let second = boundary[(index + 1) % boundary.len()];
            if first == second {
                continue;
            }
            triangles.push([centroid_index, first, second]);
        }
    }
    Ok((points, triangles))
}

#[cfg(test)]
mod profile_tests {
    use super::*;

    fn p(d: f64, z: f64) -> Point2 {
        Point2 { x: d, y: z }
    }

    fn rectangle() -> Vec<Point2> {
        vec![p(-2.0, -1.0), p(2.0, -1.0), p(2.0, 1.0), p(-2.0, 1.0)]
    }

    fn l_shape() -> Vec<Point2> {
        vec![
            p(-2.0, -1.0),
            p(0.0, -1.0),
            p(0.0, 0.0),
            p(1.0, 0.0),
            p(1.0, 1.0),
            p(-2.0, 1.0),
        ]
    }

    fn t_shape() -> Vec<Point2> {
        vec![
            p(-1.0, -1.0),
            p(1.0, -1.0),
            p(1.0, 0.0),
            p(2.0, 0.0),
            p(2.0, 1.0),
            p(-2.0, 1.0),
            p(-2.0, 0.0),
            p(-1.0, 0.0),
        ]
    }

    #[test]
    fn profile_sweeps_are_closed_and_volumes_match_the_reference() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        for (name, profile, expected_area) in [
            ("rectangle", rectangle(), 8.0),
            ("L", l_shape(), 5.0),
            ("T", t_shape(), 6.0),
        ] {
            let sweep = build_profile_pair(model, &profile, 20.0, 8, 32)
                .unwrap_or_else(|error| panic!("{name}: {error:?}"));
            assert!(
                (sweep.reference.profile_area - expected_area).abs() < 1e-9,
                "{name}: area={}",
                sweep.reference.profile_area
            );
            let scale = sweep.reference.combined_volume.abs().max(1.0);
            assert!(
                (sweep.pair.metrics.combined_volume - sweep.reference.combined_volume).abs()
                    < 5e-3 * scale,
                "{name}: mesh={} ref={}",
                sweep.pair.metrics.combined_volume,
                sweep.reference.combined_volume
            );
            // Individual split volumes include the faceted seam disk; the
            // combined volume is the independent check.
            assert!(
                (sweep.pair.metrics.line_volume - sweep.reference.line_volume).abs() < 0.05 * scale
            );
            assert!(
                (sweep.pair.metrics.arc_volume - sweep.reference.arc_volume).abs() < 0.05 * scale
            );
            assert!(sweep.pair.metrics.seam_face_count > 0);
            let combined = sweep.pair.combined_body();
            assert!(check_closed(&combined).is_ok(), "{name}");
        }
    }

    #[test]
    fn general_rectangle_matches_the_rectangular_builder() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let general = build_profile_pair(model, &rectangle(), 20.0, 8, 32).unwrap();
        let rectangular = build_pair(
            model,
            Strategy::HybridAtJunction,
            Sweep3Params {
                d0: -2.0,
                d1: 2.0,
                z0: -1.0,
                z1: 1.0,
                line_length: 20.0,
                seam_samples: 401,
                arc_steps: 96,
            },
        )
        .unwrap();
        let scale = rectangular.metrics.combined_volume.max(1.0);
        assert!(
            (general.pair.metrics.combined_volume - rectangular.metrics.combined_volume).abs()
                < 5e-3 * scale
        );
    }

    #[test]
    fn reversed_profile_is_accepted() {
        let model = LineArc::new(4.0, 90.0_f64.to_radians()).unwrap();
        let mut reversed = l_shape();
        reversed.reverse();
        let sweep = build_profile_pair(model, &reversed, 20.0, 8, 32).unwrap();
        assert!((sweep.reference.profile_area - 5.0).abs() < 1e-9);
    }

    #[test]
    fn invalid_profiles_are_rejected() {
        let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
        let bowtie = vec![p(-2.0, -1.0), p(2.0, 1.0), p(2.0, -1.0), p(-2.0, 1.0)];
        assert!(matches!(
            build_profile_pair(model, &bowtie, 20.0, 8, 32),
            Err(Solid3Error::InvalidProfile)
        ));
        let collapsed = vec![p(-4.0, -1.0), p(0.0, -1.0), p(0.0, 1.0), p(-4.0, 1.0)];
        assert!(matches!(
            build_profile_pair(model, &collapsed, 20.0, 8, 32),
            Err(Solid3Error::InvalidProfile)
        ));
    }
}
