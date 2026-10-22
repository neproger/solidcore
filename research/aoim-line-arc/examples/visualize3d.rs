//! Isometric SVG views of the two-body 3D AOIM sweep.
//!
//! Run from `research/aoim-line-arc`:
//!
//! ```sh
//! cargo run --example visualize3d
//! ```

use aoim_line_arc_research::sweep::Strategy;
use aoim_line_arc_research::sweep3d::{
    build_pair, Face3, FaceKind, Point3, Sweep3Params, SweepPair3,
};
use aoim_line_arc_research::LineArc;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

const WIDTH: f64 = 1000.0;
const HEIGHT: f64 = 920.0;
const PANEL_HEIGHT: f64 = 430.0;
const MARGIN: f64 = 55.0;

#[derive(Clone, Copy)]
struct View3 {
    scale: f64,
    left: f64,
    top: f64,
}

impl View3 {
    fn new(points: &[Point3], top_offset: f64) -> Self {
        let projected: Vec<(f64, f64)> = points.iter().map(|point| project(*point)).collect();
        let min_x = projected.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_x = projected
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_y = projected.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let max_y = projected
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max);
        let width = (WIDTH - 2.0 * MARGIN).max(1.0);
        let height = (PANEL_HEIGHT - 2.0 * MARGIN).max(1.0);
        let scale = (width / (max_x - min_x).max(1e-9)).min(height / (max_y - min_y).max(1e-9));
        let left = (WIDTH - (max_x - min_x) * scale) / 2.0 - min_x * scale;
        let top = top_offset + (PANEL_HEIGHT - (max_y - min_y) * scale) / 2.0 + max_y * scale;
        Self { scale, left, top }
    }

    fn map(&self, point: Point3) -> (f64, f64) {
        let (x, y) = project(point);
        (self.left + x * self.scale, self.top - y * self.scale)
    }
}

fn project(point: Point3) -> (f64, f64) {
    let c = 0.866_025_403_784_438_6_f64;
    let s = 0.5;
    ((point.x - point.y) * c, (point.x + point.y) * s - point.z)
}

fn polygon_coords(view: &View3, points: &[Point3], ids: &[usize]) -> String {
    ids.iter()
        .map(|&id| {
            let (x, y) = view.map(points[id]);
            format!("{x:.2},{y:.2}")
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn face_svg(
    view: &View3,
    points: &[Point3],
    face: &Face3,
    fill: &str,
    fill_opacity: f64,
    stroke: &str,
    stroke_width: f64,
) -> String {
    let coords = polygon_coords(view, points, &face.vertices);
    format!(
        "<polygon points=\"{coords}\" fill=\"{fill}\" fill-opacity=\"{fill_opacity:.2}\" \
         stroke=\"{stroke}\" stroke-width=\"{stroke_width:.2}\" stroke-linejoin=\"round\"/>\n"
    )
}

fn draw_bodies(canvas: &mut String, view: &View3, pair: &SweepPair3) {
    for face in &pair.line.faces {
        let color = if face.kind == FaceKind::Seam {
            "#7b4fb3"
        } else {
            "#2f6fbf"
        };
        let opacity = if face.kind == FaceKind::Seam {
            0.42
        } else {
            0.10
        };
        let _ = write!(
            canvas,
            "{}",
            face_svg(view, &pair.vertices, face, color, opacity, color, 0.8)
        );
    }
    for face in &pair.arc.faces {
        let color = if face.kind == FaceKind::Seam {
            "#7b4fb3"
        } else {
            "#d9822b"
        };
        let opacity = if face.kind == FaceKind::Seam {
            0.42
        } else {
            0.10
        };
        let _ = write!(
            canvas,
            "{}",
            face_svg(view, &pair.vertices, face, color, opacity, color, 0.8)
        );
    }
}

fn draw_edge_loops(canvas: &mut String, view: &View3, pair: &SweepPair3) {
    for face in pair.seam_faces() {
        let coords = polygon_coords(view, &pair.vertices, &face.vertices);
        let _ = writeln!(
            canvas,
            "<polyline points=\"{coords}\" fill=\"none\" stroke=\"#5b2d91\" \
             stroke-width=\"1.8\" stroke-linejoin=\"round\"/>"
        );
    }
}

fn draw_seam_only(canvas: &mut String, view: &View3, pair: &SweepPair3) {
    for face in pair.seam_faces() {
        let _ = write!(
            canvas,
            "{}",
            face_svg(view, &pair.vertices, face, "#9b7bc9", 0.50, "#5b2d91", 1.2)
        );
    }
}

fn render(pair: &SweepPair3, degrees: f64) -> String {
    let mut canvas = String::new();
    let title = format!("AOIM 3D sweep, Line-Arc {degrees:.0} deg, rectangular profile");
    let panel1 = 30.0;
    let panel2 = 30.0 + PANEL_HEIGHT + 40.0;
    let view1 = View3::new(&pair.vertices, panel1);
    draw_bodies(&mut canvas, &view1, pair);
    draw_edge_loops(&mut canvas, &view1, pair);
    let _ = writeln!(
        canvas,
        "<text x=\"18\" y=\"{}\" font-family=\"ui-monospace, Consolas, monospace\" \
         font-size=\"15\" fill=\"#111\">{title}</text>",
        panel1 - 12.0
    );
    let _ = writeln!(
        canvas,
        "<text x=\"18\" y=\"{}\" font-family=\"ui-monospace, Consolas, monospace\" \
         font-size=\"11\" fill=\"#333\">blue = line body, orange = arc body, purple = shared seam surface</text>",
        panel1 + 10.0
    );
    let metrics = pair.metrics;
    let _ = writeln!(
        canvas,
        "<text x=\"18\" y=\"{}\" font-family=\"ui-monospace, Consolas, monospace\" \
         font-size=\"11\" fill=\"#333\">volumes: line={:.3}, arc={:.3}, combined={:.3}; \
         seam faces={}, seam area={:.3}</text>",
        panel1 + 28.0,
        metrics.line_volume,
        metrics.arc_volume,
        metrics.combined_volume,
        metrics.seam_face_count,
        metrics.seam_area
    );

    // Second panel: the seam surface alone, same projection basis.
    let seam_points: Vec<Point3> = pair
        .seam_faces()
        .iter()
        .flat_map(|face| face.vertices.iter().map(|&id| pair.vertices[id]))
        .collect();
    let view2 = View3::new(&seam_points, panel2);
    draw_seam_only(&mut canvas, &view2, pair);
    let _ = writeln!(
        canvas,
        "<text x=\"18\" y=\"{}\" font-family=\"ui-monospace, Consolas, monospace\" \
         font-size=\"15\" fill=\"#111\">Shared seam surface only</text>",
        panel2 - 12.0
    );
    let _ = writeln!(
        canvas,
        "<text x=\"18\" y=\"{}\" font-family=\"ui-monospace, Consolas, monospace\" \
         font-size=\"11\" fill=\"#333\">same vertices and faces are used by both bodies; \
         combined shell removes them</text>",
        panel2 + 10.0
    );

    format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{WIDTH}\" height=\"{HEIGHT}\" \
         viewBox=\"0 0 {WIDTH} {HEIGHT}\">\n\
         <rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n\
         <line x1=\"20\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#ddd\"/>\n\
         {canvas}\n</svg>\n",
        panel2 - 20.0,
        WIDTH - 20.0,
        panel2 - 20.0
    )
}

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

fn main() {
    let out_dir = std::env::args().nth(1).unwrap_or_else(|| "out".to_owned());
    if let Err(error) = fs::create_dir_all(&out_dir) {
        eprintln!("cannot create {out_dir}: {error}");
        std::process::exit(1);
    }
    let mut links = String::new();
    for (degrees, line_length) in [(90.0_f64, 20.0_f64), (150.0, 20.0)] {
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        let pair = match build_pair(
            model,
            Strategy::HybridAtJunction,
            p(-2.0, 2.0, -1.0, 1.0, line_length),
        ) {
            Ok(pair) => pair,
            Err(error) => {
                eprintln!("{degrees}: {error:?}");
                std::process::exit(1);
            }
        };
        let file_name = format!("aoim-3d-{degrees:.0}.svg");
        let path = Path::new(&out_dir).join(&file_name);
        if let Err(error) = fs::write(&path, render(&pair, degrees)) {
            eprintln!("cannot write {}: {error}", path.display());
            std::process::exit(1);
        }
        let _ = writeln!(
            links,
            "<li><a href=\"{file_name}\">{degrees:.0} deg 3D</a> \
             <img src=\"{file_name}\" alt=\"AOIM 3D {degrees:.0} deg\"></li>"
        );
    }
    let index = format!(
        "<!doctype html><meta charset=\"utf-8\"><title>AOIM 3D</title>\
         <style>body{{font-family:system-ui;margin:2rem;background:#fafafa}}\
         li{{list-style:none;margin:2rem 0}}img{{max-width:1000px;width:100%;\
         border:1px solid #ddd;background:#fff}}</style>\
         <h1>AOIM 3D sweep</h1>\
         <p>Two neighbouring 3D bodies with a shared seam surface, built without Boolean.</p>\
         <ul>{links}</ul>"
    );
    let index_path = Path::new(&out_dir).join("3d.html");
    if let Err(error) = fs::write(&index_path, index) {
        eprintln!("cannot write {}: {error}", index_path.display());
        std::process::exit(1);
    }
    println!("wrote {}", index_path.display());
}
