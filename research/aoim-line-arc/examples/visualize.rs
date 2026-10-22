//! Render the current AOIM Line–Arc findings to SVG.
//!
//! Run from `research/aoim-line-arc`:
//!
//! ```sh
//! cargo run --example visualize
//! ```
//!
//! The example writes `out/aoim-line-arc-{30,90,150}.svg` and `out/index.html`.
//! It uses only the research crate and `std`; it never touches SolidCore.

use aoim_line_arc_research::adaptive::{AdaptiveParams, AdaptivePath, AdaptiveRegions};
use aoim_line_arc_research::sweep::{
    build_regions, build_untrimmed, BuiltRegions, RegionError, Strategy,
};
use aoim_line_arc_research::{LineArc, Point2};
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

const RADIUS: f64 = 4.0;
const PROFILE_MIN: f64 = -0.5 * RADIUS;
const PROFILE_MAX: f64 = 0.5 * RADIUS;
const LINE_LENGTH: f64 = 2.5 * RADIUS;

const WIDTH: f64 = 1000.0;
const HEIGHT: f64 = 1120.0;
const PANEL_HEIGHT: f64 = 500.0;
const PANEL_GAP: f64 = 40.0;
const SECOND_PANEL_TOP: f64 = 30.0 + PANEL_HEIGHT + PANEL_GAP;
const MARGIN: f64 = 45.0;

const LINE_COLOR: &str = "#2f6fbf";
const ARC_COLOR: &str = "#d9822b";
const FINITE_COLOR: &str = "#1f8a4c";
const EXTENSION_COLOR: &str = "#e08a2e";
const LIMIT_COLOR: &str = "#c0392b";
const HYBRID_COLOR: &str = "#7b4fb3";
const EVENT_COLOR: &str = "#0f7c8c";

#[derive(Clone, Copy)]
struct Bounds {
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}

impl Bounds {
    fn new(min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> Self {
        Self {
            min_x,
            max_x,
            min_y,
            max_y,
        }
    }

    fn include(&mut self, p: Point2) {
        self.min_x = self.min_x.min(p.x);
        self.max_x = self.max_x.max(p.x);
        self.min_y = self.min_y.min(p.y);
        self.max_y = self.max_y.max(p.y);
    }

    fn include_points(&mut self, points: &[Point2]) {
        for &p in points {
            self.include(p);
        }
    }

    fn padded(self, fraction: f64) -> Self {
        let span = (self.max_x - self.min_x).max(self.max_y - self.min_y);
        let pad = fraction * span;
        Self {
            min_x: self.min_x - pad,
            max_x: self.max_x + pad,
            min_y: self.min_y - pad,
            max_y: self.max_y + pad,
        }
    }
}

struct View {
    bounds: Bounds,
    scale: f64,
    left: f64,
    top: f64,
}

impl View {
    fn new(bounds: Bounds, top_offset: f64) -> Self {
        let bounds = bounds.padded(0.04);
        let width = WIDTH - 2.0 * MARGIN;
        let height = PANEL_HEIGHT - 2.0 * MARGIN;
        let data_width = bounds.max_x - bounds.min_x;
        let data_height = bounds.max_y - bounds.min_y;
        let scale = (width / data_width).min(height / data_height);
        let left = (WIDTH - data_width * scale) / 2.0;
        let top = top_offset + (PANEL_HEIGHT - data_height * scale) / 2.0;
        Self {
            bounds,
            scale,
            left,
            top,
        }
    }

    fn map(&self, p: Point2) -> (f64, f64) {
        (
            self.left + (p.x - self.bounds.min_x) * self.scale,
            self.top + (self.bounds.max_y - p.y) * self.scale,
        )
    }
}

struct Canvas {
    elements: String,
}

impl Canvas {
    fn new() -> Self {
        Self {
            elements: String::new(),
        }
    }

    fn line(&mut self, view: &View, a: Point2, b: Point2, stroke: &str, width: f64, dash: &str) {
        let (x1, y1) = view.map(a);
        let (x2, y2) = view.map(b);
        let dash = if dash.is_empty() {
            String::new()
        } else {
            format!(" stroke-dasharray=\"{dash}\"")
        };
        let _ = writeln!(
            self.elements,
            "<line x1=\"{x1:.2}\" y1=\"{y1:.2}\" x2=\"{x2:.2}\" y2=\"{y2:.2}\" \
             stroke=\"{stroke}\" stroke-width=\"{width:.2}\" stroke-linecap=\"round\"{dash}/>"
        );
    }

    fn screen_line(&mut self, a: (f64, f64), b: (f64, f64), stroke: &str, width: f64, dash: &str) {
        let (x1, y1) = a;
        let (x2, y2) = b;
        let dash = if dash.is_empty() {
            String::new()
        } else {
            format!(" stroke-dasharray=\"{dash}\"")
        };
        let _ = writeln!(
            self.elements,
            "<line x1=\"{x1:.2}\" y1=\"{y1:.2}\" x2=\"{x2:.2}\" y2=\"{y2:.2}\" \
             stroke=\"{stroke}\" stroke-width=\"{width:.2}\" stroke-linecap=\"round\"{dash}/>"
        );
    }

    fn polyline(&mut self, view: &View, points: &[Point2], stroke: &str, width: f64, dash: &str) {
        if points.len() < 2 {
            return;
        }
        let coords = points
            .iter()
            .map(|p| {
                let (x, y) = view.map(*p);
                format!("{x:.2},{y:.2}")
            })
            .collect::<Vec<_>>()
            .join(" ");
        let dash = if dash.is_empty() {
            String::new()
        } else {
            format!(" stroke-dasharray=\"{dash}\"")
        };
        let _ = writeln!(
            self.elements,
            "<polyline points=\"{coords}\" fill=\"none\" stroke=\"{stroke}\" \
             stroke-width=\"{width:.2}\" stroke-linecap=\"round\" \
             stroke-linejoin=\"round\"{dash}/>"
        );
    }

    fn polygon(
        &mut self,
        view: &View,
        points: &[Point2],
        fill: &str,
        fill_opacity: f64,
        stroke: &str,
        width: f64,
    ) {
        if points.len() < 3 {
            return;
        }
        let coords = points
            .iter()
            .map(|p| {
                let (x, y) = view.map(*p);
                format!("{x:.2},{y:.2}")
            })
            .collect::<Vec<_>>()
            .join(" ");
        let _ = writeln!(
            self.elements,
            "<polygon points=\"{coords}\" fill=\"{fill}\" fill-opacity=\"{fill_opacity:.2}\" \
             stroke=\"{stroke}\" stroke-width=\"{width:.2}\" stroke-linejoin=\"round\"/>"
        );
    }

    fn dot(&mut self, view: &View, center: Point2, radius_px: f64, fill: &str) {
        let (cx, cy) = view.map(center);
        let _ = writeln!(
            self.elements,
            "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"{radius_px:.2}\" fill=\"{fill}\"/>"
        );
    }

    fn rect(
        &mut self,
        rect: (f64, f64, f64, f64),
        fill: &str,
        fill_opacity: f64,
        stroke: &str,
        stroke_width: f64,
    ) {
        let (x, y, width, height) = rect;
        let _ = writeln!(
            self.elements,
            "<rect x=\"{x:.2}\" y=\"{y:.2}\" width=\"{width:.2}\" height=\"{height:.2}\" \
             fill=\"{fill}\" fill-opacity=\"{fill_opacity:.2}\" stroke=\"{stroke}\" \
             stroke-width=\"{stroke_width:.2}\"/>"
        );
    }

    fn text(&mut self, x: f64, y: f64, size: f64, anchor: &str, fill: &str, content: &str) {
        let content = content
            .replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;");
        let _ = writeln!(
            self.elements,
            "<text x=\"{x:.2}\" y=\"{y:.2}\" font-family=\"ui-monospace, Consolas, monospace\" \
             font-size=\"{size:.1}\" text-anchor=\"{anchor}\" fill=\"{fill}\">{content}</text>"
        );
    }

    fn finish(self, title: &str) -> String {
        self.finish_sized(title, HEIGHT)
    }

    fn finish_sized(self, title: &str, height: f64) -> String {
        format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{WIDTH}\" height=\"{height}\" \
             viewBox=\"0 0 {WIDTH} {height}\">\n\
             <title>{title}</title>\n\
             <rect x=\"0\" y=\"0\" width=\"{WIDTH}\" height=\"{height}\" fill=\"#ffffff\"/>\n\
             {}\n</svg>\n",
            self.elements
        )
    }
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

fn sector_points(model: &LineArc, inner_radius: f64, outer_radius: f64) -> Vec<Point2> {
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();
    let steps = 72;
    let mut points = arc_points(model, outer_radius, start, end, steps);
    let mut inner = arc_points(model, inner_radius, start, end, steps);
    inner.reverse();
    points.extend(inner);
    points
}

fn line_strip_points() -> Vec<Point2> {
    vec![
        Point2 {
            x: -LINE_LENGTH,
            y: -PROFILE_MAX,
        },
        Point2 {
            x: 0.0,
            y: -PROFILE_MAX,
        },
        Point2 {
            x: 0.0,
            y: -PROFILE_MIN,
        },
        Point2 {
            x: -LINE_LENGTH,
            y: -PROFILE_MIN,
        },
    ]
}

fn seam_points(model: &LineArc, from: f64, to: f64, steps: usize) -> Vec<Point2> {
    (0..=steps)
        .filter_map(|index| {
            let d = from + (to - from) * index as f64 / steps as f64;
            model.intersect_offsets(d).ok()
        })
        .collect()
}

fn construction_bounds(model: &LineArc) -> Bounds {
    let center = model.center();
    let max_radius = RADIUS + PROFILE_MAX;
    let mut bounds = Bounds::new(-LINE_LENGTH, 0.0, -PROFILE_MAX, -PROFILE_MIN);
    bounds.include(Point2 {
        x: center.x - max_radius,
        y: center.y - max_radius,
    });
    bounds.include(Point2 {
        x: center.x + max_radius,
        y: center.y + max_radius,
    });
    bounds.include(model.vertex());
    bounds.include_points(&seam_points(model, model.min_offset(), PROFILE_MAX, 100));
    bounds
}

fn sweep_bounds(model: &LineArc) -> Bounds {
    let mut bounds = Bounds::new(-LINE_LENGTH, 0.0, -PROFILE_MAX, -PROFILE_MIN);
    bounds.include_points(&line_strip_points());
    bounds.include_points(&sector_points(
        model,
        RADIUS + PROFILE_MIN,
        RADIUS + PROFILE_MAX,
    ));
    bounds.include_points(&seam_points(model, model.min_offset(), PROFILE_MAX, 100));
    bounds.include(model.vertex());
    bounds.include(Point2 { x: RADIUS, y: 0.0 });
    bounds
}

fn draw_construction(canvas: &mut Canvas, view: &View, model: &LineArc) {
    let center = model.center();
    let x_end = view.bounds.max_x - 0.3;
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();

    // Supporting circle and finite base path.
    let full_circle = arc_points(model, RADIUS, 0.0, std::f64::consts::TAU, 160);
    canvas.polyline(view, &full_circle, "#bbbbbb", 1.0, "4 4");
    canvas.line(
        view,
        Point2 {
            x: -LINE_LENGTH,
            y: 0.0,
        },
        Point2 { x: 0.0, y: 0.0 },
        "#111111",
        2.2,
        "",
    );
    let base_arc = arc_points(model, RADIUS, start, end, 100);
    canvas.polyline(view, &base_arc, "#111111", 2.2, "");
    canvas.dot(view, center, 3.0, "#111111");
    canvas.dot(view, Point2 { x: 0.0, y: 0.0 }, 3.5, "#111111");

    // Representative corresponding offsets.
    for d in [PROFILE_MIN, -0.25 * RADIUS, 0.0, 0.25 * RADIUS, PROFILE_MAX] {
        let intersection = model.intersect_offsets(d);
        let color = if intersection.is_ok() {
            LINE_COLOR
        } else {
            LIMIT_COLOR
        };
        canvas.line(
            view,
            Point2 {
                x: -LINE_LENGTH,
                y: -d,
            },
            Point2 { x: x_end, y: -d },
            color,
            1.1,
            "",
        );
        let offset_radius = RADIUS + d;
        if offset_radius > 0.0 {
            let circle = arc_points(model, offset_radius, 0.0, std::f64::consts::TAU, 120);
            canvas.polyline(
                view,
                &circle,
                if intersection.is_ok() {
                    ARC_COLOR
                } else {
                    LIMIT_COLOR
                },
                1.1,
                "",
            );
        }
        if let Ok(point) = intersection {
            canvas.dot(
                view,
                point,
                3.6,
                if d >= 0.0 {
                    FINITE_COLOR
                } else {
                    EXTENSION_COLOR
                },
            );
        }
    }

    // Seam on the finite bodies and on the supporting extensions.
    let minimum = model.min_offset();
    if minimum < 0.0 {
        let dashed = seam_points(model, minimum, 0.0, 160);
        canvas.polyline(view, &dashed, EXTENSION_COLOR, 2.2, "7 5");
    }
    if PROFILE_MAX > 0.0 {
        let solid = seam_points(model, 0.0, PROFILE_MAX, 160);
        canvas.polyline(view, &solid, FINITE_COLOR, 2.8, "");
    }
    let vertex = model.vertex();
    canvas.dot(view, vertex, 4.0, LIMIT_COLOR);
    let (vx, vy) = view.map(vertex);
    canvas.text(
        vx + 7.0,
        vy - 7.0,
        11.0,
        "start",
        LIMIT_COLOR,
        &format!("d_min={:.3}", minimum),
    );

    legend(
        canvas,
        18.0,
        58.0,
        &[
            (LINE_COLOR, "offset line"),
            (ARC_COLOR, "offset circle"),
            (FINITE_COLOR, "P(d), d>=0 (finite)"),
            (EXTENSION_COLOR, "P(d), d<0 (extensions)"),
            (LIMIT_COLOR, "no real intersection"),
        ],
    );
    canvas.text(
        18.0,
        32.0,
        15.0,
        "start",
        "#111111",
        &format!(
            "{:.0} deg: offset construction, R={:.0}, profile d in [{:.1}, {:.1}]",
            model.corner_angle_rad.to_degrees(),
            RADIUS,
            PROFILE_MIN,
            PROFILE_MAX
        ),
    );
}

fn draw_sweep(canvas: &mut Canvas, view: &View, model: &LineArc) {
    let strip = line_strip_points();
    let sector = sector_points(model, RADIUS + PROFILE_MIN, RADIUS + PROFILE_MAX);
    canvas.polygon(view, &strip, LINE_COLOR, 0.23, LINE_COLOR, 1.6);
    canvas.polygon(view, &sector, ARC_COLOR, 0.23, ARC_COLOR, 1.6);

    // Base path and joint.
    canvas.line(
        view,
        Point2 {
            x: -LINE_LENGTH,
            y: 0.0,
        },
        Point2 { x: 0.0, y: 0.0 },
        "#111111",
        1.8,
        "",
    );
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();
    canvas.polyline(
        view,
        &arc_points(model, RADIUS, start, end, 100),
        "#111111",
        1.8,
        "",
    );
    canvas.dot(view, Point2 { x: 0.0, y: 0.0 }, 3.0, "#111111");

    // Seam. The d<0 part lives on the supporting extensions, outside both
    // finite regions; it is drawn dashed on purpose.
    let minimum = model.min_offset();
    let extension_start = PROFILE_MIN.max(minimum);
    if extension_start < 0.0 {
        let dashed = seam_points(model, extension_start, 0.0, 180);
        canvas.polyline(view, &dashed, EXTENSION_COLOR, 2.4, "7 5");
    }
    if PROFILE_MAX > 0.0 {
        let solid = seam_points(model, 0.0, PROFILE_MAX, 180);
        canvas.polyline(view, &solid, FINITE_COLOR, 3.0, "");
    }

    let vertex = model.vertex();
    canvas.dot(view, vertex, 4.2, LIMIT_COLOR);
    let (vx, vy) = view.map(vertex);
    canvas.text(vx + 7.0, vy - 7.0, 11.0, "start", LIMIT_COLOR, "vertex");

    // Region labels.
    let line_label = Point2 {
        x: -0.72 * LINE_LENGTH,
        y: 0.0,
    };
    let (lx, ly) = view.map(line_label);
    canvas.text(lx, ly - 8.0, 12.0, "middle", LINE_COLOR, "line sweep");

    let mid_radius = RADIUS + 0.5 * (PROFILE_MIN + PROFILE_MAX);
    let mid_angle = start + 0.5 * (end - start);
    let center = model.center();
    let arc_label = Point2 {
        x: center.x + mid_radius * mid_angle.cos(),
        y: center.y + mid_radius * mid_angle.sin(),
    };
    let (ax, ay) = view.map(arc_label);
    canvas.text(ax, ay, 12.0, "middle", ARC_COLOR, "arc sweep");

    draw_profile_bar(canvas, model, SECOND_PANEL_TOP);
    canvas.text(
        18.0,
        SECOND_PANEL_TOP + 2.0,
        15.0,
        "start",
        "#111111",
        &format!(
            "{:.0} deg: finite sweep regions and AOIM seam",
            model.corner_angle_rad.to_degrees()
        ),
    );
}

fn draw_profile_bar(canvas: &mut Canvas, model: &LineArc, panel_top: f64) {
    let minimum = model.min_offset();
    let bar_x = 16.0;
    let bar_top = panel_top + 80.0;
    let bar_width = 12.0;
    let bar_height = 190.0;

    canvas.text(8.0, bar_top - 12.0, 10.0, "start", "#222222", "profile d");

    let y = |d: f64| bar_top + (PROFILE_MAX - d) / (PROFILE_MAX - PROFILE_MIN) * bar_height;
    let segment = |canvas: &mut Canvas, low: f64, high: f64, color: &str, opacity: f64| {
        if low >= high {
            return;
        }
        let y1 = y(low);
        let y2 = y(high);
        canvas.rect(
            (bar_x, y1.min(y2), bar_width, (y2 - y1).abs()),
            color,
            opacity,
            color,
            0.5,
        );
    };

    if PROFILE_MIN < minimum {
        segment(
            canvas,
            PROFILE_MIN,
            minimum.min(PROFILE_MAX),
            LIMIT_COLOR,
            0.55,
        );
    }
    let covered_start = PROFILE_MIN.max(minimum);
    if covered_start < 0.0 {
        segment(
            canvas,
            covered_start,
            0.0_f64.min(PROFILE_MAX),
            EXTENSION_COLOR,
            0.65,
        );
    }
    if PROFILE_MAX > 0.0 {
        segment(
            canvas,
            0.0_f64.max(covered_start),
            PROFILE_MAX,
            FINITE_COLOR,
            0.65,
        );
    }
    canvas.rect(
        (bar_x, bar_top, bar_width, bar_height),
        "none",
        0.0,
        "#333333",
        1.0,
    );

    let tick = |canvas: &mut Canvas, d: f64, label: &str| {
        let ty = y(d);
        canvas.screen_line(
            (bar_x + bar_width, ty),
            (bar_x + bar_width + 4.0, ty),
            "#333333",
            1.0,
            "",
        );
        canvas.text(
            bar_x + bar_width + 7.0,
            ty + 3.5,
            9.5,
            "start",
            "#333333",
            label,
        );
    };
    tick(canvas, PROFILE_MAX, &format!("{:+.1}", PROFILE_MAX));
    tick(canvas, 0.0, "0");
    tick(canvas, PROFILE_MIN, &format!("{:+.1}", PROFILE_MIN));
    if (PROFILE_MIN..=PROFILE_MAX).contains(&minimum) {
        let tick_y = y(minimum);
        canvas.screen_line((bar_x, tick_y), (bar_x - 4.0, tick_y), LIMIT_COLOR, 1.8, "");
    }

    let legend_x = 8.0;
    let legend_y = bar_top + bar_height + 18.0;
    let rows = [
        (FINITE_COLOR, "finite"),
        (EXTENSION_COLOR, "extensions"),
        (LIMIT_COLOR, "no seam"),
    ];
    for (index, (color, label)) in rows.iter().enumerate() {
        let row_y = legend_y + index as f64 * 15.0;
        canvas.rect((legend_x, row_y - 4.0, 12.0, 4.0), color, 1.0, color, 0.0);
        canvas.text(legend_x + 18.0, row_y, 9.0, "start", "#333333", label);
    }
}

fn legend(canvas: &mut Canvas, x: f64, y: f64, rows: &[(&str, &str)]) {
    let height = 18.0 * rows.len() as f64 + 12.0;
    canvas.rect((x, y, 285.0, height), "#ffffff", 0.92, "#cccccc", 1.0);
    for (index, (color, label)) in rows.iter().enumerate() {
        let row_y = y + 18.0 + index as f64 * 18.0;
        canvas.rect((x + 10.0, row_y - 8.0, 22.0, 4.0), color, 1.0, color, 0.0);
        canvas.text(x + 40.0, row_y, 11.0, "start", "#333333", label);
    }
}

fn render(model: &LineArc) -> String {
    let mut canvas = Canvas::new();
    let construction_view = View::new(construction_bounds(model), 30.0);
    let sweep_view = View::new(sweep_bounds(model), SECOND_PANEL_TOP);
    draw_construction(&mut canvas, &construction_view, model);
    draw_sweep(&mut canvas, &sweep_view, model);
    canvas.screen_line(
        (20.0, SECOND_PANEL_TOP - 20.0),
        (WIDTH - 20.0, SECOND_PANEL_TOP - 20.0),
        "#dddddd",
        1.0,
        "",
    );
    let title = format!(
        "AOIM Line-Arc {:.0} deg",
        model.corner_angle_rad.to_degrees()
    );
    canvas.finish(&title)
}

fn main() {
    let out_dir = std::env::args().nth(1).unwrap_or_else(|| "out".to_owned());
    if let Err(error) = fs::create_dir_all(&out_dir) {
        eprintln!("cannot create {out_dir}: {error}");
        std::process::exit(1);
    }

    let mut links = String::new();
    for degrees in [30.0_f64, 90.0, 150.0] {
        let model = match LineArc::new(RADIUS, degrees.to_radians()) {
            Ok(model) => model,
            Err(error) => {
                eprintln!("cannot build {degrees} deg model: {error:?}");
                std::process::exit(1);
            }
        };
        let file_name = format!("aoim-line-arc-{degrees:.0}.svg");
        let path = Path::new(&out_dir).join(&file_name);
        if let Err(error) = fs::write(&path, render(&model)) {
            eprintln!("cannot write {}: {error}", path.display());
            std::process::exit(1);
        }
        let _ = writeln!(
            links,
            "<li><a href=\"{file_name}\">{degrees:.0} deg</a> \
             <img src=\"{file_name}\" alt=\"AOIM {degrees:.0} deg\"></li>"
        );
    }

    let mut comparisons = String::new();
    for degrees in [30.0_f64, 90.0, 150.0] {
        let model = match LineArc::new(RADIUS, degrees.to_radians()) {
            Ok(model) => model,
            Err(error) => {
                eprintln!("cannot build {degrees} deg model: {error:?}");
                std::process::exit(1);
            }
        };
        let file_name = format!("aoim-vs-hybrid-{degrees:.0}.svg");
        let path = Path::new(&out_dir).join(&file_name);
        if let Err(error) = fs::write(&path, render_comparison(&model)) {
            eprintln!("cannot write {}: {error}", path.display());
            std::process::exit(1);
        }
        let _ = writeln!(
            comparisons,
            "<li><a href=\"{file_name}\">{degrees:.0} deg: original vs hybrid</a> \
             <img src=\"{file_name}\" alt=\"AOIM vs hybrid {degrees:.0} deg\"></li>"
        );
    }

    let mut adaptive_links = String::new();
    for (degrees, line_length, max_miter, max_deviation) in [
        (150.0_f64, 20.0_f64, 6.5_f64, 1.5_f64),
        (179.0, 20.0, 6.5, 3.5),
    ] {
        let model = match LineArc::new(RADIUS, degrees.to_radians()) {
            Ok(model) => model,
            Err(error) => {
                eprintln!("cannot build {degrees} deg model: {error:?}");
                std::process::exit(1);
            }
        };
        let file_name = format!("aoim-adaptive-{degrees:.0}.svg");
        let path = Path::new(&out_dir).join(&file_name);
        let request = AdaptiveParams {
            max_miter_length: max_miter,
            max_path_deviation: max_deviation,
            preserve_endpoints: true,
            preserve_tangency: false,
        };
        if let Err(error) = fs::write(
            &path,
            render_adaptive_comparison(&model, request, line_length),
        ) {
            eprintln!("cannot write {}: {error}", path.display());
            std::process::exit(1);
        }
        let _ = writeln!(
            adaptive_links,
            "<li><a href=\"{file_name}\">{degrees:.0} deg: adaptive path</a> \
             <img src=\"{file_name}\" alt=\"Adaptive AOIM {degrees:.0} deg\"></li>"
        );
    }

    let index = format!(
        "<!doctype html>\n<html lang=\"en\"><meta charset=\"utf-8\">\
         <title>AOIM Line-Arc visuals</title>\n\
         <style>body{{font-family:system-ui,sans-serif;margin:2rem;background:#fafafa}}\
         li{{list-style:none;margin:2rem 0}}img{{max-width:1000px;width:100%;\
         border:1px solid #ddd;background:#fff}}a{{color:#2f6fbf}}</style>\n\
         <h1>AOIM Line-Arc - geometric visuals</h1>\n\
         <h2>Original AOIM vs hybrid at junction</h2>\n\
         <p>Four panels: original construction, hybrid construction, original regions, \
         hybrid trimmed regions. Purple is the linear seam on the extensions; green is the \
         AOIM seam on the finite side.</p>\n\
         <ul>{comparisons}</ul>\n\
         <h2>Adaptive path regularization</h2>\n\
         <p>The incoming line is shortened to A, a straight segment is inserted tangent to \
         the arc at Q, and the remaining arc is kept. Solid purple is the miter-bounded \
         adaptive cut; teal dashed is the event-driven first-valid cut. Orange dashed is \
         the hybrid wedge; green is the line-line miter at A.</p>\n\
         <ul>{adaptive_links}</ul>\n\
         <h2>Single-strategy figures</h2>\n\
         <p>Top: offset construction. Bottom: finite sweep regions and the seam. \
         Green = finite bodies, orange = supporting extensions, red = no real intersection.</p>\n\
         <ul>{links}</ul></html>\n"
    );
    let index_path = Path::new(&out_dir).join("index.html");
    if let Err(error) = fs::write(&index_path, index) {
        eprintln!("cannot write {}: {error}", index_path.display());
        std::process::exit(1);
    }
    println!("wrote {}", index_path.display());
}

fn built_bounds(built: &BuiltRegions) -> Bounds {
    let mut bounds = Bounds::new(
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for polygon in [&built.line, &built.arc, &built.expected] {
        for point in &polygon.points {
            bounds.include(*point);
        }
    }
    bounds
}

fn draw_hybrid_construction(canvas: &mut Canvas, view: &View, model: &LineArc, panel_top: f64) {
    let center = model.center();
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();
    let x_end = view.bounds.max_x - 0.3;

    let full_circle = arc_points(model, RADIUS, 0.0, std::f64::consts::TAU, 160);
    canvas.polyline(view, &full_circle, "#bbbbbb", 1.0, "4 4");
    canvas.line(
        view,
        Point2 {
            x: -LINE_LENGTH,
            y: 0.0,
        },
        Point2 { x: 0.0, y: 0.0 },
        "#111111",
        2.2,
        "",
    );
    canvas.polyline(
        view,
        &arc_points(model, RADIUS, start, end, 100),
        "#111111",
        2.2,
        "",
    );
    canvas.dot(view, center, 3.0, "#111111");
    canvas.dot(view, Point2 { x: 0.0, y: 0.0 }, 3.5, "#111111");

    for d in [PROFILE_MIN, -0.25 * RADIUS, 0.0, 0.25 * RADIUS, PROFILE_MAX] {
        let line_end = if d < 0.0 {
            model
                .junction_linear_point(d)
                .map(|point| point.x)
                .unwrap_or(x_end)
        } else {
            x_end
        };
        canvas.line(
            view,
            Point2 {
                x: -LINE_LENGTH,
                y: -d,
            },
            Point2 { x: line_end, y: -d },
            LINE_COLOR,
            1.1,
            "",
        );
        let radius = RADIUS + d;
        if radius > 0.0 {
            let circle = arc_points(model, radius, 0.0, std::f64::consts::TAU, 120);
            canvas.polyline(view, &circle, ARC_COLOR, 1.0, "");
        }
        if d < 0.0 {
            if let (Ok(joint), Ok(seam)) =
                (model.joint_arc_point(d), model.junction_linear_point(d))
            {
                canvas.line(view, joint, seam, HYBRID_COLOR, 2.0, "");
                canvas.dot(view, joint, 3.2, HYBRID_COLOR);
                canvas.dot(view, seam, 3.6, EXTENSION_COLOR);
            }
        } else if let Ok(point) = model.intersect_offsets(d) {
            canvas.dot(view, point, 3.6, FINITE_COLOR);
        }
    }

    let straight_end = model.junction_linear_point(PROFILE_MIN).unwrap();
    canvas.line(
        view,
        Point2 { x: 0.0, y: 0.0 },
        straight_end,
        HYBRID_COLOR,
        2.8,
        "",
    );
    canvas.polyline(
        view,
        &seam_points(model, 0.0, PROFILE_MAX, 160),
        FINITE_COLOR,
        2.8,
        "",
    );
    let vertex = model.vertex();
    canvas.dot(view, vertex, 4.0, LIMIT_COLOR);

    legend(
        canvas,
        18.0,
        panel_top + 56.0,
        &[
            (LINE_COLOR, "offset line"),
            (ARC_COLOR, "offset circle"),
            (FINITE_COLOR, "AOIM seam, d>=0"),
            (HYBRID_COLOR, "linear seam + tangent extension, d<0"),
            (EXTENSION_COLOR, "seam point on extension"),
            (LIMIT_COLOR, "no real intersection"),
        ],
    );
    canvas.text(
        18.0,
        panel_top + 2.0,
        15.0,
        "start",
        "#111111",
        &format!(
            "{:.0} deg: hybrid at junction, R={:.0}",
            model.corner_angle_rad.to_degrees(),
            RADIUS
        ),
    );
}

fn draw_built_regions(
    canvas: &mut Canvas,
    view: &View,
    model: &LineArc,
    built: &BuiltRegions,
    panel_top: f64,
    title: &str,
    negative_color: &str,
) {
    canvas.polygon(view, &built.line.points, LINE_COLOR, 0.24, LINE_COLOR, 1.6);
    canvas.polygon(view, &built.arc.points, ARC_COLOR, 0.24, ARC_COLOR, 1.6);
    canvas.line(
        view,
        Point2 {
            x: -LINE_LENGTH,
            y: 0.0,
        },
        Point2 { x: 0.0, y: 0.0 },
        "#111111",
        1.6,
        "",
    );
    canvas.polyline(
        view,
        &arc_points(
            model,
            RADIUS,
            model.arc_start_angle(),
            model.arc_end_angle(),
            100,
        ),
        "#111111",
        1.6,
        "",
    );

    let negative: Vec<Point2> = built
        .seam
        .iter()
        .filter(|sample| sample.d <= 0.0)
        .map(|sample| sample.point)
        .collect();
    let positive: Vec<Point2> = built
        .seam
        .iter()
        .filter(|sample| sample.d >= 0.0)
        .map(|sample| sample.point)
        .collect();
    canvas.polyline(view, &negative, negative_color, 3.0, "");
    canvas.polyline(view, &positive, FINITE_COLOR, 3.0, "");
    canvas.dot(view, model.vertex(), 4.0, LIMIT_COLOR);

    let raster = built.raster;
    let box_height = 92.0;
    canvas.rect(
        (18.0, panel_top + 44.0, 380.0, box_height),
        "#ffffff",
        0.93,
        "#cccccc",
        1.0,
    );
    canvas.text(
        28.0,
        panel_top + 64.0,
        10.0,
        "start",
        "#333333",
        &format!(
            "areas: line={:.3} arc={:.3} expected={:.3}",
            built.line.area(),
            built.arc.area(),
            built.expected.area()
        ),
    );
    canvas.text(
        28.0,
        panel_top + 80.0,
        10.0,
        "start",
        "#333333",
        &format!(
            "line+arc-expected = {:+.2e} ({:+.2e})",
            built.area_balance, built.area_balance_fraction
        ),
    );
    canvas.text(
        28.0,
        panel_top + 96.0,
        10.0,
        "start",
        "#333333",
        &format!(
            "raster overlap={:.2e} gap={:.2e} sym={:.2e}",
            raster.overlap_area / raster.expected_area,
            raster.gap_area / raster.expected_area,
            raster.symmetric_difference_fraction
        ),
    );
    canvas.text(
        28.0,
        panel_top + 112.0,
        10.0,
        "start",
        if built.self_intersecting {
            LIMIT_COLOR
        } else {
            "#333333"
        },
        &format!("self-intersecting = {}", built.self_intersecting),
    );
    canvas.text(18.0, panel_top + 2.0, 15.0, "start", "#111111", title);
}

fn draw_original_regions(canvas: &mut Canvas, view: &View, model: &LineArc, panel_top: f64) {
    match build_regions(
        model,
        Strategy::Analytical,
        PROFILE_MIN,
        PROFILE_MAX,
        20.0,
        401,
        96,
    ) {
        Ok(built) => draw_built_regions(
            canvas,
            view,
            model,
            &built,
            panel_top,
            &format!(
                "{:.0} deg: original AOIM, closed trim",
                model.corner_angle_rad.to_degrees()
            ),
            EXTENSION_COLOR,
        ),
        Err(RegionError::UncoveredProfile { first_missing }) => {
            let (line, arc, untrimmed) =
                build_untrimmed(model, PROFILE_MIN, PROFILE_MAX, 20.0, 96, 600).unwrap();
            canvas.polygon(view, &line.points, LINE_COLOR, 0.24, LINE_COLOR, 1.6);
            canvas.polygon(view, &arc.points, ARC_COLOR, 0.24, ARC_COLOR, 1.6);
            canvas.line(
                view,
                Point2 {
                    x: -LINE_LENGTH,
                    y: 0.0,
                },
                Point2 { x: 0.0, y: 0.0 },
                "#111111",
                1.6,
                "",
            );
            canvas.polyline(
                view,
                &arc_points(
                    model,
                    RADIUS,
                    model.arc_start_angle(),
                    model.arc_end_angle(),
                    100,
                ),
                "#111111",
                1.6,
                "",
            );
            let minimum = model.min_offset();
            let visible = seam_points(model, minimum.max(PROFILE_MIN), PROFILE_MAX, 160);
            let negative: Vec<Point2> = visible
                .iter()
                .copied()
                .filter(|point| point.y >= 0.0)
                .collect();
            let positive: Vec<Point2> = visible
                .iter()
                .copied()
                .filter(|point| point.y <= 0.0)
                .collect();
            canvas.polyline(view, &negative, EXTENSION_COLOR, 2.2, "7 5");
            canvas.polyline(view, &positive, FINITE_COLOR, 2.8, "");
            canvas.dot(view, model.vertex(), 4.0, LIMIT_COLOR);

            canvas.rect(
                (18.0, panel_top + 44.0, 400.0, 82.0),
                "#ffffff",
                0.93,
                "#cccccc",
                1.0,
            );
            canvas.text(
                28.0,
                panel_top + 66.0,
                10.0,
                "start",
                LIMIT_COLOR,
                &format!("AOIM seam undefined below d_min={minimum:.4}",),
            );
            canvas.text(
                28.0,
                panel_top + 82.0,
                10.0,
                "start",
                "#333333",
                &format!(
                    "first missing d={first_missing:.4}; untrimmed overlap={:.3}",
                    untrimmed.overlap_area
                ),
            );
            canvas.text(
                28.0,
                panel_top + 98.0,
                10.0,
                "start",
                "#333333",
                "no closed original AOIM regions for this profile",
            );
            canvas.text(
                18.0,
                panel_top + 2.0,
                15.0,
                "start",
                "#111111",
                &format!(
                    "{:.0} deg: original AOIM, uncovered profile",
                    model.corner_angle_rad.to_degrees()
                ),
            );
        }
        Err(error) => {
            canvas.text(
                18.0,
                panel_top + 28.0,
                12.0,
                "start",
                LIMIT_COLOR,
                &format!("cannot build original AOIM regions: {error:?}"),
            );
        }
    }
}

fn render_comparison(model: &LineArc) -> String {
    let stride = PANEL_HEIGHT + PANEL_GAP;
    let tops = [
        30.0,
        30.0 + stride,
        30.0 + 2.0 * stride,
        30.0 + 3.0 * stride,
    ];
    let mut canvas = Canvas::new();

    let construction_view = View::new(construction_bounds(model), tops[0]);
    draw_construction(&mut canvas, &construction_view, model);

    let mut hybrid_bounds = construction_bounds(model);
    hybrid_bounds.include(model.junction_linear_point(PROFILE_MIN).unwrap());
    hybrid_bounds.include(model.joint_arc_point(PROFILE_MIN).unwrap());
    let hybrid_construction_view = View::new(hybrid_bounds, tops[1]);
    draw_hybrid_construction(&mut canvas, &hybrid_construction_view, model, tops[1]);

    let original_view = View::new(sweep_bounds(model), tops[2]);
    draw_original_regions(&mut canvas, &original_view, model, tops[2]);

    let hybrid = build_regions(
        model,
        Strategy::HybridAtJunction,
        PROFILE_MIN,
        PROFILE_MAX,
        20.0,
        401,
        96,
    )
    .unwrap();
    let hybrid_view = View::new(built_bounds(&hybrid), tops[3]);
    draw_built_regions(
        &mut canvas,
        &hybrid_view,
        model,
        &hybrid,
        tops[3],
        &format!(
            "{:.0} deg: hybrid AOIM, closed trim",
            model.corner_angle_rad.to_degrees()
        ),
        HYBRID_COLOR,
    );

    for top in &tops[1..] {
        canvas.screen_line(
            (20.0, top - 20.0),
            (WIDTH - 20.0, top - 20.0),
            "#dddddd",
            1.0,
            "",
        );
    }
    let height = tops[3] + PANEL_HEIGHT + 30.0;
    canvas.finish_sized(
        &format!(
            "AOIM original vs hybrid, {:.0} deg",
            model.corner_angle_rad.to_degrees()
        ),
        height,
    )
}

fn adaptive_bounds(regions: &AdaptiveRegions) -> Bounds {
    let mut bounds = Bounds::new(
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for polygon in [
        &regions.line,
        &regions.straight,
        &regions.arc,
        &regions.expected,
    ] {
        for point in &polygon.points {
            bounds.include(*point);
        }
    }
    bounds
}

fn draw_adaptive_path(
    canvas: &mut Canvas,
    view: &View,
    model: &LineArc,
    path: &AdaptivePath,
    regions: &AdaptiveRegions,
    event: Option<(&AdaptivePath, &AdaptiveRegions)>,
    panel_top: f64,
) {
    let cut = path.cut;
    let start = model.arc_start_angle();
    let end = model.arc_end_angle();
    let remaining_start = start + cut.removed_arc_angle;

    // Original path, dashed.
    canvas.line(
        view,
        Point2 {
            x: -path.line_length,
            y: 0.0,
        },
        Point2 { x: 0.0, y: 0.0 },
        "#aaaaaa",
        1.4,
        "6 5",
    );
    canvas.polyline(
        view,
        &arc_points(model, RADIUS, start, end, 100),
        "#aaaaaa",
        1.4,
        "6 5",
    );

    // Hybrid wedge for comparison.
    let hybrid_seam = (0..=120)
        .filter_map(|index| {
            let d = PROFILE_MIN + (PROFILE_MAX - PROFILE_MIN) * index as f64 / 120.0;
            aoim_line_arc_research::sweep::seam_point(model, Strategy::HybridAtJunction, d).ok()
        })
        .collect::<Vec<_>>();
    canvas.polyline(view, &hybrid_seam, EXTENSION_COLOR, 1.6, "8 5");

    // Modified path: line to A, tangent A-Q, remaining arc Q-end.
    canvas.line(
        view,
        Point2 {
            x: -path.line_length,
            y: 0.0,
        },
        cut.cut_point,
        "#111111",
        2.2,
        "",
    );
    canvas.line(
        view,
        cut.cut_point,
        cut.tangent_point,
        HYBRID_COLOR,
        2.8,
        "",
    );
    canvas.polyline(
        view,
        &arc_points(model, RADIUS, remaining_start, end, 100),
        "#111111",
        2.2,
        "",
    );

    // Line-line miter at A.
    canvas.polyline(
        view,
        &regions.seam.iter().map(|s| s.point).collect::<Vec<_>>(),
        FINITE_COLOR,
        2.4,
        "",
    );

    if let Some((event_path, event_regions)) = event {
        let event_cut = event_path.cut;
        let event_arc_start = start + event_cut.removed_arc_angle;
        canvas.line(
            view,
            event_cut.cut_point,
            event_cut.tangent_point,
            EVENT_COLOR,
            2.2,
            "6 5",
        );
        canvas.polyline(
            view,
            &arc_points(model, RADIUS, event_arc_start, end, 100),
            EVENT_COLOR,
            2.0,
            "6 5",
        );
        canvas.polyline(
            view,
            &event_regions
                .seam
                .iter()
                .map(|sample| sample.point)
                .collect::<Vec<_>>(),
            EVENT_COLOR,
            2.0,
            "6 5",
        );
        canvas.dot(view, event_cut.cut_point, 3.2, EVENT_COLOR);
        canvas.dot(view, event_cut.tangent_point, 3.2, EVENT_COLOR);
        let (ex, ey) = view.map(event_cut.cut_point);
        let (eqx, eqy) = view.map(event_cut.tangent_point);
        canvas.text(ex - 30.0, ey - 10.0, 9.5, "start", EVENT_COLOR, "A_e");
        canvas.text(eqx - 30.0, eqy + 6.0, 9.5, "start", EVENT_COLOR, "Q_e");
    }

    canvas.dot(view, cut.cut_point, 4.0, HYBRID_COLOR);
    canvas.dot(view, cut.tangent_point, 4.0, FINITE_COLOR);
    canvas.dot(view, Point2 { x: 0.0, y: 0.0 }, 3.2, "#111111");
    let (ax, ay) = view.map(cut.cut_point);
    let (qx, qy) = view.map(cut.tangent_point);
    canvas.text(ax + 6.0, ay - 6.0, 10.0, "start", HYBRID_COLOR, "A");
    canvas.text(qx + 6.0, qy + 12.0, 10.0, "start", FINITE_COLOR, "Q");

    canvas.rect(
        (18.0, panel_top + 44.0, 410.0, 116.0),
        "#ffffff",
        0.93,
        "#cccccc",
        1.0,
    );
    canvas.text(
        28.0,
        panel_top + 64.0,
        10.0,
        "start",
        HYBRID_COLOR,
        &format!(
            "miter-bounded: turn={:.2} deg, removed={:.2} deg, miter={:.3}, dev={:.3}",
            cut.theta.to_degrees(),
            cut.removed_arc_angle.to_degrees(),
            cut.miter_length,
            cut.path_deviation,
        ),
    );
    if let Some((event_path, _)) = event {
        let event_cut = event_path.cut;
        canvas.text(
            28.0,
            panel_top + 80.0,
            10.0,
            "start",
            EVENT_COLOR,
            &format!(
                "event-driven: turn={:.2} deg, removed={:.2} deg, miter={:.3}, dev={:.3}",
                event_cut.theta.to_degrees(),
                event_cut.removed_arc_angle.to_degrees(),
                event_cut.miter_length,
                event_cut.path_deviation,
            ),
        );
    }
    canvas.text(
        28.0,
        panel_top + 98.0,
        10.0,
        "start",
        "#333333",
        "gray dashed = original; orange dashed = hybrid; teal dashed = event-driven",
    );
    canvas.text(
        28.0,
        panel_top + 114.0,
        10.0,
        "start",
        "#333333",
        "purple solid = miter-bounded adaptive",
    );
    canvas.text(
        18.0,
        panel_top + 2.0,
        15.0,
        "start",
        "#111111",
        &format!(
            "{:.0} deg: miter-bounded vs event-driven path",
            model.corner_angle_rad.to_degrees()
        ),
    );
}

fn draw_adaptive_regions(
    canvas: &mut Canvas,
    view: &View,
    model: &LineArc,
    path: &AdaptivePath,
    regions: &AdaptiveRegions,
    event: Option<(&AdaptivePath, &AdaptiveRegions)>,
    panel_top: f64,
) {
    canvas.polygon(
        view,
        &regions.line.points,
        LINE_COLOR,
        0.24,
        LINE_COLOR,
        1.5,
    );
    canvas.polygon(
        view,
        &regions.straight.points,
        HYBRID_COLOR,
        0.24,
        HYBRID_COLOR,
        1.5,
    );
    canvas.polygon(view, &regions.arc.points, ARC_COLOR, 0.24, ARC_COLOR, 1.5);
    let seam: Vec<Point2> = regions.seam.iter().map(|sample| sample.point).collect();
    canvas.polyline(view, &seam, FINITE_COLOR, 2.6, "");

    let normal = Point2 {
        x: path.cut.theta.sin(),
        y: -path.cut.theta.cos(),
    };
    let q0 = Point2 {
        x: path.cut.tangent_point.x + path.d0 * normal.x,
        y: path.cut.tangent_point.y + path.d0 * normal.y,
    };
    let q1 = Point2 {
        x: path.cut.tangent_point.x + path.d1 * normal.x,
        y: path.cut.tangent_point.y + path.d1 * normal.y,
    };
    canvas.line(view, q0, q1, "#888888", 1.2, "4 4");

    if let Some((event_path, event_regions)) = event {
        for polygon in [
            &event_regions.line,
            &event_regions.straight,
            &event_regions.arc,
        ] {
            canvas.polyline(view, &polygon.points, EVENT_COLOR, 1.4, "5 4");
        }
        let event_seam: Vec<Point2> = event_regions
            .seam
            .iter()
            .map(|sample| sample.point)
            .collect();
        canvas.polyline(view, &event_seam, EVENT_COLOR, 2.0, "5 4");
        let event_normal = Point2 {
            x: event_path.cut.theta.sin(),
            y: -event_path.cut.theta.cos(),
        };
        let e0 = Point2 {
            x: event_path.cut.tangent_point.x + event_path.d0 * event_normal.x,
            y: event_path.cut.tangent_point.y + event_path.d0 * event_normal.y,
        };
        let e1 = Point2 {
            x: event_path.cut.tangent_point.x + event_path.d1 * event_normal.x,
            y: event_path.cut.tangent_point.y + event_path.d1 * event_normal.y,
        };
        canvas.line(view, e0, e1, EVENT_COLOR, 1.0, "3 3");
    }

    let raster = regions.raster;
    canvas.rect(
        (18.0, panel_top + 44.0, 390.0, 104.0),
        "#ffffff",
        0.93,
        "#cccccc",
        1.0,
    );
    canvas.text(
        28.0,
        panel_top + 64.0,
        10.0,
        "start",
        "#333333",
        &format!(
            "areas: line={:.3} straight={:.3} arc={:.3} expected={:.3}",
            regions.line.area(),
            regions.straight.area(),
            regions.arc.area(),
            regions.expected.area(),
        ),
    );
    canvas.text(
        28.0,
        panel_top + 80.0,
        10.0,
        "start",
        "#333333",
        &format!(
            "balance={:+.2e} overlap={:.2e} gap={:.2e} sym={:.2e}",
            regions.area_balance_fraction,
            raster.overlap_area / raster.expected_area,
            raster.gap_area / raster.expected_area,
            raster.symmetric_difference_fraction,
        ),
    );
    if let Some((_, event_regions)) = event {
        let event_raster = event_regions.raster;
        canvas.text(
            28.0,
            panel_top + 96.0,
            10.0,
            "start",
            EVENT_COLOR,
            &format!(
                "event areas: expected={:.3}, balance={:+.2e}, sym={:.2e}",
                event_regions.expected.area(),
                event_regions.area_balance_fraction,
                event_raster.symmetric_difference_fraction,
            ),
        );
    }
    canvas.text(
        28.0,
        panel_top + 112.0,
        10.0,
        "start",
        if regions.self_intersecting {
            LIMIT_COLOR
        } else {
            "#333333"
        },
        &format!(
            "miter-bounded self={} | teal dashed = event-driven",
            regions.self_intersecting
        ),
    );
    canvas.text(
        18.0,
        panel_top + 2.0,
        15.0,
        "start",
        "#111111",
        &format!(
            "{:.0} deg: trimmed regions (solid bounded / dashed event)",
            model.corner_angle_rad.to_degrees()
        ),
    );
}

fn render_adaptive_comparison(model: &LineArc, params: AdaptiveParams, line_length: f64) -> String {
    let stride = PANEL_HEIGHT + PANEL_GAP;
    let tops = [30.0, 30.0 + stride, 30.0 + 2.0 * stride];
    let path = match AdaptivePath::solve(*model, PROFILE_MIN, PROFILE_MAX, line_length, params) {
        Ok(path) => path,
        Err(error) => {
            let mut canvas = Canvas::new();
            canvas.text(
                24.0,
                50.0,
                14.0,
                "start",
                LIMIT_COLOR,
                &format!("adaptive path infeasible: {error:?}"),
            );
            return canvas.finish("adaptive path infeasible");
        }
    };
    let regions = match path.build_regions(401, 96, 600) {
        Ok(regions) => regions,
        Err(error) => {
            let mut canvas = Canvas::new();
            canvas.text(
                24.0,
                50.0,
                14.0,
                "start",
                LIMIT_COLOR,
                &format!("adaptive regions failed: {error:?}"),
            );
            return canvas.finish("adaptive regions failed");
        }
    };

    let event = AdaptivePath::solve_event_driven(
        *model,
        PROFILE_MIN,
        PROFILE_MAX,
        line_length,
        params.max_path_deviation,
    )
    .ok()
    .and_then(|event_path| {
        event_path
            .build_regions(401, 96, 600)
            .ok()
            .map(|event_regions| (event_path, event_regions))
    });

    let mut canvas = Canvas::new();

    let original_view = View::new(construction_bounds(model), tops[0]);
    draw_hybrid_construction(&mut canvas, &original_view, model, tops[0]);

    let mut path_bounds = sweep_bounds(model);
    path_bounds.include(path.cut.cut_point);
    path_bounds.include(path.cut.tangent_point);
    for sample in &regions.seam {
        path_bounds.include(sample.point);
    }
    if let Some((event_path, event_regions)) = &event {
        path_bounds.include(event_path.cut.cut_point);
        path_bounds.include(event_path.cut.tangent_point);
        for sample in &event_regions.seam {
            path_bounds.include(sample.point);
        }
    }
    let path_view = View::new(path_bounds, tops[1]);
    let event_ref = event.as_ref().map(|(path, regions)| (path, regions));
    draw_adaptive_path(
        &mut canvas,
        &path_view,
        model,
        &path,
        &regions,
        event_ref,
        tops[1],
    );

    let mut region_bounds = adaptive_bounds(&regions);
    if let Some((_, event_regions)) = &event {
        for polygon in [
            &event_regions.line,
            &event_regions.straight,
            &event_regions.arc,
            &event_regions.expected,
        ] {
            for point in &polygon.points {
                region_bounds.include(*point);
            }
        }
    }
    let regions_view = View::new(region_bounds, tops[2]);
    draw_adaptive_regions(
        &mut canvas,
        &regions_view,
        model,
        &path,
        &regions,
        event_ref,
        tops[2],
    );

    for top in &tops[1..] {
        canvas.screen_line(
            (20.0, top - 20.0),
            (WIDTH - 20.0, top - 20.0),
            "#dddddd",
            1.0,
            "",
        );
    }
    let height = tops[2] + PANEL_HEIGHT + 30.0;
    canvas.finish_sized(
        &format!(
            "AOIM adaptive path, {:.0} deg",
            model.corner_angle_rad.to_degrees()
        ),
        height,
    )
}
