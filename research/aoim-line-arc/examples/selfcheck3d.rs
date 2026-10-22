use aoim_line_arc_research::sweep::Strategy;
use aoim_line_arc_research::sweep3d::{build_pair, build_profile_pair, Sweep3Params};
use aoim_line_arc_research::{LineArc, Point2};

fn p(d: f64, z: f64) -> Point2 {
    Point2 { x: d, y: z }
}

fn params(line_length: f64) -> Sweep3Params {
    Sweep3Params {
        d0: -2.0,
        d1: 2.0,
        z0: -1.0,
        z1: 1.0,
        line_length,
        seam_samples: 401,
        arc_steps: 32,
    }
}

fn main() {
    for (degrees, line_length) in [
        (30.0_f64, 20.0_f64),
        (90.0, 20.0),
        (150.0, 20.0),
        (179.0, 500.0),
    ] {
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        let pair = build_pair(model, Strategy::HybridAtJunction, params(line_length)).unwrap();
        let hits = pair.self_intersections();
        println!("{degrees}: triangle face pairs={}", hits.len());
        for hit in hits.iter().take(5) {
            println!("   {hit:?}");
        }
    }
    let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
    let rectangle = [p(-2.0, -1.0), p(2.0, -1.0), p(2.0, 1.0), p(-2.0, 1.0)];
    let l_shape = [
        p(-2.0, -1.0),
        p(0.0, -1.0),
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(1.0, 1.0),
        p(-2.0, 1.0),
    ];
    let t_shape = [
        p(-1.0, -1.0),
        p(1.0, -1.0),
        p(1.0, 0.0),
        p(2.0, 0.0),
        p(2.0, 1.0),
        p(-2.0, 1.0),
        p(-2.0, 0.0),
        p(-1.0, 0.0),
    ];
    for (name, profile) in [
        ("rectangle", &rectangle[..]),
        ("L", &l_shape[..]),
        ("T", &t_shape[..]),
    ] {
        let sweep = build_profile_pair(model, profile, 20.0, 8, 32).unwrap();
        let hits = sweep.pair.self_intersections();
        println!("{name}: triangle face pairs={}", hits.len());
        if let Some(combined) = Some(sweep.pair.combined_body()) {
            for hit in hits.iter().take(3) {
                let fa = &combined.faces[hit.0];
                let fb = &combined.faces[hit.1];
                println!(
                    "   hit={hit:?} kinds={:?}/{:?} verts_a={:?} verts_b={:?}",
                    fa.kind, fb.kind, fa.vertices, fb.vertices
                );
                let ca: Vec<_> = fa
                    .vertices
                    .iter()
                    .map(|&i| sweep.pair.vertices[i])
                    .collect();
                let cb: Vec<_> = fb
                    .vertices
                    .iter()
                    .map(|&i| sweep.pair.vertices[i])
                    .collect();
                println!("      coords_a={ca:?}");
                println!("      coords_b={cb:?}");
            }
        }
    }
}
