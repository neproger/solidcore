//! Side-by-side comparison of analytical, hybrid, and adaptive strategies.
//!
//! Run from `research/aoim-line-arc`:
//!
//! ```sh
//! cargo run --example compare
//! ```

use aoim_line_arc_research::adaptive::{AdaptiveError, AdaptiveParams, AdaptivePath};
use aoim_line_arc_research::sweep::{build_regions, RegionError, SeamSample, Strategy};
use aoim_line_arc_research::{LineArc, Point2};

fn seam_metrics(samples: &[SeamSample], corner: Point2) -> (f64, f64) {
    let chord = match (samples.first(), samples.last()) {
        (Some(first), Some(last)) => first.point.distance_to(last.point),
        _ => f64::NAN,
    };
    let extent = samples
        .iter()
        .map(|sample| sample.point.distance_to(corner))
        .fold(0.0_f64, f64::max);
    (chord, extent)
}

fn main() {
    for (degrees, line_length, max_miter, max_deviation) in [
        (30.0_f64, 20.0_f64, 4.14_f64, 0.5_f64),
        (90.0, 20.0, 5.0, 0.5),
        (150.0, 20.0, 6.5, 1.5),
        (170.0, 100.0, 6.5, 3.5),
        (175.0, 500.0, 6.5, 3.5),
        (179.0, 500.0, 6.5, 3.5),
    ] {
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        let origin = Point2 { x: 0.0, y: 0.0 };
        println!(
            "\n=== alpha={degrees:.0} deg, R=4, profile=[-2,2], line_length={line_length} ==="
        );

        match build_regions(
            &model,
            Strategy::Analytical,
            -2.0,
            2.0,
            line_length,
            401,
            96,
        ) {
            Ok(built) => {
                let (chord, extent) = seam_metrics(&built.seam, origin);
                println!(
                    "analytical: closed, expected={:.3}, miter chord={:.3} extent={:.3}, \
                     overlap={:.2e} gap={:.2e} sym={:.2e} self={}",
                    built.expected.area(),
                    chord,
                    extent,
                    built.raster.overlap_area / built.raster.expected_area,
                    built.raster.gap_area / built.raster.expected_area,
                    built.raster.symmetric_difference_fraction,
                    built.self_intersecting,
                );
            }
            Err(RegionError::UncoveredProfile { first_missing }) => {
                println!("analytical: uncovered (first missing d={first_missing:.4})");
            }
            Err(error) => println!("analytical: failed ({error:?})"),
        }

        let hybrid = build_regions(
            &model,
            Strategy::HybridAtJunction,
            -2.0,
            2.0,
            line_length,
            401,
            96,
        )
        .unwrap();
        let (hybrid_chord, hybrid_extent) = seam_metrics(&hybrid.seam, origin);
        let hybrid_area = hybrid.expected.area();
        println!(
            "hybrid:     closed, expected={:.3}, miter chord={:.3} extent={:.3}, \
             overlap={:.2e} gap={:.2e} sym={:.2e} self={}",
            hybrid_area,
            hybrid_chord,
            hybrid_extent,
            hybrid.raster.overlap_area / hybrid.raster.expected_area,
            hybrid.raster.gap_area / hybrid.raster.expected_area,
            hybrid.raster.symmetric_difference_fraction,
            hybrid.self_intersecting,
        );

        let request = AdaptiveParams {
            max_miter_length: max_miter,
            max_path_deviation: max_deviation,
            preserve_endpoints: true,
            preserve_tangency: false,
        };
        match AdaptivePath::solve(model, -2.0, 2.0, line_length, request) {
            Ok(path) => match path.build_regions(401, 96, 600) {
                Ok(regions) => {
                    let (adaptive_chord, adaptive_extent) =
                        seam_metrics(&regions.seam, path.cut.cut_point);
                    let area_difference = regions.expected.area() - hybrid_area;
                    println!(
                        "adaptive:   theta={:.3} deg, A=({:.3},{:.3}), Q=({:.3},{:.3}), \
                         removed={:.2} deg, miter chord={:.3} extent={:.3}, dev={:.3}, \
                         expected={:.3} (delta={:+.3}), overlap={:.2e} gap={:.2e} sym={:.2e} self={}",
                        path.cut.theta.to_degrees(),
                        path.cut.cut_point.x,
                        path.cut.cut_point.y,
                        path.cut.tangent_point.x,
                        path.cut.tangent_point.y,
                        path.cut.removed_arc_angle.to_degrees(),
                        adaptive_chord,
                        adaptive_extent,
                        path.cut.path_deviation,
                        regions.expected.area(),
                        area_difference,
                        regions.raster.overlap_area / regions.raster.expected_area,
                        regions.raster.gap_area / regions.raster.expected_area,
                        regions.raster.symmetric_difference_fraction,
                        regions.self_intersecting,
                    );
                }
                Err(error) => println!("adaptive:   cut ok, regions failed ({error:?})"),
            },
            Err(AdaptiveError::NoCutNeeded { original_miter }) => println!(
                "adaptive:   no cut needed (original miter={original_miter:.3})"
            ),
            Err(AdaptiveError::NoFeasibleCut {
                min_miter,
                min_deviation,
            }) => println!(
                "adaptive:   infeasible (min_miter={min_miter:.3}, min_deviation={min_deviation:.3})"
            ),
            Err(error) => println!("adaptive:   {error:?}"),
        }
    }
}
