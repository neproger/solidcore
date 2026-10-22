//! Print adaptive-path cuts and region metrics.
//!
//! Run from `research/aoim-line-arc`:
//!
//! ```sh
//! cargo run --example adaptive
//! ```

use aoim_line_arc_research::adaptive::{AdaptiveError, AdaptiveParams, AdaptivePath};
use aoim_line_arc_research::LineArc;

fn main() {
    for degrees in [30.0_f64, 90.0, 150.0, 170.0, 175.0, 179.0] {
        let line_length = if degrees >= 175.0 {
            500.0
        } else if degrees >= 170.0 {
            100.0
        } else {
            20.0
        };
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        println!("\n=== alpha = {degrees:.1} deg, line_length={line_length} ===");
        for (name, max_miter, max_deviation) in [
            (
                "primary",
                if degrees < 40.0 { 4.14 } else { 6.5 },
                if degrees >= 170.0 {
                    3.5
                } else if degrees >= 150.0 {
                    1.5
                } else {
                    0.5
                },
            ),
            ("strict", 5.0, 0.5),
            ("very-strict", 4.5, 0.5),
        ] {
            let params = AdaptiveParams {
                max_miter_length: max_miter,
                max_path_deviation: max_deviation,
                preserve_endpoints: true,
                preserve_tangency: false,
            };
            match AdaptivePath::solve(model, -2.0, 2.0, line_length, params) {
                Ok(path) => {
                    let cut = path.cut;
                    match path.build_regions(401, 96, 600) {
                        Ok(regions) => {
                            let raster = regions.raster;
                            println!(
                                "  {name:7} theta={:8.3} A=({:8.3},{:5.2}) Q=({:8.3},{:8.3}) \
                                 removed={:6.2} rem={:6.2} miter={:7.3} extent={:7.3} dev={:6.3} \
                                 areas={:8.3}/{:7.3}/{:8.3}->{:9.3} bal={:+.1e} \
                                 overlap={:.1e} gap={:.1e} sym={:.1e} self={}",
                                cut.theta.to_degrees(),
                                cut.cut_point.x,
                                cut.cut_point.y,
                                cut.tangent_point.x,
                                cut.tangent_point.y,
                                cut.removed_arc_angle.to_degrees(),
                                cut.remaining_arc_angle.to_degrees(),
                                cut.miter_length,
                                cut.miter_extent,
                                cut.path_deviation,
                                regions.line.area(),
                                regions.straight.area(),
                                regions.arc.area(),
                                regions.expected.area(),
                                regions.area_balance_fraction,
                                raster.overlap_area / raster.expected_area,
                                raster.gap_area / raster.expected_area,
                                raster.symmetric_difference_fraction,
                                regions.self_intersecting,
                            );
                            if regions.self_intersecting {
                                println!(
                                    "      self pairs: line={:?} straight={:?} arc={:?} expected={:?}",
                                    regions.line.self_intersection_pairs(),
                                    regions.straight.self_intersection_pairs(),
                                    regions.arc.self_intersection_pairs(),
                                    regions.expected.self_intersection_pairs(),
                                );
                            }
                        }
                        Err(error) => println!("  {name:7} cut ok, regions failed: {error:?}"),
                    }
                }
                Err(AdaptiveError::NoCutNeeded { original_miter }) => {
                    println!("  {name:7} no cut needed (original miter={original_miter:.3})")
                }
                Err(AdaptiveError::NoFeasibleCut {
                    min_miter,
                    min_deviation,
                }) => println!(
                    "  {name:7} infeasible: min_miter={min_miter:.3}, min_deviation={min_deviation:.3}"
                ),
                Err(error) => println!("  {name:7} {error:?}"),
            }
        }
    }
}
