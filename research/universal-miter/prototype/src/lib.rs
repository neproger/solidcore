//! Isolated `PlanarSweep2` research prototype.
//!
//! This crate is outside the `SolidCore` workspace, so the production build
//! never sees it. The main task specification is
//! `../docs/16-planarsweep2-task.md`: `PlanarSweep2` analyses the flat path,
//! constructs a valid boundary between adjacent segments, and builds the B-rep
//! directly from those boundaries without Boolean, `ProjectAndTrim` or
//! `StitchFaces` forming the joints.
//!
//! Foundations kept from the earlier research:
//!
//! - **Level A** (always built): the planar curve/differential/rail mathematics.
//! - **Gate 0 input preprocessing** (`planar_sweep2`, feature `solidcore`):
//!   path/profile normalization and unification, reproduced from the production
//!   Planar Sweep where its helpers are private.
//! - **Level B surface builder** (`level_b_geometry`, feature `solidcore`):
//!   controlled NURBS swept surfaces with two-sided deviation checks.
//!
//! The independent Python validator and its immutable baseline in
//! `../prototype-python/` and `../results/reports/initial-validation.txt` are
//! preserved as the original mathematical reference.
//!
//! ## API boundary
//!
//! The validated high-level entry point is
//! [`planar_pipeline::build_planar_sweep`]. It runs source selection, boundary
//! solving, independent boundary validation, and only then the Direct builder.
//!
//! `boundary_validation` is the public independent acceptance gate. The modules
//! `direct_brep`, `boundary`, `exact_geometry`, `level_b_geometry`,
//! `planar_sweep2` and `source_selection` are prototype-level building blocks:
//! they are reachable from an external crate (this is a research crate), but
//! using them directly bypasses the high-level validation invariant.
//!
//! Rules for this sandbox are in `../AGENTS.md`. Production files and the public
//! API are not touched.
#![forbid(unsafe_code)]

pub mod curve;
pub mod differential_geometry;
pub mod harness;
pub mod miter_curve;
pub mod rail_solver;
pub mod validation;

#[cfg(feature = "solidcore")]
pub mod boundary;
#[cfg(feature = "solidcore")]
pub mod boundary_validation;
#[cfg(feature = "solidcore")]
pub mod direct_brep;
#[cfg(feature = "solidcore")]
pub mod exact_geometry;
#[cfg(feature = "solidcore")]
pub mod level_b_geometry;
#[cfg(feature = "solidcore")]
pub mod planar_pipeline;
#[cfg(feature = "solidcore")]
pub mod planar_sweep2;
#[cfg(feature = "solidcore")]
pub mod source_selection;
