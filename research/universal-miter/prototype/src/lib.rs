//! Isolated research prototype for the universal planar-sweep miter curve.
//!
//! This crate is **outside the `SolidCore` workspace** (it has its own
//! `Cargo.toml` at `research/universal-miter/`), so the production build never
//! sees it. It has two independent levels:
//!
//! - **level A** (always built): the mathematical prototype. It defines the
//!   curve data, the differential data, the rail system and the comparison
//!   harness without importing anything from the kernel. The algorithm is not
//!   implemented yet; `harness::run_case` returns
//!   [`MiterError::NotImplemented`](crate::harness::MiterError::NotImplemented).
//! - **level B** (`--features solidcore`): the integration prototype. It uses
//!   the real [`solidcore`] types; the module is `solidcore_api`.
//!
//! The independent Python validator in `prototype-python/` is the mathematical
//! reference and is not replaced by this crate.
//!
//! Rules for this sandbox are in `../AGENTS.md`. The production Sweep
//! (`crates/solidcore/src/primitive/primitive_creation/p2/planar_path.rs`) and
//! the public API are not touched.
#![forbid(unsafe_code)]

pub mod curve;
pub mod differential_geometry;
pub mod harness;
pub mod miter_curve;
pub mod rail_solver;
pub mod validation;

#[cfg(feature = "solidcore")]
pub mod level_b_geometry;
#[cfg(feature = "solidcore")]
pub mod planar_sweep2;
#[cfg(feature = "solidcore")]
pub mod solidcore_api;
