//! B-rep operations, split into independently owned port units.
//!
//! Upstream: `source/SMLib/inc/SmBrep.h`, `source/SMLib/src/SmBrep.cpp`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`p5`] | B-rep editing and transformation operations, part 5 |
//!
//! Deviations: the arena record remains [`super::Brep`]; no dependency unit is co-ported.

pub mod p5;
