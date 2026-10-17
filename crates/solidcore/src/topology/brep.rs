//! The B-rep object (`SmBrep`): construction from serialized data, editing and
//! queries, split into the units `topology.brep_p1` .. `topology.brep_p10`.
//!
//! Upstream: `source/SMLib/src/SmBrep.cpp`, `source/SMLib/inc/SmBrep.h`.
//!
//! | Rust      | Upstream                                                        |
//! |-----------|-----------------------------------------------------------------|
//! | [`p4`]    | `~SmBrep`, `DeleteEdge`, `DeleteVertex`, `StitchFaces`, `ShrinkGeometry` |
//!
//! Deviations: the B-rep records are the arenas of [`crate::topology::Brep`].

pub mod p4;
