//! Queries on faces (`SmFace`): point and curve classification against the face
//! boundary. The methods are split over the plan units `query.face_p1` .. `_p4`.
//!
//! Upstream: `source/SMLib/src/SmFace.cpp`, `source/SMLib/inc/SmFace.h`.

pub mod p2;
pub mod p3;
pub mod p4;
