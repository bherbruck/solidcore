//! Shell member kinds: what an `SmShell`'s list holds (minimal co-port).
//!
//! Upstream: `source/SMLib/inc/SmShell.h`.
//!
//! | Rust            | Upstream                                  |
//! |-----------------|-------------------------------------------|
//! | [`ShellType`]   | `SmShell::m_tShellType`                   |
//! | [`ShellMembers`]| the typed contents of `SmShell::m_pList`  |
//!
//! Deviations: CO-PORTED by `topology.brep_p10` (the shell type and list head that
//! `SmBrep::MakeTopologyFromData` writes); the `SmShell` methods are unit
//! `topology.shell`. The circular face use list is a vector in list order from its
//! head.

use crate::topology::{EdgeuseId, FaceuseId, VertexuseId};

/// The native shell member class, stored independently of the list head.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ShellType {
    /// `SmUnknown_TYPE` (16999), the constructor value.
    #[default]
    Unknown,
    /// `SmVertexuse_TYPE` (16006).
    Vertex,
    /// `SmEdgeuse_TYPE` (16005).
    Wire,
    /// `SmFaceuse_TYPE` (16022).
    Faceuse,
    /// Other values preserved by `SetShellType` without reinterpretation.
    Other(i64),
}

impl ShellType {
    /// The exact native type code.
    #[must_use]
    pub const fn code(self) -> i64 {
        match self {
            Self::Unknown => 16999,
            Self::Vertex => 16006,
            Self::Wire => 16005,
            Self::Faceuse => 16022,
            Self::Other(code) => code,
        }
    }

    /// The shell type of a native type code.
    #[must_use]
    pub const fn from_code(code: i64) -> Self {
        match code {
            16999 => Self::Unknown,
            16006 => Self::Vertex,
            16005 => Self::Wire,
            16022 => Self::Faceuse,
            other => Self::Other(other),
        }
    }
}

/// A checked representation of the shell's native list head and members.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum ShellMembers {
    /// NULL list head.
    #[default]
    Empty,
    /// Face-use circular list, from its current head.
    Faceuses(Vec<FaceuseId>),
    /// First wire edge use (the edge owns its radial list).
    Edgeuse(EdgeuseId),
    /// Vertex use of a vertex shell.
    Vertexuse(VertexuseId),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_round_trip() {
        for t in [
            ShellType::Unknown,
            ShellType::Vertex,
            ShellType::Wire,
            ShellType::Faceuse,
            ShellType::Other(7),
        ] {
            assert_eq!(ShellType::from_code(t.code()), t);
        }
        assert_eq!(ShellMembers::default(), ShellMembers::Empty);
    }
}
