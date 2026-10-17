//! Safe wrapper of the `topology.brep_p4` shim (`harness/cpp/shim/topology_brep_p4.cpp`):
//! `SmBrep::DeleteEdge`, `DeleteVertex`, `ShrinkGeometry`, `StitchFaces` and `~SmBrep`
//! on a B-rep built by the shared fixture.
//!
//! The fixture block is validated by the `topology.topology_traverser` export (its
//! `walk(brep, g, true)` order names the target edge, vertex and face); the target and
//! keep indices are range-checked against that export's counts before the C++ runs.
//! Output layout: see the shim header.

use super::topology_topology_traverser::{self as traverser, Fixture};
use super::topology_vertexuse::counts;
use crate::{call_with_buffer, FfiError};
use std::ffi::CString;

mod sys {
    extern "C" {
        pub fn sc_topology_brep_p4_eval(
            op: i32,
            path: *const std::ffi::c_char,
            input: *const f64,
            n_in: i64,
            editing: i32,
            target: i64,
            keep: i64,
            tol: f64,
            flag: i32,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
        ) -> i32;
    }
}

/// The operation run by [`eval`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    /// `DeleteEdge(edges[target], keep face's surface or NULL)`.
    DeleteEdge,
    /// `DeleteVertex(vertices[target])`.
    DeleteVertex,
    /// `ShrinkGeometry()`.
    ShrinkGeometry,
    /// `StitchFaces(tol, .., keep face or all, flag)`.
    StitchFaces,
    /// `delete brep`.
    Destroy,
    /// No operation: the records of the fixture B-rep as built (the Rust side checks
    /// its imported model against them).
    Check,
}

/// The request of [`eval`].
#[derive(Debug, Clone, Copy)]
pub struct Request {
    /// The operation.
    pub op: Op,
    /// `m_bEditingEnabled` before the operation.
    pub editing: bool,
    /// The target edge or vertex index (walk order).
    pub target: i64,
    /// The face index whose surface to keep / to stitch, or `-1`.
    pub keep: i64,
    /// `dXSectTol3d` of `StitchFaces`.
    pub tol: f64,
    /// `bAllowSpineEdges` of `StitchFaces`.
    pub flag: bool,
}

fn invalid(what: &str) -> FfiError {
    FfiError::InvalidArgument(format!("topology.brep_p4: {what}"))
}

/// Exports the fixture B-rep (validating its block), as the Rust side imports it.
pub fn export(path: &str, input: &[f64]) -> Result<Fixture, FfiError> {
    traverser::export(path, input)
}

/// Builds the fixture B-rep and runs the request on it.
pub fn eval(path: &str, input: &[f64], r: Request) -> Result<Vec<f64>, FfiError> {
    let fixture = export(path, input)?;
    let Some(c) = counts(&fixture.graph) else {
        // the fixture does not build: the shim reports it without running anything
        return Ok(vec![-1.0]);
    };
    match r.op {
        Op::DeleteEdge if r.target < 0 || r.target as u64 >= c[7] as u64 => {
            return Err(invalid("edge index"));
        }
        Op::DeleteVertex if r.target < 0 || r.target as u64 >= c[9] as u64 => {
            return Err(invalid("vertex index"));
        }
        _ => {}
    }
    if r.keep < -1 || (r.keep >= 0 && r.keep as u64 >= c[3] as u64) {
        return Err(invalid("face index"));
    }
    if !r.tol.is_finite() {
        return Err(invalid("tolerance"));
    }
    let op = match r.op {
        Op::DeleteEdge => 0,
        Op::DeleteVertex => 1,
        Op::ShrinkGeometry => 2,
        Op::StitchFaces => 3,
        Op::Destroy => 4,
        Op::Check => 5,
    };
    let cpath = CString::new(path).map_err(|_| invalid("path"))?;
    call_with_buffer(4096, |buf, len| {
        // SAFETY: the block was validated by the traverser export above, the indices are
        // in range of the B-rep it builds, and all buffers are owned here.
        unsafe {
            sys::sc_topology_brep_p4_eval(
                op,
                cpath.as_ptr(),
                input.as_ptr(),
                input.len() as i64,
                i32::from(r.editing),
                r.target,
                r.keep,
                r.tol,
                i32::from(r.flag),
                buf.as_mut_ptr(),
                buf.len() as i64,
                &raw mut *len,
            )
        }
    })
}
