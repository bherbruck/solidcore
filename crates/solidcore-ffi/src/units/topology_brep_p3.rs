//! FFI wrappers for unit `topology.brep_p3`.
//!
//! Shim: `harness/cpp/shim/topology_brep_p3.cpp` (`sc_topology_brep_p3_eval`).
//! Upstream: `source/SMLib/src/SmBrep.cpp`, `source/SM_API/src/SmApiQueries.cpp`.
//!
//! Except for `Construct` and `ApiCopyNull`, every input is a B-rep block of
//! [`super::topology_vertexuse`] (kinds `0..=3`), then `editing` (`-1` keep, `0` / `1`
//! set `m_bEditingEnabled`), then the op's parameters: the head of
//! [`super::topology_brep_p2`], whose shim builds (and normalises) the same B-rep, so
//! its `Export` / `Extras` describe the B-rep these ops run on. Indices name entities
//! in that export's order. Every output starts with `0`, or `-1 status` when the B-rep
//! could not be built. `counts` is `faces edges vertices regions` of the B-rep after
//! the op (a fresh walk).
//!
//! | [`Op`]          | parameters | outputs |
//! |-----------------|------------|---------|
//! | `Construct`     | `mse lsr` (no B-rep block) | `0 0 tol mse lsr editing infinite_void counts` (`SmBrep(mse, lsr)` in a context of its own) |
//! | `Copy`          | `analytics api` | `status tol editing counts` of the copy (`api`: `SmApiBrepCopy`, else the copy constructor with `analytics`) |
//! | `Merge`         | `analytics` | `status counts` (`MergeBrep` of the B-rep into a second build of it) |
//! | `CopyVertices`  | `n v*` | `status new counts` (`CopyVertices` into a second build) |
//! | `DeleteFace`    | `face delete_ev nesting keep_region` (`-1` null) | `status deleted counts` |
//! | `Wire`          | `which region v1 v2 curve t0 t1 orient end(3)` (`0` `MakeWireEdge`, `1` `MakeWireEdgeVertex`; region `-1` null) | `status counts` |
//! | `Orient`        | `outward unify n face*` (`n = -1`: no list) | `status maybe_not_closed`, per face `u0 v0 u1 v1 upward_faceuse` |
//! | `Squeeze`       | `edge which v` (survivor: `0` `GetVertex`, `1` the other vertex, `2` vertex `v`) | `status counts` |
//! | `TurnNurbs`     |            | `status`, per edge its curve block (the fixture's `put_curve`), per face its surface type |
//! | `ApiCopyNull`   | `0` (no B-rep block) | `0 status` (`SmApiBrepCopy(NULL, ..)`) |
//!
//! [`eval`] accepts only what upstream handles without undefined behaviour: valid
//! B-rep blocks, indices inside the B-rep (it exports the B-rep once more to count),
//! a null region only when the B-rep has an infinite region (`Wire`), the vertices
//! the wire makers dereference, a 3D wire curve over a sub-interval of its range
//! (`t0 < t1`), finite values no larger than [`MAX_GEOMETRY`] (larger geometry hangs
//! the C++ in `SmCurve::GlobalCoincidenceChecker`, HANG-16).

use crate::{call_with_buffer, FfiError};

use super::topology_brep_p2 as p2;
use super::topology_vertexuse::{self as vu, Reader};

/// Largest magnitude of any input value (HANG-16).
pub const MAX_GEOMETRY: f64 = 1.0e150;

/// Largest vertex list of a `CopyVertices` or face list of an `Orient`.
pub const MAX_LIST: usize = 32;

/// Shim operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Op {
    Construct = 2,
    Copy = 3,
    Merge = 4,
    CopyVertices = 5,
    DeleteFace = 6,
    Wire = 7,
    Orient = 8,
    Squeeze = 9,
    TurnNurbs = 10,
    ApiCopyNull = 11,
}

mod sys {
    extern "C" {
        pub fn sc_topology_brep_p3_eval(
            op: i32,
            path: *const std::ffi::c_char,
            input: *const f64,
            n_in: i64,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
        ) -> i32;
    }
}

fn bad(msg: impl Into<String>) -> FfiError {
    FfiError::InvalidArgument(msg.into())
}

fn int(rd: &mut Reader<'_>, lo: i64, hi: i64) -> Result<i64, FfiError> {
    let v = rd.num()?;
    if v.fract() != 0.0 || v < lo as f64 || v > hi as f64 {
        return Err(bad(format!("bad integer {v} ({lo}..={hi})")));
    }
    Ok(v as i64)
}

/// `-1` or an index below `n`.
fn opt_below(rd: &mut Reader<'_>, n: usize) -> Result<Option<usize>, FfiError> {
    let v = int(rd, -1, i64::from(u32::MAX))?;
    if v < 0 {
        return Ok(None);
    }
    let v = v as usize;
    if v >= n {
        return Err(bad(format!("index {v} out of range ({n})")));
    }
    Ok(Some(v))
}

fn args(op: Op, rd: &mut Reader<'_>, c: &[usize; 10], has_infinite: bool) -> Result<(), FfiError> {
    let [n_r, _, _, n_f, _, _, _, n_e, _, n_v] = *c;
    match op {
        Op::Construct | Op::ApiCopyNull | Op::TurnNurbs => {}
        Op::Copy => {
            rd.flag()?;
            rd.flag()?;
        }
        Op::Merge => {
            rd.flag()?;
        }
        Op::CopyVertices => {
            let n = rd.index(MAX_LIST)?;
            for _ in 0..n {
                rd.below(n_v)?;
            }
        }
        Op::DeleteFace => {
            rd.below(n_f)?;
            rd.flag()?;
            rd.flag()?;
            opt_below(rd, n_r)?;
        }
        Op::Wire => {
            let which = rd.index(1)?;
            if opt_below(rd, n_r)?.is_none() && !has_infinite {
                return Err(bad("null region without infinite region"));
            }
            rd.below(n_v)?;
            let v2 = opt_below(rd, n_v)?;
            if which == 0 && v2.is_none() {
                return Err(bad("MakeWireEdge needs an end vertex"));
            }
            if rd.index(3)? != 3 {
                return Err(bad("wire curve must be 3D"));
            }
            rd.at -= 1;
            let (a, b) = vu::curve(rd)?;
            let t0 = rd.finite()?;
            let t1 = rd.finite()?;
            if !(a <= t0 && t0 < t1 && t1 <= b) {
                return Err(bad("wire interval outside the curve's"));
            }
            rd.index(2)?;
            rd.vec3()?;
        }
        Op::Orient => {
            rd.flag()?;
            rd.flag()?;
            let n = int(rd, -1, MAX_LIST as i64)?;
            for _ in 0..n.max(0) {
                rd.below(n_f)?;
            }
        }
        Op::Squeeze => {
            rd.below(n_e)?;
            rd.index(2)?;
            opt_below(rd, n_v)?;
        }
    }
    Ok(())
}

fn raw(op: Op, path: &str, input: &[f64]) -> Result<Vec<f64>, FfiError> {
    let cpath = std::ffi::CString::new(path).map_err(|_| bad("path has a NUL byte"))?;
    call_with_buffer(4096, |buf, len| {
        // SAFETY: cpath is a valid NUL-terminated string; input is valid for
        // input.len() reads and holds what the shim reads for `op` (validated by
        // `eval`); buf is valid for buf.len() writes and the shim honours the
        // capacity; len is valid.
        unsafe {
            sys::sc_topology_brep_p3_eval(
                op as i32,
                cpath.as_ptr(),
                input.as_ptr(),
                input.len() as i64,
                buf.as_mut_ptr(),
                buf.len() as i64,
                &raw mut *len,
            )
        }
    })
}

/// Runs `op` on `input` (see the module docs); `path` is the `.smb` file of a file
/// B-rep block (ignored otherwise).
///
/// # Errors
///
/// [`FfiError::InvalidArgument`] for a malformed input; [`FfiError::Status`] for a shim
/// failure.
pub fn eval(op: Op, path: &str, input: &[f64]) -> Result<Vec<f64>, FfiError> {
    let mut rd = Reader { input, at: 0 };
    match op {
        Op::Construct => {
            rd.finite()?;
            rd.finite()?;
            rd.done()?;
            return raw(op, path, input);
        }
        Op::ApiCopyNull => {
            int(&mut rd, 0, 0)?;
            rd.done()?;
            return raw(op, path, input);
        }
        _ => {}
    }
    vu::brep_block(&mut rd, path)?;
    int(&mut rd, -1, 1)?;
    let head = input.get(..rd.at).ok_or_else(|| bad("input too short"))?;
    // geometry beyond 1e150 overflows the squared distances of
    // SmCurve::GlobalCoincidenceChecker, reached by the UV trim curve builds of
    // OrientTrimmedSurfaces / TurnToNURBS, which then never returns (HANG-16, the
    // bound blend.fillet_geom_p1 uses; main's HANG-10)
    // (`-f64::MAX` is the non-rational weight `NOW`)
    if input
        .iter()
        .any(|&x| x.is_finite() && x.abs() > MAX_GEOMETRY && x != -f64::MAX)
    {
        return Err(bad("geometry value beyond 1e150 (HANG-16)"));
    }
    let export = p2::eval(p2::Op::Export, path, head)?;
    let Some(c) = vu::counts(&export) else {
        // the B-rep could not be built: every op reports the same failure
        return Ok(export);
    };
    let extras = p2::eval(p2::Op::Extras, path, head)?;
    let has_infinite = extras.get(2).is_some_and(|&r| r >= 0.0);
    args(op, &mut rd, &c, has_infinite)?;
    rd.done()?;
    raw(op, path, input)
}
