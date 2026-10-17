//! FFI wrappers for unit `topology.brep_p2`.
//!
//! Shim: `harness/cpp/shim/topology_brep_p2.cpp` (`sc_topology_brep_p2_eval`).
//! Upstream: `source/SMLib/src/SmBrep.cpp`.
//!
//! Every input is a B-rep block of [`super::topology_vertexuse`] (kinds `0..=3`), then
//! `editing` (`-1` keep, `0` / `1` set `m_bEditingEnabled`), then the op's parameters.
//! Indices name entities in the export's traversal order. Every output starts with
//! `0`, or `-1 status` when the B-rep could not be built (nothing else follows). A
//! curve block is [`super::topology_vertexuse`]'s (`dim n p m cpts knots`), a surface
//! block too (`0 ..` B-spline, `1 O N` plane). An entity reference is `type index`
//! with an `SmTopoTypes.h` code (`-1 -1` null, `16011 0` the B-rep).
//!
//! | [`Op`]     | parameters | outputs |
//! |------------|------------|---------|
//! | `Export`   |            | the graph in [`super::topology_vertexuse`]'s export layout (faces, edges and vertices registered in B-rep list order first, the walk including the vertex lists) |
//! | `Extras`   |            | `tol infinite_region region_list_head editing`, per vertex `n vu*`, per shell its `m_tShellType` |
//! | `Query`    | `which`    | `0`: `n eu*` (`GetEdgeuses`), `1`: `n v*` (`GetShellVertices`), `2` `IsManifoldSolid`, `3` `IsSheet`, `4` `IsOrientedSheet`, `5`: `size edge` (`GetMinEdgeBBoxSize`) |
//! | `SetTol`   | `tol only_larger cascade` | `brep_tol`, every face, edge and vertex tolerance |
//! | `Rebuild`  |            | `status`, per edge use whether it has a UV trim curve, every edge tolerance |
//! | `Make`     | `which region v1 v2` (`0` `MakeShellVertexTopology`, `1` `MakeEdgeVertexTopology`, `2` `MakeEdgeTopology`; region `-1` null) | `status` |
//! | `Sector`   | `curve t0 t1 orient class type index t u v` | `status region` (`LocalCurveSector`) |
//! | `Merge`    | `face tol n curve*` | `status new_faces edges` (`MergeCurvesOnSurface`) |
//! | `Mate`     | `edge surface ext_factor uv_factor` | `status` (`MateLaminaEdgeToSurface`) |
//! | `Notify`   | `operation ref ref ref` | `0` |
//! | `Stale`    | `create modified` (face `i` gets an `SM_OC_TRIMSRF` cache when bit `i % 8` of `create` is set, `m_bFaceWasModified` from that bit of `modified`) | `status`, per face whether its cache survived (`DeleteStaleCaches`) |
//! | `Swap`     |            | `status` (`SwapUV`) |
//! | `Replace`  | `face surface keep create` (`keep`: `0` delete, `1` deleted list, `2` keep) | `status modified` (`ReplaceSurface`) |
//! | `RayFire`  | `P V tol has_limit limit` | `status region class` (`RayFire`) |
//! | `MateProbe` | as `Mate` | `status max_dist`: `MateLaminaEdgeToSurface`'s `dMaxDist` (`SmBrep.cpp:22200-22229`), nothing changed |
//!
//! [`eval`] accepts only what upstream handles without undefined behaviour: valid
//! B-rep blocks, indices inside the B-rep (it exports the B-rep once more to count),
//! object references whose type matches the point class for the face, edge and vertex
//! classes (`Sector`; any object for the other classes, which upstream rejects with
//! `SM_ERR` untouched), a null region only when the B-rep has an infinite region
//! (`Make`), a start vertex for the edge makers, finite values. Deliberate
//! restrictions: a `Sector` interval must have `t0 < t1` (an empty one makes the step
//! `0 / 0`, a NaN parameter for the curve evaluator); a `Mate` extension factor must not
//! be negative (a negative extension distance hangs the C++ in `N_SrfExtendByDist`,
//! `plan/upstream-issues.md` HANG-18), and a `Mate` is refused when the `MateProbe`
//! distance is at least `SM_EFF_ZERO` and its product with the factor is not finite
//! (an edge whose squared distance to the mating surface overflows: `dMaxDist` is
//! infinite and `CreateExtendedSurface` never stops halving, HANG-9); a reference `type -1` is not accepted (the null
//! reference is `-1 -1`). `Merge` and `Mate` with editing enabled run the whole upstream
//! algorithm: the callers keep their curves short of the tolerance (`Merge`) and pick
//! lamina edges of upstream files (`Mate`).

use crate::{call_with_buffer, FfiError};

use super::topology_vertexuse::{self as vu, Reader};

/// Largest number of curves of a `Merge`.
pub const MAX_CURVES: usize = 8;

/// Shim operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Op {
    Export = 0,
    Extras = 1,
    Query = 2,
    SetTol = 3,
    Rebuild = 4,
    Make = 5,
    Sector = 6,
    Merge = 7,
    Mate = 8,
    Notify = 9,
    Stale = 10,
    Swap = 11,
    Replace = 12,
    RayFire = 13,
    MateProbe = 14,
}

mod sys {
    extern "C" {
        pub fn sc_topology_brep_p2_eval(
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

/// `SmTopoTypes.h` codes with their export slot.
const TYPES: [(i64, usize); 10] = [
    (16012, 0), // SmRegion_TYPE
    (16013, 1), // SmShell_TYPE
    (16022, 2), // SmFaceuse_TYPE
    (16002, 3), // SmFace_TYPE
    (16004, 4), // SmLoopuse_TYPE
    (16003, 5), // SmLoop_TYPE
    (16005, 6), // SmEdgeuse_TYPE
    (16016, 7), // SmEdge_TYPE
    (16006, 8), // SmVertexuse_TYPE
    (16021, 9), // SmVertex_TYPE
];
/// `SmBrep_TYPE`.
const BREP_TYPE: f64 = 16011.0;
/// `SM_PC_FACE`, `SM_PC_EDGE`, `SM_PC_VERTEX` with the type their object must have.
const CLASS_TYPES: [(f64, f64); 3] = [(102.0, 16002.0), (103.0, 16016.0), (104.0, 16021.0)];

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

/// An entity reference `type index`; returns the type (`-1` null).
fn entity(rd: &mut Reader<'_>, c: &[usize; 10]) -> Result<f64, FfiError> {
    let t = rd.num()?;
    if t == -1.0 {
        int(rd, -1, -1)?;
        return Ok(t);
    }
    if t == BREP_TYPE {
        int(rd, 0, 0)?;
        return Ok(t);
    }
    let &(_, slot) = TYPES
        .iter()
        .find(|(code, _)| *code as f64 == t)
        .ok_or_else(|| bad("bad entity type"))?;
    rd.below(c.get(slot).copied().unwrap_or(0))?;
    Ok(t)
}

fn args(op: Op, rd: &mut Reader<'_>, c: &[usize; 10], has_infinite: bool) -> Result<(), FfiError> {
    let [n_r, _, _, n_f, _, _, _, n_e, _, n_v] = *c;
    match op {
        Op::Export | Op::Extras | Op::Rebuild | Op::Swap => {}
        Op::Stale => {
            int(rd, 0, 255)?;
            int(rd, 0, 255)?;
        }
        Op::Query => {
            rd.index(5)?;
        }
        Op::SetTol => {
            rd.finite()?;
            rd.flag()?;
            rd.flag()?;
        }
        Op::Make => {
            let which = rd.index(2)?;
            if opt_below(rd, n_r)?.is_none() && !has_infinite {
                return Err(bad("null region without infinite region"));
            }
            let v1 = opt_below(rd, n_v)?;
            let v2 = opt_below(rd, n_v)?;
            if which == 2 && (v1.is_none() || v2.is_none()) || which == 1 && v1.is_none() {
                return Err(bad("missing vertex"));
            }
        }
        Op::Sector => {
            if rd.index(3)? != 3 {
                return Err(bad("sector curve must be 3D"));
            }
            rd.at -= 1;
            vu::curve(rd)?;
            let t0 = rd.finite()?;
            let t1 = rd.finite()?;
            if t0 >= t1 {
                return Err(bad("empty sector interval"));
            }
            rd.index(2)?;
            let class = rd.num()?;
            if class.fract() != 0.0 || !(100.0..=110.0).contains(&class) {
                return Err(bad("bad point class"));
            }
            let ty = entity(rd, c)?;
            // only the face / edge / vertex arms cast the object; every other class is
            // upstream's `default: SER(SM_ERR)` and never touches it
            if let Some(&(_, want)) = CLASS_TYPES.iter().find(|&&(k, _)| k == class) {
                if ty != want {
                    return Err(bad("point class and object type disagree"));
                }
            }
            rd.finites(3)?;
        }
        Op::Merge => {
            rd.below(n_f)?;
            rd.finite()?;
            let n = rd.index(MAX_CURVES)?;
            for _ in 0..n {
                if rd.index(3)? != 3 {
                    return Err(bad("merge curves must be 3D"));
                }
                rd.at -= 1;
                vu::curve(rd)?;
            }
        }
        Op::Mate | Op::MateProbe => {
            rd.below(n_e)?;
            vu::surface(rd)?;
            // a negative factor is a negative extension distance for
            // CreateExtendedSurface, which never returns (N_SrfExtendByDist; HANG-18)
            if rd.finite()? < 0.0 {
                return Err(bad("negative surface extension factor"));
            }
            rd.finite()?;
        }
        Op::Notify => {
            rd.index(16)?;
            for _ in 0..3 {
                entity(rd, c)?;
            }
        }
        Op::Replace => {
            rd.below(n_f)?;
            vu::surface(rd)?;
            rd.index(2)?;
            rd.flag()?;
        }
        Op::RayFire => {
            rd.vec3()?;
            rd.finites(3)?;
            rd.finite()?;
            rd.flag()?;
            rd.finite()?;
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
            sys::sc_topology_brep_p2_eval(
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
    vu::brep_block(&mut rd, path)?;
    int(&mut rd, -1, 1)?;
    let head = input.get(..rd.at).ok_or_else(|| bad("input too short"))?;
    let export = raw(Op::Export, path, head)?;
    if op == Op::Export {
        rd.done()?;
        return Ok(export);
    }
    let Some(c) = vu::counts(&export) else {
        // the B-rep could not be built: every op reports the same failure
        return Ok(export);
    };
    let extras = raw(Op::Extras, path, head)?;
    let has_infinite = extras.get(2).is_some_and(|&r| r >= 0.0);
    args(op, &mut rd, &c, has_infinite)?;
    rd.done()?;
    if op == Op::Mate {
        mate_guard(path, input)?;
    }
    raw(op, path, input)
}

/// Refuses a `Mate` whose extension distance `dMaxDist * dSurfaceExtensionFactor`
/// would not be finite (upstream returns before extending when `dMaxDist <
/// SM_EFF_ZERO` or the distance computation fails, so those run).
fn mate_guard(path: &str, input: &[f64]) -> Result<(), FfiError> {
    let probe = raw(Op::MateProbe, path, input)?;
    let (Some(&status), Some(&max_dist)) = (probe.get(1), probe.get(2)) else {
        return Err(bad("short MateProbe output"));
    };
    let factor = input
        .get(input.len().wrapping_sub(2))
        .copied()
        .ok_or_else(|| bad("input too short"))?;
    if status == 0.0 && max_dist >= 1e-12 && !(max_dist * factor).is_finite() {
        return Err(bad("non-finite surface extension distance"));
    }
    Ok(())
}
