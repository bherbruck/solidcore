//! Runner for unit `topology.brep_p3`. C++: `solidcore_ffi::units::topology_brep_p3`.
//! Rust: `solidcore::topology::brep::p3`.
//!
//! The C++ side builds a real `SmBrep` per call with the shared topology fixture (an
//! upstream `.smb` file, a wire edge, a one-face B-rep or the `my_test_topology` wire
//! network), normalised as the `topology.brep_p2` shim does, and runs one member of the
//! unit (the constructors and `SmApiBrepCopy` too). The Rust side imports the graph
//! the `topology.brep_p2` shim exports for the same input (the `topology.vertexuse`
//! importer), restores what the import cannot carry (the B-rep tolerance, infinite
//! region, region list head, editing flag, each vertex's vertex use order, the
//! geometry owners and each face's upward face use), runs the port and writes the
//! same output layout (`out`).
//!
//! Entity selectors are taken modulo the matching entity count (`-1` selects null);
//! an op that needs an entity the B-rep lacks is skipped on both sides (`no target`).
//! `Merge` / `CopyVertices` target a second build of the same B-rep (a second import
//! on the Rust side).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use solidcore::curve::curve::{Curve as SmCurve, CurveKind};
use solidcore::error::SM_ERR;
use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::extent1d::Extent1d;
use solidcore::geom::object::ObjectRef;
use solidcore::geom::vector3d::Vec3;
use solidcore::kernel::Kernel;
use solidcore::nurbs::b_spline_curve::BSplineCurve;
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::nurbs::crv_basic::Curve as NlCurve;
use solidcore::nurbs::math::CPoint;
use solidcore::topology::brep::p1 as bp1;
use solidcore::topology::brep::p3 as bp3;
use solidcore::topology::brep_cache::{arena_identity, CacheObject};
use solidcore::topology::edge::NotifyEnv;
use solidcore::topology::topology_traverser::Marks;
use solidcore::topology::{Brep, EdgeId, FaceId, FaceuseId, RegionId, VertexId, VertexuseId};
use solidcore::{EntityRef, SolidError};
use solidcore_ffi::units::topology_brep_p2 as ffi2;
use solidcore_ffi::units::topology_brep_p3::{self as ffi, Op};
use solidcore_ffi::units::topology_vertexuse::counts;

use super::topology_vertexuse::{
    gen_brep, gen_curve, import, path_of, push_curve, push_surface, BrepIn, CrvIn,
};
use crate::case::CaseContext;
use crate::runner::default_cases;
use crate::{Case, HarnessError, Output, Rng, UnitDiff};

/// The difftest runner.
pub struct Runner;

fn harness(msg: impl Into<String>) -> HarnessError {
    HarnessError::new(msg)
}

// ---- inputs -----------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum OpIn {
    /// `SmBrep(mse, lsr)` (no fixture B-rep is used).
    Construct {
        mse: f64,
        lsr: f64,
    },
    /// The copy constructor (`api`: `SmApiBrepCopy`).
    Copy {
        analytics: bool,
        api: bool,
    },
    Merge {
        analytics: bool,
    },
    CopyVertices {
        vertices: Vec<i64>,
    },
    DeleteFace {
        face: i64,
        delete_ev: bool,
        nesting: bool,
        keep: i64,
    },
    /// `which`: `0` `MakeWireEdge`, `1` `MakeWireEdgeVertex`; `orient` an
    /// `SmOrientType` code.
    Wire {
        which: u8,
        region: i64,
        v1: i64,
        v2: i64,
        curve: CrvIn,
        t0: f64,
        t1: f64,
        orient: u8,
        end: [f64; 3],
    },
    /// `faces`: `None` for no list.
    Orient {
        outward: bool,
        unify: bool,
        faces: Option<Vec<i64>>,
    },
    /// `which`: `0` the edge's `GetVertex`, `1` the other vertex, `2` vertex `v`.
    Squeeze {
        edge: i64,
        which: u8,
        v: i64,
    },
    TurnNurbs,
    ApiCopyNull,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Inputs {
    #[serde(default = "network")]
    brep: BrepIn,
    #[serde(default)]
    validate_only: bool,
    #[serde(default)]
    init_gaps: bool,
    /// `-1` keep the built B-rep's `m_bEditingEnabled`, else set it.
    #[serde(default = "keep")]
    editing: i64,
    #[serde(flatten)]
    op: OpIn,
}

const fn network() -> BrepIn {
    BrepIn::Network
}

const fn keep() -> i64 {
    -1
}

fn flag(b: bool) -> f64 {
    f64::from(u8::from(b))
}

fn head(i: &Inputs) -> Result<Vec<f64>, HarnessError> {
    let mut v = Vec::new();
    match &i.brep {
        BrepIn::File { rebuild, .. } => v.extend([0.0, flag(*rebuild)]),
        BrepIn::Wire { curve, t0, t1 } => {
            v.push(1.0);
            push_curve(&mut v, curve)?;
            v.extend([*t0, *t1]);
        }
        BrepIn::Face { surface, dom } => {
            v.push(2.0);
            push_surface(&mut v, surface)?;
            v.extend(dom);
        }
        BrepIn::Network => v.push(3.0),
    }
    v.push(flag(i.validate_only));
    v.push(flag(i.init_gaps));
    v.push(i.editing as f64);
    Ok(v)
}

/// Selector `s` modulo `n`; `None` for a negative selector or an empty kind.
fn pick(s: i64, n: usize) -> Option<usize> {
    if s < 0 || n == 0 {
        return None;
    }
    Some((s as u64 % n as u64) as usize)
}

fn idx(x: Option<usize>) -> f64 {
    x.map_or(-1.0, |i| i as f64)
}

/// The op's arguments after the head, with selectors resolved against the counts;
/// `None` when the op needs an entity the B-rep lacks.
fn op_args(op: &OpIn, c: &[usize; 10]) -> Result<Option<(Op, Vec<f64>)>, HarnessError> {
    let [n_r, _, _, n_f, _, _, _, n_e, _, n_v] = *c;
    Ok(Some(match op {
        OpIn::Construct { mse, lsr } => (Op::Construct, vec![*mse, *lsr]),
        OpIn::ApiCopyNull => (Op::ApiCopyNull, vec![0.0]),
        OpIn::TurnNurbs => (Op::TurnNurbs, vec![]),
        OpIn::Copy { analytics, api } => (Op::Copy, vec![flag(*analytics), flag(*api)]),
        OpIn::Merge { analytics } => (Op::Merge, vec![flag(*analytics)]),
        OpIn::CopyVertices { vertices } => {
            let mut v = vec![vertices.len() as f64];
            for &s in vertices {
                let Some(x) = pick(s, n_v) else {
                    return Ok(None);
                };
                v.push(x as f64);
            }
            (Op::CopyVertices, v)
        }
        OpIn::DeleteFace {
            face,
            delete_ev,
            nesting,
            keep,
        } => {
            let Some(f) = pick(*face, n_f) else {
                return Ok(None);
            };
            (
                Op::DeleteFace,
                vec![
                    f as f64,
                    flag(*delete_ev),
                    flag(*nesting),
                    idx(pick(*keep, n_r)),
                ],
            )
        }
        OpIn::Wire {
            which,
            region,
            v1,
            v2,
            curve,
            t0,
            t1,
            orient,
            end,
        } => {
            let (Some(a), b) = (pick(*v1, n_v), pick(*v2, n_v)) else {
                return Ok(None);
            };
            if *which == 0 && b.is_none() {
                return Ok(None);
            }
            let mut v = vec![f64::from(*which), idx(pick(*region, n_r)), a as f64, idx(b)];
            push_curve(&mut v, curve)?;
            v.extend([*t0, *t1, f64::from(*orient)]);
            v.extend(end);
            (Op::Wire, v)
        }
        OpIn::Orient {
            outward,
            unify,
            faces,
        } => {
            let mut v = vec![flag(*outward), flag(*unify)];
            match faces {
                None => v.push(-1.0),
                Some(l) => {
                    v.push(l.len() as f64);
                    for &s in l {
                        let Some(x) = pick(s, n_f) else {
                            return Ok(None);
                        };
                        v.push(x as f64);
                    }
                }
            }
            (Op::Orient, v)
        }
        OpIn::Squeeze { edge, which, v } => {
            let Some(e) = pick(*edge, n_e) else {
                return Ok(None);
            };
            (
                Op::Squeeze,
                vec![e as f64, f64::from(*which), idx(pick(*v, n_v))],
            )
        }
    }))
}

fn ffi_err(e: &solidcore_ffi::FfiError) -> HarnessError {
    harness(e.to_string())
}

/// Whether the op runs without a fixture B-rep.
const fn standalone(op: &OpIn) -> bool {
    matches!(op, OpIn::Construct { .. } | OpIn::ApiCopyNull)
}

/// `(path, head, counts)` of a built B-rep, or the build failure's output.
type Prepared = Result<(String, Vec<f64>, [usize; 10]), Output>;

fn prepare(i: &Inputs) -> Result<Prepared, HarnessError> {
    let path = path_of(&i.brep)?;
    let h = head(i)?;
    let export = ffi2::eval(ffi2::Op::Export, &path, &h).map_err(|e| ffi_err(&e))?;
    match counts(&export) {
        Some(c) => Ok(Ok((path, h, c))),
        None => Ok(Err(Output::new().values("out", export))),
    }
}

fn run_cpp(i: &Inputs) -> Result<Output, HarnessError> {
    if standalone(&i.op) {
        let Some((op, args)) = op_args(&i.op, &[0; 10])? else {
            return Ok(Output::err("no target"));
        };
        let out = ffi::eval(op, "", &args).map_err(|e| ffi_err(&e))?;
        return Ok(Output::new().values("out", out));
    }
    let (path, mut input, c) = match prepare(i)? {
        Ok(x) => x,
        Err(o) => return Ok(o),
    };
    let Some((op, args)) = op_args(&i.op, &c)? else {
        return Ok(Output::err("no target"));
    };
    input.extend(args);
    let out = ffi::eval(op, &path, &input).map_err(|e| ffi_err(&e))?;
    Ok(Output::new().values("out", out))
}

// ---- Rust side ----------------------------------------------------------------------------

/// The identity each entity is notified and attributed under.
fn key_of(e: EntityRef) -> ObjectRef {
    let (type_code, i) = match e {
        EntityRef::Region(x) => (16012, x.index()),
        EntityRef::Shell(x) => (16013, x.index()),
        EntityRef::Faceuse(x) => (16022, x.index()),
        EntityRef::Face(x) => (16002, x.index()),
        EntityRef::Loopuse(x) => (16004, x.index()),
        EntityRef::Loop(x) => (16003, x.index()),
        EntityRef::Edgeuse(x) => (16005, x.index()),
        EntityRef::Edge(x) => (16016, x.index()),
        EntityRef::Vertexuse(x) => (16006, x.index()),
        EntityRef::Vertex(x) => (16021, x.index()),
        EntityRef::Brep(_) => (16999, 0),
    };
    ObjectRef {
        type_code,
        key: ObjectKey(1 + u64::from(i)),
    }
}

/// The Rust B-rep of the case (with the shim's extras restored).
fn rust_brep(path: &str, h: &[f64], c: &[usize; 10]) -> Result<Brep, HarnessError> {
    let export = ffi2::eval(ffi2::Op::Export, path, h).map_err(|e| ffi_err(&e))?;
    let (mut b, exact) = import(&export)?;
    if exact.iter().any(|e| !e) {
        return Err(harness("a face surface was not imported exactly"));
    }
    let ex = ffi2::eval(ffi2::Op::Extras, path, h).map_err(|e| ffi_err(&e))?;
    let mut at = 1usize;
    let mut next = || -> Result<f64, HarnessError> {
        let v = ex
            .get(at)
            .copied()
            .ok_or_else(|| harness("extras too short"))?;
        at += 1;
        Ok(v)
    };
    let tol = next()?;
    let inf = next()?;
    let head_r = next()?;
    let editing = next()? != 0.0;
    b.set_tolerance(tol);
    b.set_infinite_region((inf >= 0.0).then(|| RegionId::from_index(inf as u32)));
    b.set_region_list_head((head_r >= 0.0).then(|| RegionId::from_index(head_r as u32)));
    b.set_editing_enabled(editing);
    for v in 0..c[9] {
        let n = next()? as usize;
        let mut list = Vec::new();
        for _ in 0..n {
            let i = next()?;
            if i < 0.0 {
                return Err(harness("unregistered vertex use in a vertex list"));
            }
            list.push(VertexuseId::from_index(i as u32));
        }
        b.vertex_mut(VertexId::from_index(v as u32))
            .map_err(|e| harness(e.to_string()))?
            .vertexuses = list;
    }
    // the geometry's owners (set by the B-rep build upstream): the arena identities
    // (a face surface only when the export says the face owns it)
    let edge_ids: Vec<EdgeId> = b.edges().ids().collect();
    for id in edge_ids {
        if let Some(curve) = b
            .edge_mut(id)
            .map_err(|e| harness(e.to_string()))?
            .curve
            .as_mut()
        {
            let _ = curve.set_owner(
                Some(arena_identity(CacheObject::Edge(id))),
                Some(arena_identity(CacheObject::Brep)),
            );
        }
    }
    let face_ids: Vec<FaceId> = b.faces().ids().collect();
    for id in face_ids {
        if let Some(surface) = b
            .face_mut(id)
            .map_err(|e| harness(e.to_string()))?
            .surface
            .as_mut()
        {
            if surface.owner().is_some() {
                let _ = surface.set_owner(Some(arena_identity(CacheObject::Face(id))));
            }
        }
    }
    // the shared import leaves `m_pFU` unset: the face's upward (SAME) face use
    let ups: Vec<(FaceId, FaceuseId)> = b
        .faceuses()
        .iter()
        .filter(|(_, r)| r.orientation == OrientType::Same)
        .filter_map(|(id, r)| r.face.map(|f| (f, id)))
        .collect();
    for (f, fu) in ups {
        if let Ok(face) = b.face_mut(f) {
            if face.faceuse.is_none() {
                face.faceuse = Some(fu);
            }
        }
    }
    Ok(b)
}

fn rust_curve(c: &CrvIn) -> Result<BSplineCurve, HarnessError> {
    let pw = c
        .cpts
        .iter()
        .map(|p| CPoint::new(p[0], p[1], p[2], p[3]))
        .collect();
    let nl = NlCurve::new(pw, c.p, c.u.clone()).map_err(|e| harness(format!("curve: {e}")))?;
    Ok(BSplineCurve::from_nurb(nl, c.dim, false))
}

/// What a failed Rust op reports: the upstream status, or the deferral.
enum Fail {
    Deferred(Box<Output>),
    Harness(HarnessError),
}

impl From<HarnessError> for Fail {
    fn from(e: HarnessError) -> Self {
        Self::Harness(e)
    }
}

/// The status slot of a Rust result (`0` success, else the upstream code).
fn status<T>(r: Result<T, SolidError>) -> Result<(f64, Option<T>), Fail> {
    match r {
        Ok(v) => Ok((0.0, Some(v))),
        Err(e) => match Output::unported(&e) {
            Some(o) => Err(Fail::Deferred(Box::new(o))),
            None => match e {
                SolidError::Upstream { code, .. } => Ok((f64::from(code), None)),
                other => Err(Fail::Harness(harness(format!("rust: {other}")))),
            },
        },
    }
}

fn must<T>(r: Result<T, SolidError>) -> Result<T, Fail> {
    match status(r)? {
        (_, Some(v)) => Ok(v),
        (s, None) => Err(Fail::Harness(harness(format!("rust: status {s}")))),
    }
}

/// `faces edges vertices regions` of the B-rep's lists.
fn put_counts(o: &mut Vec<f64>, b: &Brep) -> Result<(), Fail> {
    o.push(must(bp1::faces(b, None))?.len() as f64);
    o.push(must(bp1::edges(b, None))?.len() as f64);
    o.push(must(bp1::vertices(b, None))?.len() as f64);
    o.push(must(bp1::regions(b, None))?.len() as f64);
    Ok(())
}

/// The fixture's `put_curve` layout of an edge curve.
fn put_curve(o: &mut Vec<f64>, c: Option<&SmCurve>) -> Result<(), Fail> {
    let Some(c) = c else {
        o.push(-1.0);
        return Ok(());
    };
    let base = match c.kind() {
        CurveKind::BSpline(x) => x,
        CurveKind::Line(x) => x.base(),
        CurveKind::Ellipse(x) => x.base(),
        CurveKind::Parabola(x) => x.base(),
        CurveKind::Hyperbola(x) => x.base(),
        _ => {
            o.push(-2.0);
            return Ok(());
        }
    };
    o.push(c.type_code() as f64);
    o.push(base.dim() as f64);
    let nurb = base.gw_nurb();
    o.push(flag(nurb.is_some()));
    if let Some(n) = nurb {
        o.extend([n.n() as f64, n.degree() as f64, n.m() as f64]);
        for p in n.cpts().iter().take(n.n() + 1) {
            o.extend([p.x, p.y, p.z, p.w]);
        }
        o.extend(n.knots().iter().take(n.m() + 1));
    }
    if let CurveKind::Line(_) = c.kind() {
        return Err(Fail::Harness(harness(
            "a line after TurnToNURBS (its extras are not exported)",
        )));
    }
    Ok(())
}

/// `SmRegion::m_bIsVoidFlag` of the new B-rep's infinite region (unit `topology.shell`).
fn region_void_flag(_b: &Brep, _r: Option<RegionId>) -> Result<f64, SolidError> {
    // UNPORTED(topology.shell): SmRegion::IsVoid
    Err(SolidError::unported("SmRegion::IsVoid", "topology.shell"))
}

#[allow(clippy::too_many_lines)]
fn run_rust_op(
    k: &mut Kernel,
    b: &mut Brep,
    second: Option<&mut Brep>,
    op: &OpIn,
    a: &[f64],
) -> Result<Vec<f64>, Fail> {
    let mut out = vec![0.0];
    let arg = |i: usize| -> Result<f64, Fail> {
        a.get(i)
            .copied()
            .ok_or_else(|| Fail::Harness(harness("args too short")))
    };
    let mut records = BTreeMap::new();
    let mut attrs = AttributeStore::default();
    let mut env = NotifyEnv {
        records: &mut records,
        attributes: &mut attrs,
        context: None,
        key_of: &key_of,
    };
    match op {
        OpIn::Construct { mse, lsr } => {
            let nb = must(bp3::new_brep(k, *mse, *lsr))?;
            out.push(0.0);
            out.push(nb.tolerance());
            out.push(nb.this_model_size_estimate(k, false));
            out.push(nb.this_large_small_size_ratio(k, false));
            out.push(flag(nb.editing_enabled()));
            out.push(must(region_void_flag(&nb, nb.infinite_region()))?);
            put_counts(&mut out, &nb)?;
        }
        OpIn::ApiCopyNull => {
            let (s, _) = status(bp3::api_brep_copy(k, &mut env, None))?;
            out.push(s);
        }
        OpIn::Copy { analytics, api } => {
            let src: &Brep = b;
            let r = if *api {
                bp3::api_brep_copy(k, &mut env, Some(src))
            } else {
                bp3::copy_brep(k, &mut env, src, None, None, true, *analytics)
            };
            let c = must(r)?;
            out.push(0.0);
            out.push(c.tolerance());
            out.push(flag(c.editing_enabled()));
            put_counts(&mut out, &c)?;
        }
        OpIn::Merge { analytics } => {
            let dst = second.ok_or_else(|| Fail::Harness(harness("no second B-rep")))?;
            let kk: &Kernel = k;
            let (s, _) = status(bp3::merge_brep(
                kk, &mut env, dst, b, None, None, *analytics,
            ))?;
            out.push(s);
            put_counts(&mut out, dst)?;
        }
        OpIn::CopyVertices { .. } => {
            let dst = second.ok_or_else(|| Fail::Harness(harness("no second B-rep")))?;
            let n = arg(0)? as usize;
            let vs: Vec<VertexId> = (0..n)
                .map(|j| arg(1 + j).map(|x| VertexId::from_index(x as u32)))
                .collect::<Result<_, _>>()?;
            let (s, r) = status(bp3::copy_vertices(b, &vs, dst, &mut env))?;
            out.push(s);
            out.push(r.map_or(0.0, |x| x.len() as f64));
            put_counts(&mut out, dst)?;
        }
        OpIn::DeleteFace { .. } => {
            let f = FaceId::from_index(arg(0)? as u32);
            let keep = (arg(3)? >= 0.0).then(|| RegionId::from_index(arg(3).unwrap_or(0.0) as u32));
            let mut marks = Marks::default();
            let opts = bp3::DeleteFaceOptions {
                delete_edges_vertices: arg(1)? != 0.0,
                do_region_nesting: arg(2)? != 0.0,
                region_to_keep: keep,
            };
            let (s, r) = status(bp3::delete_face(k, b, &mut env, &mut marks, f, opts))?;
            out.push(s);
            out.push(r.map_or(0.0, |x| x.len() as f64));
            put_counts(&mut out, b)?;
        }
        OpIn::Wire { curve, end, .. } => {
            let which = arg(0)? as u8;
            let region =
                (arg(1)? >= 0.0).then(|| RegionId::from_index(arg(1).unwrap_or(0.0) as u32));
            let v1 = VertexId::from_index(arg(2)? as u32);
            let v2 = VertexId::from_index(arg(3)?.max(0.0) as u32);
            let crv = SmCurve::from(rust_curve(curve)?);
            let at = a.len() - 6;
            let ivl = Extent1d::from_raw(arg(at)?, arg(at + 1)?);
            let orient = match arg(at + 2)? {
                0.0 => OrientType::Unknown,
                1.0 => OrientType::Same,
                2.0 => OrientType::Opposite,
                _ => return Err(Fail::Harness(harness("bad orientation"))),
            };
            let s = if which == 0 {
                status(bp3::make_wire_edge(
                    b, &mut env, region, v1, v2, &crv, &ivl, orient,
                ))?
                .0
            } else {
                let p = Vec3::new(end[0], end[1], end[2]);
                status(bp3::make_wire_edge_vertex(
                    b, &mut env, region, v1, &crv, &ivl, orient, p,
                ))?
                .0
            };
            out.push(s);
            put_counts(&mut out, b)?;
        }
        OpIn::Orient {
            outward,
            unify,
            faces,
        } => {
            let list: Option<Vec<FaceId>> = match faces {
                None => None,
                Some(l) => Some(
                    (0..l.len())
                        .map(|j| arg(3 + j).map(|x| FaceId::from_index(x as u32)))
                        .collect::<Result<_, _>>()?,
                ),
            };
            let before = must(bp1::faces(b, None))?;
            let mut marks = Marks::default();
            let (s, r) = status(bp3::orient_trimmed_surfaces(
                k,
                b,
                &mut env,
                &mut marks,
                *outward,
                *unify,
                list.as_deref(),
            ))?;
            out.push(s);
            out.push(flag(r.is_some_and(|x| x.maybe_not_closed_solid)));
            for f in before {
                let face = must(b.face(f))?;
                let d = face.uv_domain;
                out.extend([d.min().x, d.min().y, d.max().x, d.max().y]);
                out.push(face.faceuse.map_or(-1.0, |x| f64::from(x.index())));
            }
        }
        OpIn::Squeeze { .. } => {
            let e = EdgeId::from_index(arg(0)? as u32);
            let which = arg(1)? as u8;
            let any = (arg(2)? >= 0.0).then(|| VertexId::from_index(arg(2).unwrap_or(0.0) as u32));
            let v = match which {
                0 => must(solidcore::topology::edge::vertex(b, e))?,
                1 => {
                    let v0 = must(solidcore::topology::edge::vertex(b, e))?;
                    must(solidcore::topology::edge::other_vertex(b, e, v0))?
                }
                _ => any,
            };
            let s = match v {
                Some(v) => status(bp3::squeeze_edge(k, b, &mut env, e, v))?.0,
                // a null survivor is neither vertex of the edge (`SER_MSG(SM_ERR)`)
                None => f64::from(SM_ERR),
            };
            out.push(s);
            put_counts(&mut out, b)?;
        }
        OpIn::TurnNurbs => {
            let edges = must(bp1::edges(b, None))?;
            let faces = must(bp1::faces(b, None))?;
            let kk: &Kernel = k;
            let (s, _) = status(bp3::turn_to_nurbs(kk, b, &mut env))?;
            out.push(s);
            for e in edges {
                put_curve(&mut out, must(b.edge(e))?.curve.as_ref())?;
            }
            for f in faces {
                out.push(
                    must(b.face(f))?
                        .surface
                        .as_ref()
                        .map_or(-1.0, |x| x.type_code() as f64),
                );
            }
        }
    }
    Ok(out)
}

fn run_rust(i: &Inputs) -> Result<Output, HarnessError> {
    let mut k = Kernel::default();
    if standalone(&i.op) {
        let Some((_, args)) = op_args(&i.op, &[0; 10])? else {
            return Ok(Output::err("no target"));
        };
        let mut b = Brep::new();
        return match run_rust_op(&mut k, &mut b, None, &i.op, &args) {
            Ok(out) => Ok(Output::new().values("out", out)),
            Err(Fail::Deferred(o)) => Ok(*o),
            Err(Fail::Harness(e)) => Err(e),
        };
    }
    let (path, h, c) = match prepare(i)? {
        Ok(x) => x,
        Err(o) => return Ok(o),
    };
    let Some((_, args)) = op_args(&i.op, &c)? else {
        return Ok(Output::err("no target"));
    };
    let mut b = rust_brep(&path, &h, &c)?;
    let mut second = match i.op {
        OpIn::Merge { .. } | OpIn::CopyVertices { .. } => Some(rust_brep(&path, &h, &c)?),
        _ => None,
    };
    match run_rust_op(&mut k, &mut b, second.as_mut(), &i.op, &args) {
        Ok(out) => Ok(Output::new().values("out", out)),
        Err(Fail::Deferred(o)) => Ok(*o),
        Err(Fail::Harness(e)) => Err(e),
    }
}

// ---- generation ---------------------------------------------------------------------------

fn gen_op(rng: &mut Rng) -> (OpIn, i64) {
    let sel = |rng: &mut Rng| rng.below(1 << 20) as i64;
    let editing = if rng.chance(0.4) {
        -1
    } else {
        rng.below(2) as i64
    };
    match rng.below(12) {
        0 => (
            OpIn::Construct {
                mse: *rng
                    .choose(&[solidcore::geom::tol::USE_DEFAULT, 1.0, 100.0, 1e-3, 1e6])
                    .unwrap_or(&1.0),
                lsr: *rng
                    .choose(&[solidcore::geom::tol::USE_DEFAULT, 1e-3, 0.5])
                    .unwrap_or(&0.5),
            },
            editing,
        ),
        1 => (
            OpIn::Copy {
                analytics: rng.chance(0.5),
                api: rng.chance(0.5),
            },
            editing,
        ),
        2 => (
            OpIn::Merge {
                analytics: rng.chance(0.5),
            },
            editing,
        ),
        3 => {
            let n = rng.below(4) as usize;
            (
                OpIn::CopyVertices {
                    vertices: (0..n).map(|_| sel(rng)).collect(),
                },
                editing,
            )
        }
        4 | 5 => (
            OpIn::DeleteFace {
                face: sel(rng),
                delete_ev: rng.chance(0.5),
                nesting: rng.chance(0.7),
                keep: if rng.chance(0.7) { -1 } else { sel(rng) },
            },
            editing,
        ),
        6 => {
            let curve = gen_curve(rng, 3, 1.0);
            let (a, b) = (
                curve.u.first().copied().unwrap_or(0.0),
                curve.u.last().copied().unwrap_or(1.0),
            );
            (
                OpIn::Wire {
                    which: rng.below(2) as u8,
                    region: if rng.chance(0.6) { -1 } else { sel(rng) },
                    v1: sel(rng),
                    v2: sel(rng),
                    curve,
                    t0: a,
                    t1: b,
                    orient: 1 + rng.below(2) as u8,
                    end: rng.point3(-2.0, 2.0),
                },
                editing,
            )
        }
        7 | 8 => (
            OpIn::Orient {
                outward: rng.chance(0.5),
                unify: rng.chance(0.6),
                faces: if rng.chance(0.6) {
                    None
                } else {
                    let n = rng.below(3) as usize;
                    Some((0..n).map(|_| sel(rng)).collect())
                },
            },
            editing,
        ),
        9 | 10 => (
            OpIn::Squeeze {
                edge: sel(rng),
                which: rng.below(3) as u8,
                v: sel(rng),
            },
            editing,
        ),
        _ => (OpIn::TurnNurbs, editing),
    }
}

/// Generated cases per run, whatever `--seeds` says (it can only raise this).
pub const MIN_GENERATED: usize = 400;

impl UnitDiff for Runner {
    fn id(&self) -> &'static str {
        "topology.brep_p3"
    }

    fn cases(&self, ctx: &CaseContext) -> Result<Vec<Case>, HarnessError> {
        let ctx = CaseContext {
            seeds: ctx.seeds.max(MIN_GENERATED),
            ..ctx.clone()
        };
        default_cases(self, &ctx)
    }

    fn run_cpp(&self, case: &Case) -> Result<Output, HarnessError> {
        run_cpp(&case.parse()?)
    }

    fn run_rust(&self, case: &Case) -> Result<Output, HarnessError> {
        run_rust(&case.parse()?)
    }

    fn generate(&self, rng: &mut Rng) -> Option<Value> {
        let brep = gen_brep(rng);
        let (op, editing) = gen_op(rng);
        serde_json::to_value(Inputs {
            brep,
            validate_only: rng.chance(0.2),
            init_gaps: rng.chance(0.2),
            editing,
            op,
        })
        .ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Geometry beyond 1e150 hangs the C++ in `SmCurve::GlobalCoincidenceChecker`
    /// (HANG-16; fuzz case 1124: one control point at z = 1e300, Orient by rays): the
    /// FFI must reject it before calling in, and still accept the non-rational weight.
    #[test]
    fn huge_geometry_is_rejected() {
        let case = |z: f64| -> Inputs {
            let w = -f64::MAX;
            let v = serde_json::json!({
                "brep": {"kind": "face", "dom": [0.0, 0.0, 1.0, 1.0], "surface": {"bspline": {
                    "net": [[[0, 0, 0, w], [0, 1, 0, w]], [[1, 0, 0, w], [1, 1, z, w]]],
                    "p": 1, "q": 1, "u": [0, 0, 1, 1], "v": [0, 0, 1, 1]}}},
                "op": "orient", "outward": false, "unify": false, "faces": null
            });
            serde_json::from_value(v).unwrap()
        };
        let err = run_cpp(&case(1e300)).unwrap_err().to_string();
        assert!(err.contains("HANG-16"), "{err}");
        assert!(run_cpp(&case(0.5)).is_ok());
    }
}
