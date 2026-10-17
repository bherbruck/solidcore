//! `SmBrep` part 4: deleting edges and vertices from a B-rep (the Euler-style
//! topology surgery behind face merging and wire removal), stitching faces,
//! shrinking geometry to the used domains, and the B-rep destructor.
//!
//! Upstream: `source/SMLib/src/SmBrep.cpp`, `source/SMLib/inc/SmBrep.h`.
//!
//! | Rust                       | Upstream                      |
//! |----------------------------|-------------------------------|
//! | [`delete_edge`]            | `SmBrep::DeleteEdge`          |
//! | [`delete_vertex`]          | `SmBrep::DeleteVertex`        |
//! | [`stitch_faces`]           | `SmBrep::StitchFaces`         |
//! | [`StitchOutcome`]          | its `rlStitchedEdges` .. `rdMaxEdgeFaceTrimCurveGap` outputs |
//! | [`shrink_geometry`]        | `SmBrep::ShrinkGeometry`      |
//! | [`destroy`]                | `SmBrep::~SmBrep`             |
//! | [`DeleteEnv`]              | the objects' addresses and `SmTopology` members the notifications use |
//!
//! Deviations:
//! - **Deletion.** Upstream `delete`s topology objects; the port removes their
//!   records from the B-rep arenas ([`Brep::remove_edge_record`] and siblings), so the
//!   ids of the other records stay valid. `SmOwningTopology::Remove` / `PostInsert` on
//!   an owner's list are the list edits of the arena model: the id leaves / joins the
//!   owner's `Vec` (`PostInsert` appends) and the member's back link is cleared / set.
//!   A `Remove` whose member is not owned by the owner is `SM_ERR` with nothing
//!   changed, as upstream; `DeleteEdge` and `DeleteVertex` ignore that status, as upstream.
//!   The B-rep's face, edge and vertex lists are the arenas, so `m_p*ListHead->Remove`
//!   and the following `delete` are one record removal.
//! - **Surface identity.** Upstream compares `SmSurface*` pointers. As in
//!   [`crate::topology::edge`], a surface is named by the face holding it: two faces
//!   have the same surface when they are the same face (both with a surface).
//!   `pOptSurfaceToKeep` is the face whose surface is to be kept.
//! - **Null pointers.** Where upstream dereferences a null pointer or casts a list
//!   owner to the wrong kind (undefined behaviour), the port returns
//!   [`SolidError::InvalidInput`]; `NER` checks return the `SM_ERR_NULL_POINTER`
//!   status, as upstream.
//! - **Surface swap.** `DeleteEdge` swaps the two faces' surfaces through
//!   `SmFace::SetSurface` (ported in `topology.face_p1`, which moves an owned
//!   `Surface`): the swap takes the surface out of the deleted face and hands the
//!   replaced one back.
//! - **`SmLockBrepBox`** is RAII upstream. `LockBrepBox::scoped` is the scope: it locks,
//!   runs the body and unlocks once on every path out of it (`StitchFaces`).
//! - **Notifications** go to the ported handlers (`SmEdge::Notify`,
//!   `SmVertex::Notify`) with the caller's [`DeleteEnv`]; `SmBrep::Notify` (unit
//!   `topology.brep_p2`) and `SmFace::Notify` (unit `topology.face_p3`) are not
//!   ported yet. Their statuses are ignored upstream; [`SolidError::Unported`]
//!   propagates.
//! - **Shell lists.** The shell's `m_pList` / `m_tShellType` (unit `topology.shell`)
//!   are not in the arena model yet; reading or writing them returns
//!   [`SolidError::Unported`] naming that unit.
//! - **Loops.** A loop made by splitting a loop gets the split face as
//!   [`crate::topology::Loop::face`]; `Loop::edges` stays empty (the loop uses carry
//!   the topology).
//! - **Destructor.** [`destroy`] runs the destructor's observable work (editing
//!   enabled, the context's boolean flag set for the duration and restored, the
//!   notifications) and empties the arenas; Rust drops the records themselves.
//!   The flag is restored on every return, including [`SolidError::Unported`].
//! - **Bounds.** No recursion; every loop is bounded by a record list. The edge-use
//!   walks are those of the ported `CollectEdgeuses` (bounded there).

use crate::error::{ignore_status, ignored, EntityRef, Result, SolidError};
use crate::geom::attribute::{AttributeStore, ObjectKey};
use crate::geom::context::{MarkLock, ALL_MARKS};
use crate::geom::extent1d::Extent1d;
use crate::geom::extent2d::Extent2d;
use crate::geom::object::{NotifyOperation, ObjectRef};
use crate::kernel::Kernel;
use crate::math::{Point2, Vec3, EFF_ZERO};
use crate::nurbs::b_spline_curve_n_lib::TrimMode;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::nurbs::knots::Side;
use crate::topology::edge::{self, NotifyEnv};
use crate::topology::edgeuse::p1 as eu1;
use crate::topology::face::p1 as face1;
use crate::topology::face::p2 as face2;
use crate::topology::topology::Topology;
use crate::topology::topology_traverser::{shell_traversal, Entity, Marks};
use crate::topology::{
    loopuse, vertex, vertexuse, Brep, EdgeId, EdgeuseId, EdgeuseOwner, FaceId, FaceuseId, Loop,
    LoopId, Loopuse, LoopuseId, LoopuseStart, RegionId, Shell, ShellId, VertexId, VertexuseId,
    VertexuseOwner,
};
use std::collections::BTreeMap;

const OP_DELETE_EDGE: &str = "topology::brep::p4::delete_edge";
const OP_DELETE_VERTEX: &str = "topology::brep::p4::delete_vertex";
const OP_SHRINK: &str = "topology::brep::p4::shrink_geometry";
const OP_DESTROY: &str = "topology::brep::p4::destroy";

/// The caller's state the notifications of [`delete_edge`] and [`delete_vertex`]
/// go through (as [`NotifyEnv`], with the kernel's context).
pub struct DeleteEnv<'a> {
    /// The topology records (`SmTopology` members) of the notified objects.
    pub records: &'a mut BTreeMap<ObjectKey, Topology>,
    /// The attribute store.
    pub attributes: &'a mut AttributeStore,
    /// The identity each topology entity is notified under.
    pub key_of: &'a dyn Fn(EntityRef) -> ObjectRef,
}

impl core::fmt::Debug for DeleteEnv<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DeleteEnv")
            .field("records", &self.records.len())
            .finish_non_exhaustive()
    }
}

// ---- unported callees ---------------------------------------------------------------------

/// `SmBrep::Notify` (unit `topology.brep_p2`).
fn brep_notify(_b: &Brep, _operation: NotifyOperation) -> Result<()> {
    // UNPORTED(topology.brep_p2): SmBrep::Notify
    Err(SolidError::unported("SmBrep::Notify", "topology.brep_p2"))
}

/// `SmFace::Notify` (unit `topology.face_p3`).
fn face_notify(_b: &Brep, _f: FaceId, _operation: NotifyOperation) -> Result<()> {
    // UNPORTED(topology.face_p3): SmFace::Notify
    Err(SolidError::unported("SmFace::Notify", "topology.face_p3"))
}

/// `SmEdgeuse::NormalizedEvaluate(t, TRUE, ..)`: the UV point at normalized `t`.
fn edgeuse_normalized_evaluate_uv(_b: &Brep, _eu: EdgeuseId, _t: f64) -> Result<Vec3> {
    // UNPORTED(topology.edgeuse_p3): SmEdgeuse::NormalizedEvaluate
    Err(SolidError::unported(
        "SmEdgeuse::NormalizedEvaluate",
        "topology.edgeuse_p3",
    ))
}

/// `SmFace::ComputeLoopOrientation(loop, orient)` (default domain and flags).
fn face_compute_loop_orientation(_b: &Brep, _f: FaceId, _l: Option<LoopId>) -> Result<OrientType> {
    // UNPORTED(topology.face_p3): SmFace::ComputeLoopOrientation
    Err(SolidError::unported(
        "SmFace::ComputeLoopOrientation",
        "topology.face_p3",
    ))
}

/// `SmFace::GetVertices(vertices)` (default arguments).
fn face_vertices(_b: &Brep, _f: FaceId) -> Result<Vec<VertexId>> {
    // UNPORTED(topology.face_p4): SmFace::GetVertices
    Err(SolidError::unported(
        "SmFace::GetVertices",
        "topology.face_p4",
    ))
}

/// `SmFace::GetEdges(edges)` (default arguments).
fn face_edges(_b: &Brep, _f: FaceId) -> Result<Vec<EdgeId>> {
    // UNPORTED(topology.face_p4): SmFace::GetEdges
    Err(SolidError::unported("SmFace::GetEdges", "topology.face_p4"))
}

/// `SmShell::GetRegion`.
fn shell_region(_b: &Brep, _s: ShellId) -> Result<Option<RegionId>> {
    // UNPORTED(topology.shell): SmShell::GetRegion
    Err(SolidError::unported("SmShell::GetRegion", "topology.shell"))
}

/// `SmShell::IsWireShell`.
fn shell_is_wire_shell(_b: &Brep, _s: ShellId) -> Result<bool> {
    // UNPORTED(topology.shell): SmShell::IsWireShell
    Err(SolidError::unported(
        "SmShell::IsWireShell",
        "topology.shell",
    ))
}

/// A shell's list head and type (`m_pList`, `m_tShellType`) as a vertex-use or edge-use
/// head: whether it is `head`.
fn shell_list_is(_b: &Brep, _s: ShellId, _head: EntityRef) -> Result<bool> {
    // UNPORTED(topology.shell): SmShell::m_tShellType
    Err(SolidError::unported(
        "SmShell::m_tShellType",
        "topology.shell",
    ))
}

/// Sets a shell's list head and type (`m_pList`, `UpdateListSize`, `m_tShellType`):
/// a vertex-use or edge-use head, or a face-use shell (`None`, type only).
fn shell_set_list(_b: &mut Brep, _s: ShellId, _head: Option<EntityRef>) -> Result<()> {
    // UNPORTED(topology.shell): SmShell::m_pList
    Err(SolidError::unported("SmShell::m_pList", "topology.shell"))
}

/// `SmOwningTopology::Remove(shell)` on a region's shell list (`m_pList`, not in the
/// arena model yet).
fn region_remove_shell(_b: &mut Brep, _r: Option<RegionId>, _s: ShellId) -> Result<()> {
    // UNPORTED(topology.shell): SmRegion::m_pList
    Err(SolidError::unported("SmRegion::m_pList", "topology.shell"))
}

/// `SmRegion::GetShells(shells)`.
fn region_shells(_b: &Brep, _r: RegionId) -> Result<Vec<ShellId>> {
    // UNPORTED(topology.shell): SmRegion::GetShells
    Err(SolidError::unported(
        "SmRegion::GetShells",
        "topology.shell",
    ))
}

/// `SmBrep::GetFaces(faces)` (no filter).
fn brep_get_faces(_b: &Brep) -> Result<Vec<FaceId>> {
    // UNPORTED(topology.brep_p1): SmBrep::GetFaces
    Err(SolidError::unported("SmBrep::GetFaces", "topology.brep_p1"))
}

/// `SmBrep::GetEdges(edges)` (no filter).
fn brep_get_edges(_b: &Brep) -> Result<Vec<EdgeId>> {
    // UNPORTED(topology.brep_p1): SmBrep::GetEdges
    Err(SolidError::unported("SmBrep::GetEdges", "topology.brep_p1"))
}

/// `SmBrep::GetVertices(vertices)` (no filter).
fn brep_get_vertices(_b: &Brep) -> Result<Vec<VertexId>> {
    // UNPORTED(topology.brep_p1): SmBrep::GetVertices
    Err(SolidError::unported(
        "SmBrep::GetVertices",
        "topology.brep_p1",
    ))
}

/// `SmBrep::GetRegions(regions)` (no filter).
fn brep_get_regions(_b: &Brep) -> Result<Vec<RegionId>> {
    // UNPORTED(topology.brep_p1): SmBrep::GetRegions
    Err(SolidError::unported(
        "SmBrep::GetRegions",
        "topology.brep_p1",
    ))
}

/// `SmBrep::GetSurfaces(surfaces)`: each surface named by the face holding it.
fn brep_get_surfaces(_b: &Brep) -> Result<Vec<FaceId>> {
    // UNPORTED(topology.brep_p1): SmBrep::GetSurfaces
    Err(SolidError::unported(
        "SmBrep::GetSurfaces",
        "topology.brep_p1",
    ))
}

/// `SmBrep::GetCurves(curves)`: each curve named by the edge holding it.
fn brep_get_curves(_b: &Brep) -> Result<Vec<EdgeId>> {
    // UNPORTED(topology.brep_p1): SmBrep::GetCurves
    Err(SolidError::unported(
        "SmBrep::GetCurves",
        "topology.brep_p1",
    ))
}

/// `SmLockBrepBox::SmLockBrepBox(brep)`: locks the B-rep's bounding box
/// (`SmBrep::LockBoundingBox`, unit `query.brep`, inlined into the constructor).
fn lock_brep_box(_b: &Brep) -> Result<()> {
    // UNPORTED(topology.brep_p1): SmLockBrepBox::SmLockBrepBox
    Err(SolidError::unported(
        "SmLockBrepBox::SmLockBrepBox",
        "topology.brep_p1",
    ))
}

/// `SmLockBrepBox`: the scope guard of the B-rep's bounding box lock. Upstream
/// `~SmLockBrepBox` unlocks it when the guard goes out of scope, so every return path
/// of the scope unlocks exactly once.
///
/// The guard cannot hold the B-rep (the scope edits it) and so cannot unlock from
/// `Drop`; [`LockBrepBox::scoped`] is the scope: it locks, runs the body and unlocks
/// on the way out, whatever the body returned. [`LockBrepBox::unlock`] consumes the
/// guard, so it unlocks once.
#[derive(Debug)]
struct LockBrepBox;

impl LockBrepBox {
    /// `SmLockBrepBox lockit(brep)`.
    fn lock(b: &Brep) -> Result<Self> {
        lock_brep_box(b)?;
        Ok(Self)
    }

    /// `~SmLockBrepBox()`: unlocks (`SmBrep::UnLockBoundingBox`); the destructor
    /// cannot fail, so the status is dropped.
    fn unlock(self, b: &mut Brep) -> Result<()> {
        let Self = self; // the guard ends here
        ignore_status(unlock_bounding_box(b))
    }

    /// Runs `body` with the lock held and unlocks after it, on success and on error.
    ///
    /// [`SolidError::Unported`] outranks everything else: from the body it is the
    /// result as is; from the unlock (not ported yet) it replaces the body's result,
    /// which cannot be told to be upstream's without it. A lock that fails (while it
    /// is not ported) never runs the body and has nothing to unlock.
    fn scoped<T>(b: &mut Brep, body: impl FnOnce(&mut Brep) -> Result<T>) -> Result<T> {
        let guard = Self::lock(b)?;
        let result = body(b);
        let unlocked = guard.unlock(b);
        match (result, unlocked) {
            (Err(e), _) if e.is_unported() => Err(e),
            (_, Err(e)) => Err(e),
            (result, Ok(())) => result,
        }
    }
}

/// `SmBrep::UnLockBoundingBox`.
fn unlock_bounding_box(_b: &mut Brep) -> Result<()> {
    // UNPORTED(query.brep): SmBrep::UnLockBoundingBox
    Err(SolidError::unported(
        "SmBrep::UnLockBoundingBox",
        "query.brep",
    ))
}

// ---- helpers -------------------------------------------------------------------------------

/// `SmFace::CalculateUVDomainFromUVTrimCurves(domain)` with upstream's defaults
/// (`bLimitDomain = TRUE`, no loop gap outputs): the domain it returns.
fn face_calculate_uv_domain(k: &Kernel, b: &mut Brep, f: FaceId) -> Result<Extent2d> {
    face1::calculate_uv_domain_from_uv_trim_curves(k, b, f, true, false, false).map(|o| o.domain)
}

/// `SmFace::RemoveLoop(pLoop)`. Upstream notifies the face and then dereferences the
/// loop, so a null loop is the null-dereference error after the notification.
fn face_remove_loop(b: &mut Brep, f: FaceId, l: Option<LoopId>) -> Result<()> {
    if let Some(l) = l {
        return face1::remove_loop(b, f, l);
    }
    ignore_status(face_notify(b, f, NotifyOperation::PreEdit))?;
    Err(SolidError::invalid_input(
        OP_DELETE_EDGE,
        "null pointer dereference",
    ))
}

/// The surface swap of `DeleteEdge`:
/// `pSurfTmp = pSurvF->GetSurface(); pSurvF->SetSurface(pDelF->GetSurface());
/// pDelF->SetSurface(pSurfTmp);`.
///
/// Surfaces are owned values here, so the swap takes the surface out of `del_f`, gives
/// it to `surv_f` (`SetSurface` hands back the one it replaces, without deleting it)
/// and gives that one to `del_f`. `SetSurface` is `void` upstream; the only failure
/// the port can raise is [`SolidError::Unported`] from its notification. A face
/// without a surface has none to hand over: upstream notifies and then dereferences
/// the null surface.
fn swap_face_surfaces(
    b: &mut Brep,
    key_of: &dyn Fn(EntityRef) -> ObjectRef,
    surv_f: FaceId,
    del_f: FaceId,
) -> Result<()> {
    let del_surface = deref(b.face_mut(del_f)?.surface.take(), OP_DELETE_EDGE)?;
    let surv_this = key_of(EntityRef::Face(surv_f));
    let surv_old = face1::set_surface(b, surv_f, surv_this, del_surface, false)?;
    let Some(surv_old) = surv_old else {
        ignore_status(face_notify(b, del_f, NotifyOperation::ChangeGeometry))?;
        return Err(SolidError::invalid_input(
            OP_DELETE_EDGE,
            "null pointer dereference",
        ));
    };
    let del_this = key_of(EntityRef::Face(del_f));
    face1::set_surface(b, del_f, del_this, surv_old, false)?;
    Ok(())
}

/// A pointer upstream dereferences without a check.
fn deref<T>(o: Option<T>, op: &'static str) -> Result<T> {
    o.ok_or_else(|| SolidError::invalid_input(op, "null pointer dereference"))
}

/// `NER(p)`: the null-pointer status.
fn ner<T>(o: Option<T>, op: &'static str) -> Result<T> {
    o.ok_or_else(|| SolidError::null_pointer(op))
}

/// The surface of face `f`, named by the face (see the module docs).
fn surface_of(b: &Brep, f: FaceId) -> Result<Option<FaceId>> {
    Ok(b.face(f)?.surface.is_some().then_some(f))
}

fn set_ccw(b: &mut Brep, eu: EdgeuseId, to: Option<EdgeuseId>) -> Result<()> {
    b.edgeuse_mut(eu)?.ccw = to;
    Ok(())
}

fn set_cw(b: &mut Brep, eu: EdgeuseId, to: Option<EdgeuseId>) -> Result<()> {
    b.edgeuse_mut(eu)?.cw = to;
    Ok(())
}

fn ccw(b: &Brep, eu: EdgeuseId) -> Result<Option<EdgeuseId>> {
    eu1::ccw_edgeuse(b, eu)
}

/// `x->m_pCCW = y; y->m_pCW = x` with `y` read now (`y` dereferenced).
fn link(b: &mut Brep, x: Option<EdgeuseId>, y: Option<EdgeuseId>, op: &'static str) -> Result<()> {
    let x = deref(x, op)?;
    set_ccw(b, x, y)?;
    let y = deref(y, op)?;
    set_cw(b, y, Some(x))
}

/// `SmOwningTopology::Remove` of an edge use from its edge's list.
fn edge_remove(b: &mut Brep, e: EdgeId, eu: EdgeuseId, op: &'static str) -> Result<()> {
    if b.edgeuse(eu)?.edge != Some(e) {
        return Err(SolidError::sm_err(op));
    }
    let list = &mut b.edge_mut(e)?.edgeuses;
    let Some(at) = list.iter().position(|&x| x == eu) else {
        return Err(SolidError::sm_err(op));
    };
    list.remove(at);
    b.edgeuse_mut(eu)?.edge = None;
    Ok(())
}

/// `SmOwningTopology::Remove` of a vertex use from its vertex's list.
fn vertex_remove(b: &mut Brep, v: VertexId, vu: VertexuseId, op: &'static str) -> Result<()> {
    if b.vertexuse(vu)?.vertex != Some(v) {
        return Err(SolidError::sm_err(op));
    }
    let list = &mut b.vertex_mut(v)?.vertexuses;
    let Some(at) = list.iter().position(|&x| x == vu) else {
        return Err(SolidError::sm_err(op));
    };
    list.remove(at);
    b.vertexuse_mut(vu)?.vertex = None;
    Ok(())
}

/// `SmOwningTopology::PostInsert` of a vertex use at the end of a vertex's list.
fn vertex_post_insert(b: &mut Brep, v: VertexId, vu: VertexuseId) -> Result<()> {
    b.vertex_mut(v)?.vertexuses.push(vu);
    b.vertexuse_mut(vu)?.vertex = Some(v);
    Ok(())
}

/// `SmOwningTopology::Remove` of a loop use from its face use's list.
fn faceuse_remove(b: &mut Brep, fu: FaceuseId, lu: LoopuseId, op: &'static str) -> Result<()> {
    if b.loopuse(lu)?.faceuse != Some(fu) {
        return Err(SolidError::sm_err(op));
    }
    let list = &mut b.faceuse_mut(fu)?.loopuses;
    let Some(at) = list.iter().position(|&x| x == lu) else {
        return Err(SolidError::sm_err(op));
    };
    list.remove(at);
    b.loopuse_mut(lu)?.faceuse = None;
    Ok(())
}

/// `SmOwningTopology::PostInsert` of a loop use at the end of a face use's list.
fn faceuse_post_insert(b: &mut Brep, fu: FaceuseId, lu: LoopuseId) -> Result<()> {
    b.faceuse_mut(fu)?.loopuses.push(lu);
    b.loopuse_mut(lu)?.faceuse = Some(fu);
    Ok(())
}

/// `SmOwningTopology::Remove` of a face use from its shell (the arena model keeps
/// the shell's faces: a face leaves it when its face use does).
fn shell_remove_faceuse(b: &mut Brep, s: ShellId, fu: FaceuseId, op: &'static str) -> Result<()> {
    if b.faceuse(fu)?.shell != Some(s) {
        return Err(SolidError::sm_err(op));
    }
    let face = b.faceuse(fu)?.face;
    b.faceuse_mut(fu)?.shell = None;
    if let Some(f) = face {
        b.shell_mut(s)?.faces.retain(|&x| x != f);
    }
    Ok(())
}

/// A vertex use becomes the vertex use of loop use `lu` (`m_tVertexuseType`,
/// `m_pSorLUorEU`) and `lu` a vertex loop use of it (`m_tLoopuseType`, `m_pEUorVU`).
fn make_vertex_loop(b: &mut Brep, vu: VertexuseId, lu: LoopuseId) -> Result<()> {
    b.vertexuse_mut(vu)?.owner = VertexuseOwner::Loopuse(Some(lu));
    b.loopuse_mut(lu)?.start = LoopuseStart::Vertexuse(Some(vu));
    Ok(())
}

/// `SmVertex::Notify(SM_NO_PRE_EDIT, vertex, NULL, NULL)`.
fn notify_vertex(k: &mut Kernel, b: &mut Brep, env: &mut DeleteEnv<'_>, v: VertexId) -> Result<()> {
    let key_of = env.key_of;
    let this = key_of(EntityRef::Vertex(v));
    vertex::notify(
        b,
        v,
        env.records,
        env.attributes,
        Some(&mut k.context),
        NotifyOperation::PreEdit,
        this,
        [Some(this), None, None],
        &|vu| key_of(EntityRef::Vertexuse(vu)),
    )
}

/// `SmEdge::Notify(SM_NO_PRE_EDIT, edge, NULL, NULL)`.
fn notify_edge(k: &mut Kernel, b: &mut Brep, env: &mut DeleteEnv<'_>, e: EdgeId) -> Result<()> {
    let key_of = env.key_of;
    let this = key_of(EntityRef::Edge(e));
    let mut n = NotifyEnv {
        records: env.records,
        attributes: env.attributes,
        context: Some(&mut k.context),
        key_of,
    };
    edge::notify(
        b,
        e,
        &mut n,
        NotifyOperation::PreEdit,
        [Some(this), None, None],
    )
}

/// Deletes an edge use record (`delete pEU`).
fn delete_edgeuse(b: &mut Brep, eu: EdgeuseId) -> Result<()> {
    b.remove_edgeuse_record(eu).map(|_| ())
}

// ---- DeleteEdge ----------------------------------------------------------------------------

/// `SmBrep::DeleteEdge`: deletes a wire, lamina or manifold edge. Deleting a manifold
/// edge between two faces merges them (the face of the loop use chosen below
/// survives; `surface_to_keep`, the face holding the surface to keep, can force the
/// survivor's surface), between two sides of one loop splits the loop; deleting a
/// lamina edge shortens its loop; deleting a wire edge may split its shell.
///
/// # Errors
///
/// `SM_ERR` ([`SolidError::Upstream`]) when editing is not enabled, the edge is not
/// wire, lamina or manifold, a lamina edge is a strut, or two inner loops would be
/// merged; the null-pointer status where upstream checks a pointer (`NER`); the
/// statuses of `CollectEdgeuses` and the other checked calls;
/// [`SolidError::InvalidInput`] where upstream dereferences null;
/// [`SolidError::Unported`] from the callees not ported yet (see the module docs).
pub fn delete_edge(
    k: &mut Kernel,
    b: &mut Brep,
    env: &mut DeleteEnv<'_>,
    e: EdgeId,
    surface_to_keep: Option<FaceId>,
) -> Result<()> {
    const OP: &str = OP_DELETE_EDGE;
    // check state - m_bEditingEnabled must be on
    if !b.editing_enabled() {
        return Err(SolidError::sm_err(OP));
    }

    // Only delete wire, lamina, or manifold edges in this method
    let is_manifold = edge::is_manifold(b, e)?;
    if !(edge::is_wire(b, e)? || is_manifold || edge::is_lamina(b, e)?) {
        return Err(SolidError::sm_err(OP));
    }

    // prepare the public - notify the Brep it's about to be edited
    ignore_status(brep_notify(b, NotifyOperation::PreEdit))?;
    ignore_status(notify_edge(k, b, env, e))?;
    ignore_status(brep_notify(b, NotifyOperation::RmFromBrep))?;

    // get radial/mate list starters
    let eu = deref(eu1::primary_edgeuse(b, e)?, OP)?;
    let eu_mate = deref(eu1::mate(b, eu)?, OP)?;

    if is_manifold {
        delete_manifold_edge(b, env.key_of, e, eu, eu_mate, surface_to_keep)
    } else if edge::is_lamina(b, e)? {
        delete_lamina_edge(b, e, eu, eu_mate)
    } else {
        delete_wire_edge(k, b, e, eu, eu_mate)
    }
}

/// The manifold branch of [`delete_edge`].
fn delete_manifold_edge(
    b: &mut Brep,
    key_of: &dyn Fn(EntityRef) -> ObjectRef,
    e: EdgeId,
    eu: EdgeuseId,
    eu_mate: EdgeuseId,
    surface_to_keep: Option<FaceId>,
) -> Result<()> {
    const OP: &str = OP_DELETE_EDGE;
    // get Radial/Mate neighbor Edgeuses
    let eu_rad = deref(eu1::radial(b, eu)?, OP)?;
    let eu_rad_mate = deref(eu1::mate(b, eu_rad)?, OP)?;

    let mut flip_face = false;

    // Don't do the face flip check if same surface on both sides.
    let srf_face = deref(eu1::face(b, eu)?, OP)?;
    let srf_rad_face = deref(eu1::face(b, eu_rad)?, OP)?;
    let srf = surface_of(b, srf_face)?;
    let srf_rad = surface_of(b, srf_rad_face)?;
    if srf != srf_rad {
        let norm1 = edgeuse_normal(b, eu, srf, OP)?;
        let norm2 = edgeuse_normal(b, eu_rad, srf_rad, OP)?;
        let dot = norm1.dot(norm2);
        if dot < 0.0 {
            flip_face = true;
        }
    }

    // get Primary, Mate, Radial, and RadialMate Loopuses
    let mut lu = ner(eu1::loopuse(b, eu)?, OP)?;
    let lu_mate = ner(eu1::loopuse(b, eu_mate)?, OP)?;
    let lu_rad = ner(eu1::loopuse(b, eu_rad)?, OP)?;
    let mut lu_rad_mate = ner(eu1::loopuse(b, eu_rad_mate)?, OP)?;

    // get primary faceuse and face
    let faceuse = ner(loopuse::faceuse(b, lu)?, OP)?;
    let face = ner(eu1::faceuse_face(b, faceuse)?, OP)?;

    // get Radial face
    let fu_rad = deref(loopuse::faceuse(b, lu_rad)?, OP)?;
    let face_rad = ner(eu1::faceuse_face(b, fu_rad)?, OP)?;

    // mark which side of face primaryEdgeuse is on
    let eu_on_primary_side = face2::upward_faceuse(b, face)? == Some(faceuse);

    // note when removed edge splits a loop
    let split_loop = lu == lu_rad;

    // mark when face and radial face have same orientation
    let same_orientation_faces =
        eu1::faceuse_orientation(b, faceuse)? == eu1::faceuse_orientation(b, fu_rad)?;

    // Determine if either end of edge is a strut end
    let v_is_strut = eu1::is_strut(b, eu)?;
    let v_rad_is_strut = eu1::is_strut(b, eu_rad)?;

    let both_inner = loopuse::orientation(b, lu)? == OrientType::Opposite
        && loopuse::orientation(b, lu_rad)? == OrientType::Opposite;

    // determine when faces are to be merged
    let merge_face = !split_loop && !v_is_strut && !v_rad_is_strut && !both_inner;

    // When merging faces - pick the surviving and delete faces.
    let mut surv_lu = None;
    let mut del_lu = None;
    let mut surv_f = None;
    let mut del_f = None;
    if merge_face {
        let swap_del_face = if loopuse::orientation(b, lu)? != loopuse::orientation(b, lu_rad)? {
            // one outer, one inner: choose inner loop to survive
            loopuse::orientation(b, lu)? == OrientType::Same
        } else if let Some(keep) = surface_to_keep {
            surface_of(b, face)? != Some(keep)
        } else {
            false
        };
        let (s, d) = if swap_del_face {
            (lu_rad, lu)
        } else {
            (lu, lu_rad)
        };
        surv_lu = Some(s);
        del_lu = Some(d);
        let s_fu = deref(loopuse::faceuse(b, s)?, OP)?;
        surv_f = Some(ner(eu1::faceuse_face(b, s_fu)?, OP)?);
        let d_fu = deref(loopuse::faceuse(b, d)?, OP)?;
        del_f = eu1::faceuse_face(b, d_fu)?;
    }

    // get edges connected to pEdge through loops
    let mut eu_cw = eu1::cw_edgeuse(b, eu)?;
    let mut eu_rad_cw = eu1::cw_edgeuse(b, eu_rad)?;
    let mut eu_mate_cw = eu1::cw_edgeuse(b, eu_mate)?;
    let mut eu_rad_mate_cw = eu1::cw_edgeuse(b, eu_rad_mate)?;

    let eu_single = ccw(b, eu)? == Some(eu);
    let eu_rad_single = ccw(b, eu_rad)? == Some(eu_rad);
    let single_closed_loop = lu != lu_rad && (eu_single || eu_rad_single);
    let single_closed_loop_isolated = lu != lu_rad && eu_single && eu_rad_single;

    // notify the face and the radial face of the upcoming edit
    ignore_status(face_notify(b, face, NotifyOperation::PreEdit))?;
    ignore_status(face_notify(b, face_rad, NotifyOperation::PreEdit))?;

    if merge_face {
        // inform the public about this upcoming merge in Brep
        ignore_status(brep_notify(b, NotifyOperation::MergeInBrep))?;
    }

    // Remove parameter space curves from both faces if they do not have the same
    // underlying surface
    if !split_loop && surface_of(b, face)? != surface_of(b, face_rad)? {
        face1::remove_uv_trim_curves(b, face)?;
        face1::remove_uv_trim_curves(b, face_rad)?;
    }

    // When merging two faces - see if Face Surfaces need to be swapped
    if !split_loop && !both_inner {
        if let Some(keep) = surface_to_keep {
            let sf = deref(surv_f, OP)?;
            if Some(keep) != surface_of(b, sf)? {
                let df = deref(del_f, OP)?;
                // when DelF owns the surface to keep
                if surface_of(b, df)? == Some(keep) {
                    // swap the surfaces between the faces
                    swap_face_surfaces(b, key_of, sf, df)?;
                    if flip_face {
                        ignore_status(face1::flip_face_orientation(b, sf))?;
                    }
                }
                // else: WARN (asked to keep a surface not used by the faces)
            }
        }
    }

    if eu_rad_cw == Some(eu_rad) {
        let c = ccw(b, eu)?;
        link(b, eu_cw, c, OP)?;
        let c = ccw(b, eu_mate)?;
        link(b, eu_mate_cw, c, OP)?;
    } else if eu_cw == Some(eu) {
        let c = ccw(b, eu_rad)?;
        link(b, eu_rad_cw, c, OP)?;
        let c = ccw(b, eu_rad_mate)?;
        link(b, eu_rad_mate_cw, c, OP)?;
    } else {
        let c = ccw(b, eu_rad)?;
        link(b, eu_cw, c, OP)?;
        let c = ccw(b, eu)?;
        link(b, eu_rad_cw, c, OP)?;
        let c = ccw(b, eu_rad_mate)?;
        link(b, eu_mate_cw, c, OP)?;
        let c = ccw(b, eu_mate)?;
        link(b, eu_rad_mate_cw, c, OP)?;
    }

    if split_loop && !single_closed_loop {
        let new_l = b.add_loop_record(Loop {
            face,
            edges: Vec::new(),
            loopuse: None,
        })?;
        lu = b.add_loopuse_record(Loopuse::default())?;
        lu_rad_mate = b.add_loopuse_record(Loopuse::default())?;
        {
            let r = b.loopuse_mut(lu)?;
            r.loop_ = Some(new_l);
            r.mate = Some(lu_rad_mate);
        }
        {
            let r = b.loopuse_mut(lu_rad_mate)?;
            r.loop_ = Some(new_l);
            r.mate = Some(lu);
        }
        let (fu, fu_mate) = face2::faceuses(b, face)?;
        let fu = deref(fu, OP)?;
        let fu_mate = deref(fu_mate, OP)?;
        if eu_on_primary_side {
            b.loop_mut(new_l)?.loopuse = Some(lu);
            faceuse_post_insert(b, fu, lu)?;
            faceuse_post_insert(b, fu_mate, lu_rad_mate)?;
        } else {
            b.loop_mut(new_l)?.loopuse = Some(lu_rad_mate);
            faceuse_post_insert(b, fu_mate, lu)?;
            faceuse_post_insert(b, fu, lu_rad_mate)?;
        }
        b.loopuse_mut(lu)?.orientation = OrientType::Opposite;
        b.loopuse_mut(lu_rad_mate)?.orientation = OrientType::Opposite;
    }

    if v_is_strut || single_closed_loop_isolated {
        let vu = ner(eu1::vertexuse(b, eu)?, OP)?;
        if single_closed_loop_isolated && loopuse::orientation(b, lu)? == OrientType::Same {
            make_vertex_loop(b, vu, lu_rad)?;
        } else {
            make_vertex_loop(b, vu, lu)?;
        }
        let vu_rad_mate = ner(eu1::vertexuse(b, eu_rad_mate)?, OP)?;
        if single_closed_loop_isolated && loopuse::orientation(b, lu_rad_mate)? == OrientType::Same
        {
            make_vertex_loop(b, vu_rad_mate, lu_mate)?;
        } else {
            make_vertex_loop(b, vu_rad_mate, lu_rad_mate)?;
        }
    } else {
        // Normal case: need to remove vertexuses.
        let vu = ner(eu1::vertexuse(b, eu)?, OP)?;
        let v = ner(vertexuse::vertex(b, vu)?, OP)?;
        ignore_status(vertex_remove(b, v, vu, OP))?;
        b.remove_vertexuse_record(vu)?;
        let vu_rad_mate = ner(eu1::vertexuse(b, eu_rad_mate)?, OP)?;
        ignore_status(vertex_remove(b, v, vu_rad_mate, OP))?;
        b.remove_vertexuse_record(vu_rad_mate)?;
    }

    if v_rad_is_strut && !single_closed_loop_isolated {
        let vu_mate = ner(eu1::vertexuse(b, eu_mate)?, OP)?;
        if v_is_strut {
            // Both are strut edges
            make_vertex_loop(b, vu_mate, lu_mate)?;
            let vu_rad = ner(eu1::vertexuse(b, eu_rad)?, OP)?;
            make_vertex_loop(b, vu_rad, lu_rad)?;
        } else {
            make_vertex_loop(b, vu_mate, lu_rad_mate)?;
            let vu_rad = ner(eu1::vertexuse(b, eu_rad)?, OP)?;
            make_vertex_loop(b, vu_rad, lu)?;
        }
    } else {
        // remove pMate Vertex vertex uses
        let vu_mate = ner(eu1::vertexuse(b, eu_mate)?, OP)?;
        let v_mate = ner(vertexuse::vertex(b, vu_mate)?, OP)?;
        ignore_status(vertex_remove(b, v_mate, vu_mate, OP))?;
        b.remove_vertexuse_record(vu_mate)?;
        let vu_rad = ner(eu1::vertexuse(b, eu_rad)?, OP)?;
        ignore_status(vertex_remove(b, v_mate, vu_rad, OP))?;
        b.remove_vertexuse_record(vu_rad)?;
    }

    for x in [eu, eu_mate, eu_rad, eu_rad_mate] {
        ignore_status(edge_remove(b, e, x, OP))?;
    }

    for (l, x) in [
        (lu, eu),
        (lu_mate, eu_mate),
        (lu_rad, eu_rad),
        (lu_rad_mate, eu_rad_mate),
    ] {
        if loopuse::is_edge_loopuse(b, l)?
            && loopuse::eu_or_vu(b, l)? == Some(EntityRef::Edgeuse(x))
        {
            let c = ccw(b, x)?;
            ignore_status(loopuse::set_edgeuse(b, l, c))?;
        }
    }

    if eu_cw == Some(eu) {
        eu_cw = None;
    }
    if eu_mate_cw == Some(eu_mate) {
        eu_mate_cw = None;
    }
    if eu_rad_cw == Some(eu_rad) {
        eu_rad_cw = None;
    }
    if eu_rad_mate_cw == Some(eu_rad_mate) {
        eu_rad_mate_cw = None;
    }

    for x in [eu, eu_mate, eu_rad, eu_rad_mate] {
        delete_edgeuse(b, x)?;
    }
    b.remove_edge_record(e)?;

    if v_is_strut || v_rad_is_strut {
        if !v_is_strut {
            loopuse::collect_edgeuses(b, lu_mate, eu_rad_mate_cw)?;
            loopuse::collect_edgeuses(b, lu_rad, eu_cw)?;
        }
        if !v_rad_is_strut {
            loopuse::collect_edgeuses(b, lu_mate, eu_mate_cw)?;
            loopuse::collect_edgeuses(b, lu_rad, eu_rad_cw)?;
        }
        return Ok(());
    }

    if split_loop {
        // split a loop branch - not merging two faces
        loopuse::collect_edgeuses(b, lu, eu_cw)?;
        loopuse::collect_edgeuses(b, lu_mate, eu_mate_cw)?;
        loopuse::collect_edgeuses(b, lu_rad, eu_rad_cw)?;
        loopuse::collect_edgeuses(b, lu_rad_mate, eu_rad_mate_cw)?;

        if loopuse::orientation(b, lu_mate)? == OrientType::Same {
            let l = loopuse::loop_(b, lu_mate)?;
            let loop_orient = face_compute_loop_orientation(b, face, l)?;
            if loop_orient != OrientType::Same {
                loopuse::collect_edgeuses(b, lu, eu_rad_cw)?;
                loopuse::collect_edgeuses(b, lu_mate, eu_rad_mate_cw)?;
                loopuse::collect_edgeuses(b, lu_rad, eu_cw)?;
                loopuse::collect_edgeuses(b, lu_rad_mate, eu_mate_cw)?;
            }
        }
        return Ok(());
    }

    // Different loops on the two sides (radially) - merging two faces
    if both_inner {
        return Err(SolidError::sm_err(OP));
    }
    let surv_lu = deref(surv_lu, OP)?;
    let del_lu = deref(del_lu, OP)?;
    let surv_f = deref(surv_f, OP)?;
    let del_f = deref(del_f, OP)?;

    if !single_closed_loop_isolated {
        if eu_cw.is_some() {
            loopuse::collect_edgeuses(b, surv_lu, eu_cw)?;
        } else if eu_rad_cw.is_some() {
            loopuse::collect_edgeuses(b, surv_lu, eu_rad_cw)?;
        }
        let surv_lu_other = deref(loopuse::other_loopuse(b, surv_lu)?, OP)?;
        if eu_mate_cw.is_some() {
            loopuse::collect_edgeuses(b, surv_lu_other, eu_mate_cw)?;
        } else if eu_rad_mate_cw.is_some() {
            loopuse::collect_edgeuses(b, surv_lu_other, eu_rad_mate_cw)?;
        }
    }

    let del_loop = loopuse::loop_(b, del_lu)?;
    let del_lu_mate = loopuse::other_loopuse(b, del_lu)?;

    face_remove_loop(b, del_f, del_loop)?;

    if surface_of(b, del_f)? == surface_of(b, surv_f)? {
        let uv_union = face2::uv_domain(b, del_f)?.union(&face2::uv_domain(b, surv_f)?);
        face2::set_uv_domain(b, surv_f, &uv_union)?;
    }

    if let Some(l) = del_loop {
        b.remove_loop_record(l)?;
    }
    if let Some(l) = del_lu_mate {
        b.remove_loopuse_record(l)?;
    }
    b.remove_loopuse_record(del_lu)?;

    // move the remaining loops of the deleted face to the surviving face
    for move_loop in face2::loops(b, del_f, false, None)? {
        face1::remove_loop(b, del_f, move_loop)?;
        if !same_orientation_faces {
            let l_lu = deref(b.loop_(move_loop)?.loopuse, OP)?;
            b.loop_mut(move_loop)?.loopuse = loopuse::other_loopuse(b, l_lu)?;
        }
        face1::insert_loop(b, surv_f, move_loop)?;
    }

    let (fu_del, fu_del_mate) = face2::faceuses(b, del_f)?;
    let fu_del = deref(fu_del, OP)?;
    let fu_del_mate = deref(fu_del_mate, OP)?;
    let s = deref(eu1::faceuse_shell(b, fu_del)?, OP)?;
    ignore_status(shell_remove_faceuse(b, s, fu_del, OP))?;
    let s = deref(eu1::faceuse_shell(b, fu_del_mate)?, OP)?;
    ignore_status(shell_remove_faceuse(b, s, fu_del_mate, OP))?;
    b.remove_faceuse_record(fu_del)?;
    b.remove_faceuse_record(fu_del_mate)?;
    b.remove_face_record(del_f)?;
    Ok(())
}

/// The normal of surface `srf` (named by its face) at the UV point of edge use `eu`
/// at its middle (`NormalizedEvaluate(0.5, TRUE, ..)`, `EvaluateNormal(uv, FALSE,
/// FALSE, ..)`, both statuses ignored: a failed evaluation leaves the undefined
/// vector).
fn edgeuse_normal(b: &Brep, eu: EdgeuseId, srf: Option<FaceId>, op: &'static str) -> Result<Vec3> {
    let uv3 = ignored(edgeuse_normalized_evaluate_uv(b, eu, 0.5))?.unwrap_or(Vec3::UNDEF);
    let uv = Point2::new(uv3.x, uv3.y);
    let f = deref(srf, op)?;
    let s = deref(b.face(f)?.surface.as_ref(), op)?;
    Ok(ignored(s.evaluate_normal(uv, Side::Left, Side::Left))?.unwrap_or(Vec3::UNDEF))
}

/// The lamina branch of [`delete_edge`].
fn delete_lamina_edge(b: &mut Brep, e: EdgeId, eu: EdgeuseId, eu_mate: EdgeuseId) -> Result<()> {
    const OP: &str = OP_DELETE_EDGE;
    let lu = ner(eu1::loopuse(b, eu)?, OP)?;
    let lu_mate = ner(eu1::loopuse(b, eu_mate)?, OP)?;
    let faceuse = ner(loopuse::faceuse(b, lu)?, OP)?;
    let face = ner(eu1::faceuse_face(b, faceuse)?, OP)?;

    ignore_status(face_notify(b, face, NotifyOperation::PreEdit))?;

    let v_is_strut = eu1::is_strut(b, eu)?;
    let v_other_is_strut = eu1::is_strut(b, eu_mate)?;
    if v_is_strut || v_other_is_strut {
        // a nonmanifold edge is a strut edge
        return Err(SolidError::sm_err(OP));
    }

    let eu_cw = eu1::cw_edgeuse(b, eu)?;
    let eu_mate_cw = eu1::cw_edgeuse(b, eu_mate)?;

    let single_closed_loop_isolated = ccw(b, eu)? == Some(eu);

    let c = ccw(b, eu)?;
    link(b, eu_cw, c, OP)?;
    let c = ccw(b, eu_mate)?;
    link(b, eu_mate_cw, c, OP)?;

    if single_closed_loop_isolated {
        let vu = ner(eu1::vertexuse(b, eu)?, OP)?;
        make_vertex_loop(b, vu, lu)?;
        let vu_mate = ner(eu1::vertexuse(b, eu_mate)?, OP)?;
        make_vertex_loop(b, vu_mate, lu_mate)?;
    } else {
        // Normal case: need to remove two vertexuses.
        let vu = ner(eu1::vertexuse(b, eu)?, OP)?;
        let v = ner(vertexuse::vertex(b, vu)?, OP)?;
        ignore_status(vertex_remove(b, v, vu, OP))?;
        b.remove_vertexuse_record(vu)?;
        let vu_mate = ner(eu1::vertexuse(b, eu_mate)?, OP)?;
        let v_mate = ner(vertexuse::vertex(b, vu_mate)?, OP)?;
        ignore_status(vertex_remove(b, v_mate, vu_mate, OP))?;
        b.remove_vertexuse_record(vu_mate)?;
    }

    ignore_status(edge_remove(b, e, eu, OP))?;
    ignore_status(edge_remove(b, e, eu_mate, OP))?;

    if loopuse::eu_or_vu(b, lu)? == Some(EntityRef::Edgeuse(eu)) {
        let c = ccw(b, eu)?;
        ignore_status(loopuse::set_edgeuse(b, lu, c))?;
    }
    if loopuse::eu_or_vu(b, lu_mate)? == Some(EntityRef::Edgeuse(eu_mate)) {
        let c = ccw(b, eu_mate)?;
        ignore_status(loopuse::set_edgeuse(b, lu_mate, c))?;
    }

    delete_edgeuse(b, eu)?;
    delete_edgeuse(b, eu_mate)?;
    b.remove_edge_record(e)?;
    Ok(())
}

/// A new vertex-use shell holding `vu` (`None`: upstream warns that the vertex use
/// has been deleted) in the region of `shell`.
fn new_vertex_shell(b: &mut Brep, shell: ShellId, vu: Option<VertexuseId>) -> Result<()> {
    let s = b.add_shell_record(Shell::default())?;
    shell_set_list(b, s, vu.map(EntityRef::Vertexuse))?;
    if let Some(vu) = vu {
        b.vertexuse_mut(vu)?.owner = VertexuseOwner::Shell(Some(s));
    }
    // else WARN: the vertex use has been deleted
    let region = shell_region(b, shell)?;
    b.shell_mut(s)?.region = region;
    Ok(())
}

/// The wire branch of [`delete_edge`].
fn delete_wire_edge(
    k: &mut Kernel,
    b: &mut Brep,
    e: EdgeId,
    eu: EdgeuseId,
    eu_mate: EdgeuseId,
) -> Result<()> {
    const OP: &str = OP_DELETE_EDGE;
    let shell = ner(eu1::shell(b, eu)?, OP)?;

    ignore_status(edge_remove(b, e, eu, OP))?;
    ignore_status(edge_remove(b, e, eu_mate, OP))?;

    let vu1 = ner(eu1::vertexuse(b, eu)?, OP)?;
    let v1 = ner(vertexuse::vertex(b, vu1)?, OP)?;
    ignore_status(vertex_remove(b, v1, vu1, OP))?;

    let vu2 = ner(eu1::vertexuse(b, eu_mate)?, OP)?;
    let v2 = ner(vertexuse::vertex(b, vu2)?, OP)?;
    ignore_status(vertex_remove(b, v2, vu2, OP))?;

    let mut vu1 = Some(vu1);
    let mut vu2 = Some(vu2);
    for (v, slot) in [(v1, &mut vu1), (v2, &mut vu2)] {
        let Some(vu) = *slot else { continue };
        if vertex::vertexuses(b, v)?.is_empty() {
            ignore_status(vertex_post_insert(b, v, vu))?;
            b.vertexuse_mut(vu)?.owner = VertexuseOwner::Shell(Some(shell));
        } else {
            // the vertex is connected to some other edges: delete the vertex use
            b.remove_vertexuse_record(vu)?;
            *slot = None;
        }
    }

    b.remove_edge_record(e)?;

    if v1 == v2 {
        if vertex::is_shell_vertex(b, v1)? {
            let head = vertex::primary_vertexuse(b, v1)?;
            shell_set_list(b, shell, head.map(EntityRef::Vertexuse))?;
        } else if shell_is_wire_shell(b, shell)? {
            let vu = deref(vertex::primary_vertexuse(b, v1)?, OP)?;
            let head = vertexuse::edgeuse(b, vu)?;
            shell_set_list(b, shell, head.map(EntityRef::Edgeuse))?;
        }
        delete_edgeuse(b, eu)?;
        delete_edgeuse(b, eu_mate)?;
        return Ok(());
    }

    let v1_shell = vertex::is_shell_vertex(b, v1)?;
    let v2_shell = vertex::is_shell_vertex(b, v2)?;
    if v1_shell && v2_shell {
        shell_set_list(b, shell, vu1.map(EntityRef::Vertexuse))?;
        new_vertex_shell(b, shell, vu2)?;
        delete_edgeuse(b, eu)?;
        delete_edgeuse(b, eu_mate)?;
        return Ok(());
    }
    if v1_shell != v2_shell {
        let (vu_new, v_other) = if v1_shell { (vu1, v2) } else { (vu2, v1) };
        new_vertex_shell(b, shell, vu_new)?;
        if shell_is_wire_shell(b, shell)? {
            let edges = vertex::edges(b, v_other, None)?;
            if let Some(&start_wire) = edges.first() {
                // the shell remains a wire shell and needs a new primary edge use
                let head = eu1::primary_edgeuse(b, start_wire)?;
                shell_set_list(b, shell, head.map(EntityRef::Edgeuse))?;
            } else {
                let head = vertex::primary_vertexuse(b, v_other)?;
                shell_set_list(b, shell, head.map(EntityRef::Vertexuse))?;
            }
        }
        delete_edgeuse(b, eu)?;
        delete_edgeuse(b, eu_mate)?;
        return Ok(());
    }

    let mut lock = MarkLock::new(core::slice::from_mut(&mut k.context), 0, ALL_MARKS)?;
    let r = wire_split_shell(k, b, &lock, shell, v1, v2, eu, eu_mate);
    lock.release(core::slice::from_mut(&mut k.context))?;
    r
}

/// The shell traversals of the wire branch of [`delete_edge`], under `lock`.
fn wire_split_shell(
    k: &mut Kernel,
    b: &mut Brep,
    lock: &MarkLock,
    shell: ShellId,
    v1: VertexId,
    v2: VertexId,
    eu: EdgeuseId,
    eu_mate: EdgeuseId,
) -> Result<()> {
    const OP: &str = OP_DELETE_EDGE;
    let mut marks = Marks::default();
    let mut edgeuses1 = Vec::new();
    let mut faceuses1 = Vec::new();
    shell_traversal(
        k,
        b,
        &mut marks,
        v1,
        shell,
        Some(&mut edgeuses1),
        Some(&mut faceuses1),
        lock,
    )?;
    let split_shell = !marks.is_marked(k, Entity::Vertex(v2), lock.mark_type());
    if split_shell {
        let new_shell = b.add_shell_record(Shell::default())?;
        let region = shell_region(b, shell)?;
        b.shell_mut(new_shell)?.region = region;
        let mut edgeuses2 = Vec::new();
        let mut faceuses2 = Vec::new();
        shell_traversal(
            k,
            b,
            &mut marks,
            v2,
            shell,
            Some(&mut edgeuses2),
            Some(&mut faceuses2),
            lock,
        )?;
        if faceuses2.is_empty() {
            shell_set_list(
                b,
                new_shell,
                edgeuses2.first().map(|&x| EntityRef::Edgeuse(x)),
            )?;
        } else {
            // the first face is set later by PostInsert(pFU)
            shell_set_list(b, new_shell, None)?;
        }
        for &x in &edgeuses2 {
            if eu1::is_shell_edgeuse(b, x)? {
                b.edgeuse_mut(x)?.owner = EdgeuseOwner::Shell(Some(new_shell));
            }
        }
        for &fu in &faceuses2 {
            ignore_status(shell_remove_faceuse(b, shell, fu, OP))?;
            let face = b.faceuse(fu)?.face;
            b.faceuse_mut(fu)?.shell = Some(new_shell);
            if let Some(f) = face {
                b.shell_mut(new_shell)?.faces.push(f);
            }
        }
    }
    if shell_list_is(b, shell, EntityRef::Edgeuse(eu))?
        || shell_list_is(b, shell, EntityRef::Edgeuse(eu_mate))?
        || (faceuses1.is_empty() && split_shell)
    {
        if let Some(&first) = edgeuses1.first() {
            shell_set_list(b, shell, Some(EntityRef::Edgeuse(first)))?;
        }
    }
    delete_edgeuse(b, eu)?;
    delete_edgeuse(b, eu_mate)?;
    Ok(())
}

// ---- DeleteVertex --------------------------------------------------------------------------

/// `SmBrep::DeleteVertex`: deletes a vertex with no edges: a shell vertex (with its
/// shell when the vertex is the shell's) or the vertex of vertex loops (with the
/// loops and their loop uses).
///
/// # Errors
///
/// `SM_ERR` ([`SolidError::Upstream`]) when editing is not enabled or the vertex has
/// edges; the null-pointer status where upstream checks a face;
/// [`SolidError::InvalidInput`] where upstream dereferences null, casts a vertex use's
/// owner to a loop use it is not, or reads past its vertex use pairs;
/// [`SolidError::Unported`] from the callees not ported yet.
pub fn delete_vertex(
    k: &mut Kernel,
    b: &mut Brep,
    env: &mut DeleteEnv<'_>,
    v: VertexId,
) -> Result<()> {
    const OP: &str = OP_DELETE_VERTEX;
    // check state - m_bEditingEnabled must be on
    if !b.editing_enabled() {
        return Err(SolidError::sm_err(OP));
    }

    // inform the public of upcoming change
    ignore_status(brep_notify(b, NotifyOperation::PreEdit))?;
    ignore_status(notify_vertex(k, b, env, v))?;
    ignore_status(brep_notify(b, NotifyOperation::RmFromBrep))?;

    // vertices should be disconnected from edges prior to delete
    if !vertex::edges(b, v, None)?.is_empty() {
        return Err(SolidError::sm_err(OP));
    }

    let vertexuses = vertex::vertexuses(b, v)?;
    if vertexuses.is_empty() {
        // WARN: deleting a vertex with no vertexuses
        b.remove_vertex_record(v)?;
        return Ok(());
    }

    if vertex::is_shell_vertex(b, v)? {
        let vu = deref(vertexuses.first().copied(), OP)?;
        let VertexuseOwner::Shell(shell) = b.vertexuse(vu)?.owner else {
            return Err(SolidError::invalid_input(
                OP,
                "shell vertex use without a shell",
            ));
        };
        let shell = deref(shell, OP)?;
        // The vertex to delete may not actually own a shell.
        if shell_list_is(b, shell, EntityRef::Vertexuse(vu))? {
            let region = shell_region(b, shell)?;
            ignore_status(region_remove_shell(b, region, shell))?;
            b.remove_shell_record(shell)?;
        }
        ignore_status(vertex_remove(b, v, vu, OP))?;
        b.remove_vertex_record(v)?;
        b.remove_vertexuse_record(vu)?;
        return Ok(());
    }

    // Must be loop vertex (can be connected to 1 or more faces)
    for pair in vertexuses.chunks(2) {
        let [vu1, vu2] = *pair else {
            return Err(SolidError::invalid_input(OP, "odd number of vertex uses"));
        };
        let lu_of = |b: &Brep, vu: VertexuseId| -> Result<LoopuseId> {
            match b.vertexuse(vu)?.owner {
                VertexuseOwner::Loopuse(lu) => deref(lu, OP),
                _ => Err(SolidError::invalid_input(
                    OP,
                    "vertex use owner is not a loop use",
                )),
            }
        };
        let lu1 = lu_of(b, vu1)?;
        let lu2 = lu_of(b, vu2)?;
        let fu1 = deref(loopuse::faceuse(b, lu1)?, OP)?;
        let f = ner(eu1::faceuse_face(b, fu1)?, OP)?;

        ignore_status(face_notify(b, f, NotifyOperation::PreEdit))?;

        // remove and delete vertex to loop connections
        ignore_status(faceuse_remove(b, fu1, lu1, OP))?;
        let fu2 = deref(loopuse::faceuse(b, lu2)?, OP)?;
        ignore_status(faceuse_remove(b, fu2, lu2, OP))?;
        if let Some(l) = loopuse::loop_(b, lu1)? {
            b.remove_loop_record(l)?;
        }
        loopuse::set_loop(b, lu1, None)?;
        b.remove_loopuse_record(lu1)?;
        b.remove_loopuse_record(lu2)?;
        ignore_status(vertex_remove(b, v, vu1, OP))?;
        ignore_status(vertex_remove(b, v, vu2, OP))?;
        b.remove_vertexuse_record(vu1)?;
        b.remove_vertexuse_record(vu2)?;
    }

    // delete unconnected vertex
    b.remove_vertex_record(v)?;
    Ok(())
}

// ---- StitchFaces ---------------------------------------------------------------------------

/// The outputs of [`stitch_faces`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StitchOutcome {
    /// `rlStitchedEdges`: the number of edges stitched.
    pub stitched_edges: u32,
    /// `rlLaminaEdges`: the number of lamina edges remaining.
    pub lamina_edges: u32,
    /// `rdMaxVertexEdgeGap`: the largest gap between coincident vertices considered.
    pub max_vertex_edge_gap: f64,
    /// `rdMaxEdgeFaceTrimCurveGap`: the largest gap between coincident edges considered.
    pub max_edge_face_trim_curve_gap: f64,
}

/// `SmStitch::DoStitching` with the options `StitchFaces` sets.
fn do_stitching(
    _b: &mut Brep,
    _xsect_tol_3d: f64,
    _allow_spine_edges: bool,
    _vertices: Option<&[VertexId]>,
    _edges: Option<&[EdgeId]>,
) -> Result<StitchOutcome> {
    // UNPORTED(heal.stitch_p1): SmStitch::SmStitch
    Err(SolidError::unported("SmStitch::SmStitch", "heal.stitch_p1"))
}

/// `SmBrep::StitchFaces`: glues coincident vertex and edge pairs of the B-rep (of the
/// topology of `faces` when given; `None` entries are skipped) within
/// `xsect_tol_3d`, with spine edges allowed when `allow_spine_edges`.
///
/// # Errors
///
/// Those of the stitcher; [`SolidError::Unported`] while the bounding-box lock, the
/// face queries and the stitcher are not ported. The lock is released on every path.
pub fn stitch_faces(
    b: &mut Brep,
    xsect_tol_3d: f64,
    faces: Option<&[Option<FaceId>]>,
    allow_spine_edges: bool,
) -> Result<StitchOutcome> {
    // lock brep bounding box, until goes out of scope (every return below unlocks)
    LockBrepBox::scoped(b, |b| {
        let mut lists = None;
        if let Some(faces) = faces {
            let mut vertices = Vec::new();
            let mut edges = Vec::new();
            for f in faces.iter().flatten() {
                vertices.extend(face_vertices(b, *f)?);
                edges.extend(face_edges(b, *f)?);
            }
            lists = Some((vertices, edges));
        }
        // pass the stitch call along (SER: its status is returned as is)
        do_stitching(
            b,
            xsect_tol_3d,
            allow_spine_edges,
            lists.as_ref().map(|(v, _)| v.as_slice()),
            lists.as_ref().map(|(_, e)| e.as_slice()),
        )
    })
}

// ---- ShrinkGeometry ------------------------------------------------------------------------

/// `SmBrep::ShrinkGeometry`: shrinks each face's UV domain to its trim curves (snapped
/// to the surface domain within a thousandth of its size), trims each surface to its
/// face's domain and each edge curve to its edge's interval.
///
/// # Errors
///
/// Those of the curve trims; [`SolidError::Unported`] while the B-rep and face
/// queries it uses are not ported.
pub fn shrink_geometry(k: &Kernel, b: &mut Brep) -> Result<()> {
    for f in brep_get_faces(b)? {
        let Some(face_domain) = ignored(face_calculate_uv_domain(k, b, f))? else {
            continue;
        };
        let surf_domain = deref(face2::surface(b, f)?, OP_SHRINK)?.natural_uv_domain();
        let tol = face_domain.size().length() / 1000.0;
        let mut min = face_domain.min();
        let mut max = face_domain.max();
        if min.x - surf_domain.min().x < tol {
            min.x = surf_domain.min().x;
        }
        if min.y - surf_domain.min().y < tol {
            min.y = surf_domain.min().y;
        }
        if surf_domain.max().x - max.x < tol {
            max.x = surf_domain.max().x;
        }
        if surf_domain.max().y - max.y < tol {
            max.y = surf_domain.max().y;
        }
        let new_domain = Extent2d::from_points(min, max);
        face2::set_uv_domain(b, f, &new_domain)?;
    }

    for face in brep_get_surfaces(b)? {
        let mut face_domain = face2::uv_domain(b, face)?;
        let surf_domain = deref(face2::surface(b, face)?, OP_SHRINK)?.natural_uv_domain();
        // The status of `Intersect` is dropped upstream, but on disjoint domains it has
        // `Init()`ed the result (SmExtent2d.cpp:414-418), and that undefined extent is
        // the face domain from here on (`AreEqual`, `TrimWithDomain`, `SetUVDomain`).
        face_domain = surf_domain
            .intersect(&face_domain)
            .unwrap_or_else(|_| Extent2d::new());
        if !surf_domain.are_equal(&face_domain, EFF_ZERO) {
            let mut dom = face_domain;
            if let Some(s) = b.face_mut(face)?.surface.as_mut() {
                ignore_status(s.trim_with_domain(k, &mut dom))?;
            }
            ignore_status(face1::remove_uv_trim_curves(b, face))?;
        }
        face2::set_uv_domain(b, face, &face_domain)?;
    }

    for e in brep_get_curves(b)? {
        let mut ivl: Extent1d = edge::interval(b, e)?;
        if let Some(c) = b.edge_mut(e)?.curve.as_mut() {
            // may snap ivl by tol to existing knots
            c.trim(k, &mut ivl, TrimMode::Notify)?;
        }
    }
    Ok(())
}

// ---- ~SmBrep -------------------------------------------------------------------------------

/// `SmBrep::~SmBrep`: enables editing, sets the context's boolean flag for the
/// duration (restored after), notifies the destruction and deletes every face (with
/// its loops and uses), edge (with its edge uses), vertex (with its vertex uses) and
/// region (with its shells).
///
/// # Errors
///
/// [`SolidError::Unported`] while `SmBrep::Notify` and the B-rep queries are not
/// ported.
pub fn destroy(k: &mut Kernel, b: &mut Brep) -> Result<()> {
    // Do these before Notify (in case of user callback).
    b.set_editing_enabled(true);
    let prev_doing_boolean = k.context.doing_boolean();
    k.context.set_doing_boolean(true);
    let r = destroy_records(b);
    k.context.set_doing_boolean(prev_doing_boolean);
    r
}

fn destroy_records(b: &mut Brep) -> Result<()> {
    const OP: &str = OP_DESTROY;
    ignore_status(brep_notify(b, NotifyOperation::Destruction))?;

    // faces
    for f in brep_get_faces(b)? {
        let fu1 = deref(b.face(f)?.faceuse, OP)?;
        let fu2 = b.faceuse(fu1)?.mate;
        for l in face2::loops(b, f, false, None)? {
            let lu1 = b.loop_(l)?.loopuse;
            if let Some(lu1) = lu1 {
                if let Some(lu2) = b.loopuse(lu1)?.mate {
                    b.remove_loopuse_record(lu2)?;
                }
                b.remove_loopuse_record(lu1)?;
            }
            b.remove_loop_record(l)?;
        }
        if let Some(fu2) = fu2 {
            b.remove_faceuse_record(fu2)?;
        }
        b.remove_faceuse_record(fu1)?;
        b.remove_face_record(f)?;
    }

    // edges
    for e in brep_get_edges(b)? {
        for eu in b.edge(e)?.edgeuses.clone() {
            delete_edgeuse(b, eu)?;
        }
        b.remove_edge_record(e)?;
    }

    // vertices
    for v in brep_get_vertices(b)? {
        for vu in b.vertex(v)?.vertexuses.clone() {
            b.remove_vertexuse_record(vu)?;
        }
        b.remove_vertex_record(v)?;
    }

    // regions
    for r in brep_get_regions(b)? {
        for s in region_shells(b, r)? {
            b.remove_shell_record(s)?;
        }
        // the region record itself goes with the B-rep
    }

    unlock_bounding_box(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lock_scope_does_not_run_the_body_when_the_lock_is_unported() {
        let mut b = Brep::new();
        let mut ran = false;
        let r = LockBrepBox::scoped(&mut b, |_| {
            ran = true;
            Ok(())
        });
        assert!(matches!(r, Err(ref e) if e.is_unported()));
        assert!(!ran);
    }

    #[test]
    fn remove_loop_without_loop_notifies_first() {
        // the notification (unported) comes before the null dereference upstream
        let mut b = Brep::new();
        let r = face_remove_loop(&mut b, FaceId::from_index(0), None);
        assert!(matches!(r, Err(ref e) if e.is_unported()));
    }
}
