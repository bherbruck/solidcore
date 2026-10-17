//! `SmBrep` construction and copying, face and edge removal, wire edges, the
//! orientation of trimmed surfaces and the NURBS conversion of plan unit
//! `topology.brep_p3`.
//!
//! Upstream: `source/SMLib/src/SmBrep.cpp`, `source/SMLib/inc/SmBrep.h`,
//! `source/SM_API/src/SmApiQueries.cpp`.
//!
//! | Rust                            | Upstream                                         |
//! |---------------------------------|--------------------------------------------------|
//! | [`new_brep`]                    | `SmBrep::SmBrep(double, double, SmContext*)`     |
//! | [`copy_brep`]                   | `SmBrep::SmBrep(const SmBrep&, SmCopyBrepMap*, SmCopyBrepMap*, SmBoolean, SmBoolean)` |
//! | [`api_brep_copy`]               | `SmApiBrepCopy`                                  |
//! | [`copy_vertices`]               | `SmBrep::CopyVertices`                           |
//! | [`delete_face`], [`DeleteFaceOptions`] | `SmBrep::DeleteFace`                      |
//! | [`make_wire_edge`]              | `SmBrep::MakeWireEdge`                           |
//! | [`make_wire_edge_vertex`]       | `SmBrep::MakeWireEdgeVertex`                     |
//! | [`merge_brep`]                  | `SmBrep::MergeBrep`                              |
//! | [`orient_trimmed_surfaces`]     | `SmBrep::OrientTrimmedSurfaces`                  |
//! | [`squeeze_edge`]                | `SmBrep::SqueezeEdge`                            |
//! | [`turn_to_nurbs`]               | `SmBrep::TurnToNURBS`                            |
//! | `Brep::remove_from_*_list` (crate) | `SmOwningTopology::Remove` on the B-rep's list heads (and `~SmTopology`) |
//!
//! Deviations: out-params become return values; `rpNewEdge`/`rpNewVertex` are only
//! returned on success. The context of a stack-allocated B-rep
//! (`pOptContextForStack`) is the kernel. Notifications go to the caller's
//! [`NotifyEnv`] (the B-rep spatial cache is not kept: no `BrepCacheSlot`).
//!
//! - **Records.** Upstream `delete` of a topology object unlinks it from its owning
//!   list (`~SmTopology`); records stay in their arenas, unreachable, as in
//!   `topology.brep_p2` (faces, edges and vertices leave the B-rep's lists, regions
//!   leave the region list). The copy constructors build a new [`Brep`] and return it;
//!   `MergeBrep` copies from another [`Brep`] (a self-merge, `MergeBrep(*this)`, cannot
//!   be expressed).
//! - **Ignored statuses.** The copy constructor ignores `MergeBrep`'s status (`SE`):
//!   only [`SolidError::Unported`] propagates. `MergeBrep` ignores the list inserts'
//!   statuses (`SE(PostInsert(..))`) as upstream.
//! - **Sliced curve copy.** `MakeWireEdge`/`MakeWireEdgeVertex` copy a curve that
//!   already belongs to an edge with `new SmCurve(*pCurve)`, a base-class copy without
//!   geometry; the port clones the curve with its geometry (`Curve`'s `Clone`, the
//!   copy constructor of the curve kinds). Recorded as `WR-168` in
//!   `plan/upstream-issues.md`; unreached today (both functions stop at
//!   `topology.shell` first).
//! - **Offset surfaces.** `TurnToNURBS` converts `SmOffsetSurface` faces; there is no
//!   offset surface kind yet (`surface.offset_surface`), so every surface that is not
//!   a B-spline kind is skipped, which is upstream's branch for the other kinds.
//! - **Edge use marks.** `SqueezeEdge` marks the squeezed edge uses (and their radial
//!   partners, which are squeezed too); upstream never reads those marks (the check is
//!   commented out) and deletes the edge uses, so the port does not record them.
//! - **No recursion.** Every loop is bounded by its input lists; `DeleteFace`'s shell
//!   split loop removes at least one vertex per pass.

use crate::curve::curve::{Curve, CurveKind};
use crate::error::{EntityRef, Result, SolidError, SM_ERR_INVALID_INPUT};
use crate::geom::context::{MarkLock, MarkType, ALL_MARKS};
use crate::geom::extent1d::Extent1d;
use crate::geom::object::{self, NotifyOperation, ObjectRef};
use crate::geom::tol::{self, SmallTopoType};
use crate::kernel::Kernel;
use crate::math::{Point3, Vec3};
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::nurbs::b_spline_surface::BSplineSurface;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::surface::{Surface, SurfaceKind};
use crate::topology::brep::p1;
use crate::topology::brep::p2::{self, NotifyArg, OldSurface};
use crate::topology::brep_cache::{arena_identity, CacheObject};
use crate::topology::edge::{self, NotifyEnv, SetCurveOptions};
use crate::topology::edgeuse::p1 as eu1;
use crate::topology::face::p1 as face1;
use crate::topology::face::p2 as face2;
use crate::topology::loopuse as lu1;
use crate::topology::proto_topology::p1::CopyBrepMap;
use crate::topology::topology_traverser::{self as tt, Entity, Marks};
use crate::topology::{
    vertex, vertexuse, Brep, EdgeId, EdgeuseId, EdgeuseOwner, FaceId, FaceuseId, LoopuseId,
    LoopuseStart, Region, RegionId, ShellId, VertexId, Vertexuse, VertexuseId, VertexuseOwner,
};

const OP: &str = "topology::brep::p3";

/// Upstream `SM_ERR_ASSERT_FAILURE` (`SmMessages.h`).
const SM_ERR_ASSERT_FAILURE: i32 = 1005;

/// Upstream dereferences null here (no `NER`): the FFI guard rejects the input.
const fn null_deref(reason: &'static str) -> SolidError {
    SolidError::invalid_input(OP, reason)
}

/// `NER(ptr)` on a null pointer: upstream returns `SM_ERR_NULL_POINTER`.
const fn ner() -> SolidError {
    SolidError::null_pointer(OP)
}

/// `SER_MSG(SM_ERR, ..)` / `SER(SM_ERR)`.
const fn ser() -> SolidError {
    SolidError::sm_err(OP)
}

// ---- the B-rep's own lists (SmOwningTopology::Remove on its list heads) ------------------

impl Brep {
    /// Removes `f` from the face list (`m_pFaceListHead->Remove`, also what deleting
    /// the face does through `~SmTopology`).
    pub(crate) fn remove_from_face_list(&mut self, f: FaceId) {
        self.face_list.retain(|&x| x != f);
    }

    /// Removes `e` from the edge list (`~SmTopology` of a deleted edge).
    pub(crate) fn remove_from_edge_list(&mut self, e: EdgeId) {
        self.edge_list.retain(|&x| x != e);
    }

    /// Removes `v` from the vertex list (`~SmTopology` of a deleted vertex).
    pub(crate) fn remove_from_vertex_list(&mut self, v: VertexId) {
        self.vertex_list.retain(|&x| x != v);
    }

    /// `PreInsert(pRegion)` on the B-rep's region list: the region joins the list as
    /// its new head.
    pub(crate) fn pre_insert_region(&mut self, r: RegionId) -> Result<()> {
        self.region_mut(r)?.in_brep = true;
        self.region_list_head = Some(r);
        Ok(())
    }

    /// `PostInsert(pRegion)` on the B-rep's region list: the region joins the list at
    /// its end (it becomes the head of an empty list).
    pub(crate) fn post_insert_region(&mut self, r: RegionId) -> Result<()> {
        let empty = !self.regions.iter().any(|(_, x)| x.in_brep);
        self.region_mut(r)?.in_brep = true;
        if empty {
            self.region_list_head = Some(r);
        }
        Ok(())
    }

    /// `Remove(pRegion)` on the B-rep's region list: the head moves to the next
    /// region when it is removed (none when the list empties).
    pub(crate) fn remove_region_from_list(&mut self, r: RegionId) -> Result<()> {
        let order = p1::regions(self, None)?;
        if self.region_list_head == Some(r) {
            let pos = order.iter().position(|&x| x == r);
            let next = pos
                .and_then(|i| order.get(i + 1).or_else(|| order.first()))
                .copied()
                .filter(|&x| x != r);
            self.region_list_head = next;
        }
        self.region_mut(r)?.in_brep = false;
        Ok(())
    }
}

// ---- unported callees -----------------------------------------------------------------------

/// `SmRegion::SetIsVoid` (unit `topology.shell`).
fn region_set_is_void(b: &mut Brep, r: RegionId, is_void: bool) -> Result<()> {
    let _ = (b, r, is_void);
    // UNPORTED(topology.shell): SmRegion::SetIsVoid
    Err(SolidError::unported(
        "SmRegion::SetIsVoid",
        "topology.shell",
    ))
}

/// `SmRegion::IsVoid` (unit `topology.shell`).
fn region_is_void(b: &Brep, r: RegionId) -> Result<bool> {
    let _ = (b, r);
    // UNPORTED(topology.shell): SmRegion::IsVoid
    Err(SolidError::unported("SmRegion::IsVoid", "topology.shell"))
}

/// `pNewR->m_bIsVoidFlag = pR->m_bIsVoidFlag` (the `SmRegion` member, unit
/// `topology.shell`).
fn copy_region_void_flag(dst: &mut Brep, r: RegionId, src: &Brep, from: RegionId) -> Result<()> {
    let _ = (dst, r, src, from);
    // UNPORTED(topology.shell): SmRegion::m_bIsVoidFlag
    Err(SolidError::unported(
        "SmRegion::m_bIsVoidFlag",
        "topology.shell",
    ))
}

/// `SmRegion::GetShells` (unit `topology.shell`).
fn region_shells(b: &Brep, r: RegionId) -> Result<Vec<ShellId>> {
    let _ = (b, r);
    // UNPORTED(topology.shell): SmRegion::GetShells
    Err(SolidError::unported(
        "SmRegion::GetShells",
        "topology.shell",
    ))
}

/// `pRegion->PostInsert(pShell)` on the region's shell list (unit `topology.shell`).
fn region_post_insert_shell(b: &mut Brep, r: RegionId, s: ShellId) -> Result<()> {
    let _ = (b, r, s);
    // UNPORTED(topology.shell): SmRegion shell list (SmOwningTopology::PostInsert)
    Err(SolidError::unported(
        "SmRegion::PostInsert",
        "topology.shell",
    ))
}

/// `pRegion->Remove(pShell)` on the region's shell list (unit `topology.shell`).
fn region_remove_shell(b: &mut Brep, r: RegionId, s: ShellId) -> Result<()> {
    let _ = (b, r, s);
    // UNPORTED(topology.shell): SmRegion shell list (SmOwningTopology::Remove)
    Err(SolidError::unported("SmRegion::Remove", "topology.shell"))
}

/// What `SmShell::m_tShellType` / `m_pList` say (unit `topology.shell`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ShellList {
    /// `SmFaceuse_TYPE` (`m_pList` the face use list).
    Faceuses,
    /// `SmEdgeuse_TYPE` with its wire edge use.
    Edgeuse(Option<EdgeuseId>),
    /// `SmVertexuse_TYPE` with its vertex use.
    Vertexuse(Option<VertexuseId>),
}

/// `m_tShellType` and `m_pList` of shell `s` (unit `topology.shell`).
fn shell_list(b: &Brep, s: ShellId) -> Result<ShellList> {
    let _ = (b, s);
    // UNPORTED(topology.shell): SmShell::m_tShellType / m_pList
    Err(SolidError::unported("SmShell::GetList", "topology.shell"))
}

/// Sets `m_tShellType` / `m_pList` of shell `s` (`UpdateListSize` included; unit
/// `topology.shell`).
fn set_shell_list(b: &mut Brep, s: ShellId, list: ShellList) -> Result<()> {
    let _ = (b, s, list);
    // UNPORTED(topology.shell): SmShell::m_tShellType / m_pList
    Err(SolidError::unported(
        "SmShell::SetShellType",
        "topology.shell",
    ))
}

/// `m_lListSize` of shell `s` (`SmShell::GetSize`, unit `topology.shell`).
fn shell_list_size(b: &Brep, s: ShellId) -> Result<usize> {
    let _ = (b, s);
    // UNPORTED(topology.shell): SmShell::GetSize
    Err(SolidError::unported("SmShell::GetSize", "topology.shell"))
}

/// `pShell->Remove(pFU)` on the shell's face use list (unit `topology.shell`).
fn shell_remove_faceuse(b: &mut Brep, s: ShellId, fu: FaceuseId) -> Result<()> {
    let _ = (b, s, fu);
    // UNPORTED(topology.shell): SmShell face use list (SmOwningTopology::Remove)
    Err(SolidError::unported("SmShell::Remove", "topology.shell"))
}

/// `pShell->PostInsert(pFU)` on the shell's face use list (unit `topology.shell`).
fn shell_post_insert_faceuse(b: &mut Brep, s: ShellId, fu: FaceuseId) -> Result<()> {
    let _ = (b, s, fu);
    // UNPORTED(topology.shell): SmShell face use list (SmOwningTopology::PostInsert)
    Err(SolidError::unported(
        "SmShell::PostInsert",
        "topology.shell",
    ))
}

/// Copies the list links of a face use (`m_pListOwner`, `m_pNext`, `m_pLast` through
/// the copy map) into `fu` (unit `topology.shell`).
fn copy_faceuse_links(
    dst: &mut Brep,
    fu: FaceuseId,
    src: &Brep,
    from: FaceuseId,
    map: &p1::ObjectMap,
) -> Result<()> {
    let _ = (dst, fu, src, from, map);
    // UNPORTED(topology.shell): SmShell face use list (m_pListOwner / m_pNext / m_pLast)
    Err(SolidError::unported(
        "SmShell::PostInsert",
        "topology.shell",
    ))
}

/// `SmShell::TransferChildrenTo(pTo, NULL)` (unit `topology.shell`).
fn shell_transfer_children(b: &mut Brep, from: ShellId, to: ShellId) -> Result<()> {
    let _ = (b, from, to);
    // UNPORTED(topology.shell): SmShell::TransferChildrenTo
    Err(SolidError::unported(
        "SmShell::TransferChildrenTo",
        "topology.shell",
    ))
}

/// `SmFace::Notify(operation, data1, data2, data3)` of face `f` (unit
/// `topology.face_p3`).
fn face_notify(
    b: &mut Brep,
    f: FaceId,
    env: &mut NotifyEnv<'_>,
    operation: NotifyOperation,
    data: [Option<NotifyArg>; 3],
) -> Result<()> {
    let _ = (b, f, env, operation, data);
    // UNPORTED(topology.face_p3): SmFace::Notify
    Err(SolidError::unported("SmFace::Notify", "topology.face_p3"))
}

/// `SmFace::GetVertices` (unit `topology.face_p4`).
fn face_vertices(b: &Brep, f: FaceId) -> Result<Vec<VertexId>> {
    let _ = (b, f);
    // UNPORTED(topology.face_p4): SmFace::GetVertices
    Err(SolidError::unported(
        "SmFace::GetVertices",
        "topology.face_p4",
    ))
}

/// `SmBrep::MakeShellVertex(pRegion, crPoint, rpNewShell, rpNewVertex)` (unit
/// `topology.brep_p9`).
fn make_shell_vertex(
    b: &mut Brep,
    region: Option<RegionId>,
    point: Point3,
) -> Result<(ShellId, VertexId)> {
    let _ = (b, region, point);
    // UNPORTED(topology.brep_p9): SmBrep::MakeShellVertex
    Err(SolidError::unported(
        "SmBrep::MakeShellVertex",
        "topology.brep_p9",
    ))
}

/// `SmBrep::DeleteEdge(pEdge, pOptSurfaceToKeep)` (unit `topology.brep_p4`;
/// `surface_to_keep` is `NULL`, the default, at every call here).
fn delete_edge(
    k: &mut Kernel,
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    e: EdgeId,
    surface_to_keep: Option<FaceId>,
) -> Result<()> {
    let _ = (k, b, env, e, surface_to_keep);
    // UNPORTED(topology.brep_p4): SmBrep::DeleteEdge
    Err(SolidError::unported(
        "SmBrep::DeleteEdge",
        "topology.brep_p4",
    ))
}

/// `SmBrep::DeleteVertex(pVertex)` (unit `topology.brep_p4`).
fn delete_vertex(k: &mut Kernel, b: &mut Brep, env: &mut NotifyEnv<'_>, v: VertexId) -> Result<()> {
    let _ = (k, b, env, v);
    // UNPORTED(topology.brep_p4): SmBrep::DeleteVertex
    Err(SolidError::unported(
        "SmBrep::DeleteVertex",
        "topology.brep_p4",
    ))
}

/// `SmBrep::CalculateBoundingBox(rBox, bApprox, bTight, bExpandByZoneTol3d)` (unit
/// `query.brep`).
fn brep_bounding_box(
    k: &Kernel,
    b: &Brep,
    approx: bool,
    tight: bool,
    expand_by_zone_tol_3d: bool,
) -> Result<crate::geom::extent3d::Extent3d> {
    let _ = (k, b, approx, tight, expand_by_zone_tol_3d);
    // UNPORTED(query.brep): SmBrep::CalculateBoundingBox
    Err(SolidError::unported(
        "SmBrep::CalculateBoundingBox",
        "query.brep",
    ))
}

/// `SmEdgeuse::NormalizedEvaluate(t, bParameterSpaceEval, rPoint, &rFirstDeriv,
/// bOKToMakeUVTrimCurve)` (unit `topology.edgeuse_p3`): the point and first derivative
/// at normalized parameter `t`.
fn normalized_evaluate(
    k: &Kernel,
    b: &mut Brep,
    eu: EdgeuseId,
    t: f64,
    parameter_space: bool,
    ok_to_make_uv_trim_curve: bool,
) -> Result<(Point3, Vec3)> {
    let _ = (k, b, eu, t, parameter_space, ok_to_make_uv_trim_curve);
    // UNPORTED(topology.edgeuse_p3): SmEdgeuse::NormalizedEvaluate
    Err(SolidError::unported(
        "SmEdgeuse::NormalizedEvaluate",
        "topology.edgeuse_p3",
    ))
}

// ---- helpers --------------------------------------------------------------------------------

/// `pRegion` or, for `None`, `GetInfiniteRegion()`.
fn region_or_infinite(b: &Brep, region: Option<RegionId>) -> Option<RegionId> {
    region.or_else(|| b.infinite_region())
}

/// `SmBrep::Notify` of this B-rep without spatial cache.
fn brep_notify(
    b: &Brep,
    env: &mut NotifyEnv<'_>,
    operation: NotifyOperation,
    data: [Option<NotifyArg>; 3],
) -> Result<()> {
    p2::notify(b, env, None, operation, data)
}

/// The topology entity `e` as a notification argument.
#[allow(clippy::unnecessary_wraps)]
const fn arg(e: EntityRef) -> Option<NotifyArg> {
    Some(NotifyArg::Entity(e))
}

/// The B-rep as a notification argument.
#[allow(clippy::unnecessary_wraps)]
const fn brep_arg(b: &Brep) -> Option<NotifyArg> {
    Some(NotifyArg::Object(p2::brep_ref(b)))
}

/// The edge use's vertex (`pEU->GetVertexuse()->GetVertex()`), `NER` on each.
fn eu_vertex(b: &Brep, eu: EdgeuseId) -> Result<(VertexuseId, VertexId)> {
    let vu = eu1::vertexuse(b, eu)?.ok_or_else(ner)?;
    let v = vertexuse::vertex(b, vu)?.ok_or_else(ner)?;
    Ok((vu, v))
}

/// `pV->Remove(pVU)`: takes the vertex use off the vertex's list.
fn vertex_remove_vertexuse(b: &mut Brep, v: VertexId, vu: VertexuseId) -> Result<()> {
    b.vertex_mut(v)?.vertexuses.retain(|&x| x != vu);
    Ok(())
}

/// `pE->Remove(pEU)`: takes the edge use off the edge's radial list.
fn edge_remove_edgeuse(b: &mut Brep, e: EdgeId, eu: EdgeuseId) -> Result<()> {
    b.edge_mut(e)?.edgeuses.retain(|&x| x != eu);
    Ok(())
}

/// `pFU->Remove(pLU)`: takes the loop use off the face use's list.
fn faceuse_remove_loopuse(b: &mut Brep, fu: FaceuseId, lu: LoopuseId) -> Result<()> {
    b.faceuse_mut(fu)?.loopuses.retain(|&x| x != lu);
    Ok(())
}

/// `delete pFace`: notifies its destruction (`SmFace::Notify`, unit `topology.face_p3`)
/// and drops the surface it owns (the record leaves the B-rep's face list).
fn destroy_face(b: &mut Brep, env: &mut NotifyEnv<'_>, f: FaceId) -> Result<()> {
    face_notify(
        b,
        f,
        env,
        NotifyOperation::Destruction,
        [arg(EntityRef::Face(f)), None, None],
    )?;
    b.remove_from_face_list(f);
    let face = b.face_mut(f)?;
    face.faceuse = None;
    let owned = face
        .surface
        .as_ref()
        .is_some_and(|s| s.owner() == Some(arena_identity(CacheObject::Face(f))));
    if owned {
        face.surface = None;
    }
    Ok(())
}

// ---- construction -----------------------------------------------------------------------------

/// `SmBrep(dThisModelSizeEstimate, dThisLargeSmallSizeRatio, pOptContextForStack)`: a
/// new B-rep with the given size estimate and large/small ratio (upstream defaults
/// [`tol::USE_DEFAULT`]), the context's zone tolerance, editing disabled, and an
/// infinite void region as its only region; the construction is reported to the
/// context's callbacks (`SmObject::Notify`).
///
/// # Errors
///
/// [`SolidError::Unported`]: making the infinite region void is
/// `SmRegion::SetIsVoid` (`topology.shell`).
pub fn new_brep(
    k: &mut Kernel,
    model_size_estimate: f64,
    large_small_size_ratio: f64,
) -> Result<Brep> {
    let mut b = Brep::new();

    // When asked (should be extremly rare) set SystemTolerance sizes
    b.set_this_model_size_estimate(k, model_size_estimate);
    b.set_this_large_small_size_ratio(k, large_small_size_ratio);
    b.tolerance = tol::context_zone_tol_3d(k, Some(&k.context), SmallTopoType::NormalSize);

    // Create infinite region and put into brep's region list
    let r = b.add_region_record(Region::default())?;
    b.set_infinite_region(Some(r));
    region_set_is_void(&mut b, r, true)?;
    b.pre_insert_region(r)?;

    // report construction at SmObject::Notify level - skip other levels
    let this = p2::brep_ref(&b);
    object::notify(
        Some(&mut k.context),
        NotifyOperation::Construction,
        this,
        Some(this),
        None,
        None,
    );
    Ok(b)
}

/// The copy constructor `SmBrep(crBrepToCopy, pOptCopyBrepMap, pOptReverseMap,
/// bCopyAttributes, bAddAnalytics)`: a new B-rep with `src`'s editing and validation
/// flags, tolerance, size estimate and ratio, and a copy of its topology and geometry
/// ([`merge_brep`]; `add_analytics`, upstream default `TRUE`, recognises analytic
/// surfaces). `copy_map` gets copy to source, `reverse_map` source to copy.
/// `copy_attributes` (default `TRUE`) is unused upstream.
///
/// # Errors
///
/// [`SolidError::Unported`] from [`merge_brep`] and [`p2::set_tolerance`] (other
/// `MergeBrep` failures are ignored, as upstream); those of the notification.
#[allow(clippy::too_many_arguments)]
pub fn copy_brep(
    k: &mut Kernel,
    env: &mut NotifyEnv<'_>,
    src: &Brep,
    copy_map: Option<&mut CopyBrepMap<EntityRef>>,
    reverse_map: Option<&mut CopyBrepMap<EntityRef>>,
    copy_attributes: bool,
    add_analytics: bool,
) -> Result<Brep> {
    let _ = copy_attributes;
    let mut b = Brep::new();
    b.set_editing_enabled(src.editing_enabled());
    b.set_validate_only(src.validate_only());

    // Create infinite region and put into brep's region list
    let r = b.add_region_record(Region::default())?;
    b.set_infinite_region(Some(r));
    b.pre_insert_region(r)?;

    // copy tolerances
    p2::set_tolerance(k, &mut b, src.tolerance(), true, true)?;
    b.model_size_estimate = src.model_size_estimate;
    b.large_small_size_ratio = src.large_small_size_ratio;
    b.tolerance = src.tolerance;

    // do the actual copying (SE: the status is not acted on)
    if let Err(e) = merge_brep(k, env, &mut b, src, copy_map, reverse_map, add_analytics) {
        if e.is_unported() {
            return Err(e);
        }
    }

    // inform the public about the construction and the copy
    brep_notify(
        &b,
        env,
        NotifyOperation::Construction,
        [
            brep_arg(&b),
            Some(NotifyArg::Object(p2::brep_ref(src))),
            None,
        ],
    )?;
    Ok(b)
}

/// `SmApiBrepCopy`: a copy of `brep` that preserves its surface types (the copy
/// constructor without maps, `bAddAnalytics = FALSE`).
///
/// # Errors
///
/// `SM_ERR_INVALID_INPUT` for no B-rep; those of [`copy_brep`].
pub fn api_brep_copy(k: &mut Kernel, env: &mut NotifyEnv<'_>, brep: Option<&Brep>) -> Result<Brep> {
    let Some(brep) = brep else {
        return Err(SolidError::Upstream {
            operation: OP,
            code: SM_ERR_INVALID_INPUT,
            entity: None,
        });
    };
    copy_brep(k, env, brep, None, None, true, false)
}

/// `CopyVertices`: copies each vertex of `src` into `dest` as a shell vertex of its
/// infinite region at the same point, reporting each copy (`SM_NO_COPY`). Returns the
/// new vertices (`pNewVertices`).
///
/// # Errors
///
/// [`SolidError::Unported`] for a non-empty list: `SmBrep::MakeShellVertex` is unit
/// `topology.brep_p9`; [`SolidError::NotFound`] for a dangling id.
pub fn copy_vertices(
    src: &Brep,
    vertices: &[VertexId],
    dest: &mut Brep,
    env: &mut NotifyEnv<'_>,
) -> Result<Vec<VertexId>> {
    let mut out = Vec::new();
    // for every vertex - copy as Shell vertex
    for &v in vertices {
        let coords = vertex::point(src, v)?;
        let inf = dest.infinite_region();

        // make new ShellVertex
        let (_, new_v) = make_shell_vertex(dest, inf, coords)?;

        // inform the public - propogate attributes and report copy
        let this = (env.key_of)(EntityRef::Vertex(v));
        let to = (env.key_of)(EntityRef::Vertex(new_v));
        object::notify(
            env.context.as_deref_mut(),
            NotifyOperation::Copy,
            this,
            Some(to),
            Some(p2::brep_ref(dest)),
            Some(p2::brep_ref(src)),
        );
        out.push(new_v);
    }
    Ok(out)
}

// ---- MergeBrep ----------------------------------------------------------------------------------

/// The copy recorded for `from` cast to the expected kind.
fn mapped<T>(
    from: EntityRef,
    map: &p1::ObjectMap,
    cast: impl Fn(EntityRef) -> Option<T>,
) -> Option<T> {
    p1::get_object_map(from, map).and_then(cast)
}

const fn as_region(e: EntityRef) -> Option<RegionId> {
    match e {
        EntityRef::Region(r) => Some(r),
        _ => None,
    }
}

const fn as_shell(e: EntityRef) -> Option<ShellId> {
    match e {
        EntityRef::Shell(s) => Some(s),
        _ => None,
    }
}

const fn as_loopuse(e: EntityRef) -> Option<LoopuseId> {
    match e {
        EntityRef::Loopuse(x) => Some(x),
        _ => None,
    }
}

const fn as_edgeuse(e: EntityRef) -> Option<EdgeuseId> {
    match e {
        EntityRef::Edgeuse(x) => Some(x),
        _ => None,
    }
}

const fn as_vertexuse(e: EntityRef) -> Option<VertexuseId> {
    match e {
        EntityRef::Vertexuse(x) => Some(x),
        _ => None,
    }
}

const fn as_faceuse(e: EntityRef) -> Option<FaceuseId> {
    match e {
        EntityRef::Faceuse(x) => Some(x),
        _ => None,
    }
}

/// `MergeBrep`: adds a copy of `other`'s topology graph (regions except the infinite
/// one, which is mapped onto this B-rep's, shells, faces, loops, edges, vertices and
/// their uses, with copies of the surfaces, curves and UV trim curves) to `b`, without
/// intersection or coincidence checks. Surfaces are copied with
/// `CopyAndAddAnalytics` when `add_analytics` (upstream default `TRUE`), else with
/// `Copy`. Every new object is reported (`SM_NO_COPY`) once the graph is complete.
/// `copy_map` gets copy to source, `reverse_map` source to copy (`sm_SetObjectMap`).
///
/// # Errors
///
/// [`SolidError::Unported`] for any B-rep with a region: the region void flag and
/// shell lists are unit `topology.shell`; those of the geometry copies (`SER`);
/// [`SolidError::NotFound`] / [`SolidError::InvalidInput`] for a dangling id or a
/// null pointer upstream dereferences.
#[allow(clippy::too_many_lines)]
pub fn merge_brep(
    k: &Kernel,
    env: &mut NotifyEnv<'_>,
    b: &mut Brep,
    other: &Brep,
    mut copy_map: Option<&mut CopyBrepMap<EntityRef>>,
    mut reverse_map: Option<&mut CopyBrepMap<EntityRef>>,
    add_analytics: bool,
) -> Result<()> {
    let _ = k;
    let mut map = p1::ObjectMap::new();
    let set = |orig: EntityRef,
               copy: EntityRef,
               map: &mut p1::ObjectMap,
               cm: &mut Option<&mut CopyBrepMap<EntityRef>>,
               rm: &mut Option<&mut CopyBrepMap<EntityRef>>| {
        p1::set_object_map(orig, Some(copy), map, cm.as_deref_mut(), rm.as_deref_mut());
    };

    // We need to copy everything except the infinite region.
    let my_inf = b.infinite_region();
    let other_inf = other.infinite_region();

    // process Regions and shells
    // Map the infintite regions of the Breps.
    if let Some(o) = other_inf {
        p1::set_object_map(
            EntityRef::Region(o),
            my_inf.map(EntityRef::Region),
            &mut map,
            copy_map.as_deref_mut(),
            reverse_map.as_deref_mut(),
        );
    }
    let regions = p1::regions(other, None)?;
    for &r in &regions {
        let new_r = if let Some(x) = mapped(EntityRef::Region(r), &map, as_region) {
            x
        } else {
            let x = b.add_region_record(Region::default())?;
            set(
                EntityRef::Region(r),
                EntityRef::Region(x),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );
            b.post_insert_region(x)?;
            x
        };

        // copy IsVoid state from old to new region
        copy_region_void_flag(b, new_r, other, r)?;

        // for every ThisRegion->shell
        for s in region_shells(other, r)? {
            let new_s = b.add_shell_record(crate::topology::Shell {
                region: Some(new_r),
                ..crate::topology::Shell::default()
            })?;
            set(
                EntityRef::Shell(s),
                EntityRef::Shell(new_s),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );
            // set up new Shell (SE: status not acted on, except an unported list)
            region_post_insert_shell(b, new_r, new_s)?;
            let ty = match shell_list(other, s)? {
                ShellList::Faceuses => ShellList::Faceuses,
                ShellList::Edgeuse(_) => ShellList::Edgeuse(None),
                ShellList::Vertexuse(_) => ShellList::Vertexuse(None),
            };
            set_shell_list(b, new_s, ty)?;
        }
    }

    // for faces and their loops
    let faces = p1::faces(other, None)?;
    for &f in &faces {
        let fu1 = other
            .face(f)?
            .faceuse
            .ok_or(null_deref("face without face use"))?;
        let fu2 = other
            .faceuse(fu1)?
            .mate
            .ok_or(null_deref("face use without mate"))?;

        // Allocate new face and faceuses
        let new_f = b.add_face_record(crate::topology::Face::new(ShellId::from_index(0)))?;
        let new_fu1 = b.add_faceuse_record(crate::topology::Faceuse::default())?;
        let new_fu2 = b.add_faceuse_record(crate::topology::Faceuse::default())?;
        // (add_face_record joined the face list; upstream's PostInsert below)
        b.remove_from_face_list(new_f);

        // Relate the Faces and Faceuses.
        set(
            EntityRef::Face(f),
            EntityRef::Face(new_f),
            &mut map,
            &mut copy_map,
            &mut reverse_map,
        );
        set(
            EntityRef::Faceuse(fu1),
            EntityRef::Faceuse(new_fu1),
            &mut map,
            &mut copy_map,
            &mut reverse_map,
        );
        set(
            EntityRef::Faceuse(fu2),
            EntityRef::Faceuse(new_fu2),
            &mut map,
            &mut copy_map,
            &mut reverse_map,
        );

        // Set Copy Face data
        let src_face = other.face(f)?;
        let face_surf = src_face
            .surface
            .as_ref()
            .ok_or(null_deref("face without surface"))?;
        let mut new_surface = if add_analytics {
            face_surf.copy_and_add_analytics()?
        } else {
            face_surf.copy()?
        };
        *new_surface.owner_mut() = Some(arena_identity(CacheObject::Face(new_f)));
        let (rect, dom, ftol) = (
            src_face.rectangular_trim,
            src_face.uv_domain,
            src_face.tolerance,
        );
        {
            let nf = b.face_mut(new_f)?;
            nf.rectangular_trim = rect;
            nf.surface = Some(new_surface);
            nf.uv_domain = dom;
            nf.tolerance = ftol;
        }

        // add newFace to target Brep FaceList
        b.face_list.push(new_f);

        // Connect up face and faceuses and set FU fields
        let (o1, o2) = (
            other.faceuse(fu1)?.orientation,
            other.faceuse(fu2)?.orientation,
        );
        {
            let x = b.faceuse_mut(new_fu1)?;
            x.face = Some(new_f);
            x.mate = Some(new_fu2);
            x.orientation = o1;
        }
        {
            let x = b.faceuse_mut(new_fu2)?;
            x.face = Some(new_f);
            x.mate = Some(new_fu1);
            x.orientation = o2;
        }
        b.face_mut(new_f)?.faceuse = Some(if o1 == OrientType::Same {
            new_fu1 // FU1 is primary faceuse
        } else {
            new_fu2 // FU2 is primary faceuse
        });

        // for every face->loop
        for l in face2::loops(other, f, false, None)? {
            let lu1_ = other
                .loop_(l)?
                .loopuse
                .ok_or(null_deref("loop without loop use"))?;
            let lu2_ = other
                .loopuse(lu1_)?
                .mate
                .ok_or(null_deref("loop use without mate"))?;

            // Now allocate new loop and loopuses
            let new_l = b.add_loop_record(crate::topology::Loop {
                face: new_f,
                edges: Vec::new(),
                loopuse: None,
            })?;
            let new_lu1 = b.add_loopuse_record(crate::topology::Loopuse::default())?;
            let new_lu2 = b.add_loopuse_record(crate::topology::Loopuse::default())?;

            // Relate Loops and Loopuses.
            set(
                EntityRef::Loop(l),
                EntityRef::Loop(new_l),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );
            set(
                EntityRef::Loopuse(lu1_),
                EntityRef::Loopuse(new_lu1),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );
            set(
                EntityRef::Loopuse(lu2_),
                EntityRef::Loopuse(new_lu2),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );

            // Connect up faceuses and loopuses and loops
            b.loop_mut(new_l)?.loopuse = Some(if o1 == OrientType::Same {
                new_lu1
            } else {
                new_lu2
            });
            b.face_mut(new_f)?.loops.push(new_l);
            let (s1, s2) = (other.loopuse(lu1_)?, other.loopuse(lu2_)?);
            let start_kind = |s: LoopuseStart| match s {
                LoopuseStart::Unknown => LoopuseStart::Unknown,
                LoopuseStart::Edgeuse(_) => LoopuseStart::Edgeuse(None),
                LoopuseStart::Vertexuse(_) => LoopuseStart::Vertexuse(None),
            };
            let (or1, st1, or2, st2) = (
                s1.orientation,
                start_kind(s1.start),
                s2.orientation,
                start_kind(s2.start),
            );
            {
                let x = b.loopuse_mut(new_lu1)?;
                x.loop_ = Some(new_l);
                x.mate = Some(new_lu2);
                x.faceuse = Some(new_fu1);
                x.orientation = or1;
                x.start = st1;
            }
            {
                let x = b.loopuse_mut(new_lu2)?;
                x.loop_ = Some(new_l);
                x.mate = Some(new_lu1);
                x.faceuse = Some(new_fu2);
                x.orientation = or2;
                x.start = st2;
            }
            b.faceuse_mut(new_fu1)?.loopuses.push(new_lu1);
            b.faceuse_mut(new_fu2)?.loopuses.push(new_lu2);
        }
    }

    // for edges and their edgeuses
    let edges = p1::edges(other, None)?;
    for &e in &edges {
        let src_e = other.edge(e)?;
        let curve = src_e
            .curve
            .as_ref()
            .ok_or(null_deref("edge without curve"))?;
        let mut new_c = curve.copy(k)?;
        let (ivl, etol) = (src_e.interval, src_e.tolerance);
        let new_e = b.add_edge_record(crate::topology::Edge {
            vertices: [VertexId::from_index(0); 2],
            edgeuses: Vec::new(),
            interval: ivl,
            tolerance: etol,
            curve: None,
        })?;
        let _ = new_c.set_owner(Some(arena_identity(CacheObject::Edge(new_e))), None);
        b.edge_mut(new_e)?.curve = Some(new_c);

        // Relate the Edges.
        set(
            EntityRef::Edge(e),
            EntityRef::Edge(new_e),
            &mut map,
            &mut copy_map,
            &mut reverse_map,
        );

        // Now do edgeuses
        for eu in edge::edgeuses(other, e, None, None)? {
            let src_eu = other.edgeuse(eu)?;
            let owner = match src_eu.owner {
                EdgeuseOwner::Unknown => EdgeuseOwner::Unknown,
                EdgeuseOwner::Loopuse(x) => EdgeuseOwner::Loopuse(
                    x.and_then(|x| mapped(EntityRef::Loopuse(x), &map, as_loopuse)),
                ),
                EdgeuseOwner::Shell(x) => {
                    EdgeuseOwner::Shell(x.and_then(|x| mapped(EntityRef::Shell(x), &map, as_shell)))
                }
            };
            let uv = match &src_eu.uv_trim_curve {
                Some(c) => Some(c.copy(k)?),
                None => None,
            };
            let orientation = src_eu.orientation;
            let new_eu = b.add_edgeuse_record(crate::topology::Edgeuse {
                edge: Some(new_e),
                owner,
                orientation,
                ..crate::topology::Edgeuse::default()
            })?;
            // (UV trim curve owners are not represented, as `SetUVTrimCurve`)
            if let Some(c) = uv {
                b.edgeuse_mut(new_eu)?.uv_trim_curve = Some(c);
            }

            // add copied edgeuse to pNewE within the Brep
            b.edge_mut(new_e)?.edgeuses.push(new_eu);

            // Relate the Edgeuses.
            set(
                EntityRef::Edgeuse(eu),
                EntityRef::Edgeuse(new_eu),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );
        }
    }

    // for vertices and their vertexuses
    let vertices = p1::vertices(other, None)?;
    for &v in &vertices {
        let src_v = other.vertex(v)?;
        let (point, vtol) = (src_v.point, src_v.tolerance);
        let new_v = b.add_vertex_record(crate::topology::Vertex::default())?;

        // set copy vertex values (SetPoint notifies a vertex without uses: nothing)
        b.vertex_mut(new_v)?.point = point;
        b.vertex_mut(new_v)?.tolerance = vtol;

        // Relate the Vertices.
        set(
            EntityRef::Vertex(v),
            EntityRef::Vertex(new_v),
            &mut map,
            &mut copy_map,
            &mut reverse_map,
        );

        // for every vertex->vertexuse
        for vu in vertex::vertexuses(other, v)? {
            let new_vu = b.add_vertexuse_record(Vertexuse {
                vertex: Some(new_v),
                ..Vertexuse::default()
            })?;
            b.vertex_mut(new_v)?.vertexuses.push(new_vu);

            // Relate the Vertexuses.
            set(
                EntityRef::Vertexuse(vu),
                EntityRef::Vertexuse(new_vu),
                &mut map,
                &mut copy_map,
                &mut reverse_map,
            );

            // set copied vertexuse values
            let owner = match other.vertexuse(vu)?.owner {
                VertexuseOwner::Unknown => VertexuseOwner::Unknown,
                VertexuseOwner::Edgeuse(x) => VertexuseOwner::Edgeuse(
                    x.and_then(|x| mapped(EntityRef::Edgeuse(x), &map, as_edgeuse)),
                ),
                VertexuseOwner::Loopuse(x) => VertexuseOwner::Loopuse(
                    x.and_then(|x| mapped(EntityRef::Loopuse(x), &map, as_loopuse)),
                ),
                VertexuseOwner::Shell(x) => VertexuseOwner::Shell(
                    x.and_then(|x| mapped(EntityRef::Shell(x), &map, as_shell)),
                ),
            };
            b.vertexuse_mut(new_vu)?.owner = owner;
        }
    }

    // Second pass of copying: the non-hierarchial relationships.
    for &r in &regions {
        for s in region_shells(other, r)? {
            let new_s = mapped(EntityRef::Shell(s), &map, as_shell);
            let list = shell_list(other, s)?;
            let Some(new_s) = new_s else { continue };
            let new_list = match list {
                ShellList::Edgeuse(Some(x)) => {
                    ShellList::Edgeuse(mapped(EntityRef::Edgeuse(x), &map, as_edgeuse))
                }
                ShellList::Vertexuse(Some(x)) => {
                    ShellList::Vertexuse(mapped(EntityRef::Vertexuse(x), &map, as_vertexuse))
                }
                // (an empty list is skipped upstream)
                ShellList::Edgeuse(None) | ShellList::Vertexuse(None) => {
                    continue;
                }
                ShellList::Faceuses => ShellList::Faceuses,
            };
            set_shell_list(b, new_s, new_list)?;
        }
    }

    // for faces and loops
    for &f in &faces {
        let fu1 = other
            .face(f)?
            .faceuse
            .ok_or(null_deref("face without face use"))?;
        let fu2 = other
            .faceuse(fu1)?
            .mate
            .ok_or(null_deref("face use without mate"))?;
        for (from, to) in [
            (fu1, mapped(EntityRef::Faceuse(fu1), &map, as_faceuse)),
            (fu2, mapped(EntityRef::Faceuse(fu2), &map, as_faceuse)),
        ] {
            let to = to.ok_or(null_deref("unmapped face use"))?;
            // set newFU ListOwner, Next, and Last Value
            let shell = other
                .faceuse(from)?
                .shell
                .and_then(|s| mapped(EntityRef::Shell(s), &map, as_shell));
            b.faceuse_mut(to)?.shell = shell;
            copy_faceuse_links(b, to, other, from, &map)?;
        }

        // for every loop
        for l in face2::loops(other, f, false, None)? {
            let lu1_ = other
                .loop_(l)?
                .loopuse
                .ok_or(null_deref("loop without loop use"))?;
            let lu2_ = other
                .loopuse(lu1_)?
                .mate
                .ok_or(null_deref("loop use without mate"))?;
            for from in [lu1_, lu2_] {
                let to = mapped(EntityRef::Loopuse(from), &map, as_loopuse)
                    .ok_or(null_deref("unmapped loop use"))?;
                let start = match other.loopuse(from)?.start {
                    LoopuseStart::Unknown => LoopuseStart::Unknown,
                    LoopuseStart::Edgeuse(x) => LoopuseStart::Edgeuse(
                        x.and_then(|x| mapped(EntityRef::Edgeuse(x), &map, as_edgeuse)),
                    ),
                    LoopuseStart::Vertexuse(x) => LoopuseStart::Vertexuse(
                        x.and_then(|x| mapped(EntityRef::Vertexuse(x), &map, as_vertexuse)),
                    ),
                };
                b.loopuse_mut(to)?.start = start;
            }
        }
    }

    // edges and edgeuse
    for &e in &edges {
        for eu in edge::edgeuses(other, e, None, None)? {
            let new_eu = mapped(EntityRef::Edgeuse(eu), &map, as_edgeuse)
                .ok_or(null_deref("unmapped edge use"))?;
            let src = other.edgeuse(eu)?;
            let (ccw, cw) = if src.ccw.is_some() {
                (
                    src.ccw
                        .and_then(|x| mapped(EntityRef::Edgeuse(x), &map, as_edgeuse)),
                    src.cw
                        .and_then(|x| mapped(EntityRef::Edgeuse(x), &map, as_edgeuse)),
                )
            } else {
                // Wire edge
                (None, None)
            };
            let vu = src
                .vertexuse
                .and_then(|x| mapped(EntityRef::Vertexuse(x), &map, as_vertexuse));
            let x = b.edgeuse_mut(new_eu)?;
            x.ccw = ccw;
            x.cw = cw;
            x.vertexuse = vu;
        }
        // the edge record's vertex pair (`GetVertex` reads it through the edge uses)
        let new_e = mapped(EntityRef::Edge(e), &map, |x| match x {
            EntityRef::Edge(x) => Some(x),
            _ => None,
        })
        .ok_or(null_deref("unmapped edge"))?;
        let [a, c] = other.edge(e)?.vertices;
        let va = mapped(EntityRef::Vertex(a), &map, |x| match x {
            EntityRef::Vertex(x) => Some(x),
            _ => None,
        });
        let vc = mapped(EntityRef::Vertex(c), &map, |x| match x {
            EntityRef::Vertex(x) => Some(x),
            _ => None,
        });
        if let (Some(va), Some(vc)) = (va, vc) {
            b.edge_mut(new_e)?.vertices = [va, vc];
        }
    }

    // Third pass of copying: Notify on all new topology.
    let copy_notify = |env: &mut NotifyEnv<'_>, from: EntityRef, to: Option<EntityRef>| {
        let this = (env.key_of)(from);
        let to = to.map(|t| (env.key_of)(t));
        object::notify(
            env.context.as_deref_mut(),
            NotifyOperation::Copy,
            this,
            to,
            Some(p2::brep_ref(b)),
            Some(p2::brep_ref(other)),
        );
    };
    for &r in &regions {
        for s in region_shells(other, r)? {
            copy_notify(
                env,
                EntityRef::Shell(s),
                p1::get_object_map(EntityRef::Shell(s), &map),
            );
        }
        copy_notify(
            env,
            EntityRef::Region(r),
            p1::get_object_map(EntityRef::Region(r), &map),
        );
    }
    for &f in &faces {
        let fu1 = other
            .face(f)?
            .faceuse
            .ok_or(null_deref("face without face use"))?;
        let fu2 = other
            .faceuse(fu1)?
            .mate
            .ok_or(null_deref("face use without mate"))?;
        for l in face2::loops(other, f, false, None)? {
            let lu1_ = other
                .loop_(l)?
                .loopuse
                .ok_or(null_deref("loop without loop use"))?;
            let lu2_ = other
                .loopuse(lu1_)?
                .mate
                .ok_or(null_deref("loop use without mate"))?;
            for x in [
                EntityRef::Loop(l),
                EntityRef::Loopuse(lu1_),
                EntityRef::Loopuse(lu2_),
            ] {
                copy_notify(env, x, p1::get_object_map(x, &map));
            }
        }
        for x in [
            EntityRef::Face(f),
            EntityRef::Faceuse(fu1),
            EntityRef::Faceuse(fu2),
        ] {
            copy_notify(env, x, p1::get_object_map(x, &map));
        }
    }
    for &e in &edges {
        for eu in edge::edgeuses(other, e, None, None)? {
            let x = EntityRef::Edgeuse(eu);
            copy_notify(env, x, p1::get_object_map(x, &map));
        }
        let x = EntityRef::Edge(e);
        copy_notify(env, x, p1::get_object_map(x, &map));
    }
    for &v in &vertices {
        for vu in vertex::vertexuses(other, v)? {
            let x = EntityRef::Vertexuse(vu);
            copy_notify(env, x, p1::get_object_map(x, &map));
        }
        let x = EntityRef::Vertex(v);
        copy_notify(env, x, p1::get_object_map(x, &map));
    }
    Ok(())
}

// ---- DeleteFace ---------------------------------------------------------------------------------

/// The optional arguments of [`delete_face`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeleteFaceOptions {
    /// `bDeleteEdgesVertices` (default `FALSE`): also delete the edges and vertices
    /// left hanging (wire edges, shell vertices) once the face is gone.
    pub delete_edges_vertices: bool,
    /// `bDoRegionNesting` (default `TRUE`): combine regions and shells as needed.
    pub do_region_nesting: bool,
    /// `pOptRegionToKeep` (default `NULL`): the region kept when two merge.
    pub region_to_keep: Option<RegionId>,
}

impl Default for DeleteFaceOptions {
    fn default() -> Self {
        Self {
            delete_edges_vertices: false,
            do_region_nesting: true,
            region_to_keep: None,
        }
    }
}

/// `DeleteFace`: removes face `f` and its loops and uses from the B-rep. Its lamina
/// edges become wire edges, its other edges lose this face's edge uses; with region
/// nesting the regions on either side merge (keeping the region to keep, else the
/// infinite or void one) and the remaining topology of the shell is split into
/// connected shells. With `delete_edges_vertices` the edges that became wires and the
/// vertices that became shell vertices are deleted too; they are returned
/// (`pOptDeletedEdgesVertices`, stale upstream).
///
/// # Errors
///
/// `SM_ERR` when editing is disabled, the face is not in the B-rep, or the topology is
/// inconsistent; `SM_ERR_NULL_POINTER` for a missing face use, shell, edge use mate,
/// vertex use or vertex (`NER`); [`SolidError::Unported`] for every face in the
/// B-rep: `SmFace::Notify` (`topology.face_p3`), then `SmFace::GetVertices`
/// (`topology.face_p4`), the shell and region lists (`topology.shell`),
/// `DeleteEdge`/`DeleteVertex` (`topology.brep_p4`).
#[allow(clippy::too_many_lines)]
pub fn delete_face(
    k: &mut Kernel,
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    marks: &mut Marks,
    f: FaceId,
    options: DeleteFaceOptions,
) -> Result<Vec<EntityRef>> {
    let DeleteFaceOptions {
        delete_edges_vertices,
        do_region_nesting,
        region_to_keep,
    } = options;
    // init output
    let mut deleted = Vec::new();

    // check state - m_bEditingEnabled must be on
    if !b.editing_enabled() {
        return Err(ser());
    }

    // Check if face is in brep
    if !p1::faces(b, None)?.contains(&f) {
        return Err(ser());
    }

    // tell Brep it's about to change
    brep_notify(b, env, NotifyOperation::PreEdit, [brep_arg(b), None, None])?;
    face_notify(
        b,
        f,
        env,
        NotifyOperation::PreEdit,
        [arg(EntityRef::Face(f)), None, None],
    )?;
    brep_notify(
        b,
        env,
        NotifyOperation::RmFromBrep,
        [arg(EntityRef::Face(f)), brep_arg(b), None],
    )?;

    // Get all vertices for later new shell detection
    let mut verts: Vec<Option<VertexId>> = face_vertices(b, f)?.into_iter().map(Some).collect();
    let orig_vertices: Vec<VertexId> = verts.iter().flatten().copied().collect();

    let fu = face2::upward_faceuse(b, f)?.ok_or_else(ner)?;
    let other_fu = eu1::faceuse_mate(b, fu)?.ok_or(null_deref("face use without mate"))?;

    let mut shell = eu1::faceuse_shell(b, fu)?.ok_or_else(ner)?;
    let mut other_shell = eu1::faceuse_shell(b, other_fu)?;

    // Take care of the case where we have two Regions being combined as a result of
    // this operation.
    if do_region_nesting && Some(shell) != other_shell {
        let os = other_shell.ok_or(null_deref("face use without shell"))?;
        let other_region = eu1::shell_region(b, os)?;
        if let Some(keep) = region_to_keep {
            if Some(keep) == other_region {
                // Swap shells -- keep the specified Region
                shell = eu1::faceuse_shell(b, other_fu)?.ok_or_else(ner)?;
                other_shell = Some(eu1::faceuse_shell(b, fu)?.ok_or_else(ner)?);
            }
        } else {
            let other_r = other_region.ok_or(null_deref("shell without region"))?;
            if other_region == b.infinite_region()
                || (region_is_void(b, other_r)?
                    && eu1::shell_region(b, shell)? != b.infinite_region())
            {
                // Swap shells -- keep the infinite (or void) region.
                shell = eu1::faceuse_shell(b, other_fu)?.ok_or_else(ner)?;
                other_shell = Some(eu1::faceuse_shell(b, fu)?.ok_or_else(ner)?);
            }
        }
        let os = other_shell.ok_or(null_deref("face use without shell"))?;

        // Transfer all stuff from other shell to shell
        tt::remove_shell(b, os, shell)?;

        // Now pOtherShell should be empty - remove it from region and delete it.
        let surviving = eu1::shell_region(b, shell)?.ok_or(null_deref("shell without region"))?;
        let del_region = eu1::shell_region(b, os)?.ok_or(null_deref("shell without region"))?;
        if surviving == del_region {
            return Err(ser()); // Should not happen
        }
        region_remove_shell(b, del_region, os)?;
        other_shell = None;
        for s in region_shells(b, del_region)? {
            region_remove_shell(b, del_region, s)?;
            region_post_insert_shell(b, surviving, s)?;
        }
        if Some(del_region) == b.infinite_region() {
            b.set_infinite_region(Some(surviving));
            region_set_is_void(b, surviving, true)?;
        }

        // allow attributes to propagate and inform the public of Region merges
        brep_notify(
            b,
            env,
            NotifyOperation::MergeInBrep,
            [
                arg(EntityRef::Region(surviving)),
                arg(EntityRef::Region(del_region)),
                brep_arg(b),
            ],
        )?;

        // Remove and Delete the DelRegion
        b.remove_region_from_list(del_region)?;
    }

    let mut new_wire_edges: Vec<EdgeId> = Vec::new();
    let mut any_edgeuse: Option<EdgeuseId> = None;
    let mut new_shell_vertex_count = 0usize;

    // for every loop in FaceToBeDeleted - remove use connections to vertices and edges
    // then delete the loop and its loopuses
    for lu in eu1::faceuse_loopuses(b, fu, false, None)? {
        let other_lu = lu1::other_loopuse(b, lu)?.ok_or(null_deref("loop use without mate"))?;
        let l = lu1::loop_(b, lu)?;

        if lu1::is_vertex_loopuse(b, lu)? {
            // Single vertex loop - either remove vertexuses belonging to loops or change
            // them to shell vertexuses.
            let vu = lu1::vertexuse(b, lu)?.ok_or_else(ner)?;
            let v = vertexuse::vertex(b, vu)?.ok_or_else(ner)?;
            let other_vu = lu1::vertexuse(b, other_lu)?.ok_or_else(ner)?;
            let n_vus = vertex::vertexuses(b, v)?.len();
            // Remove vertexuses from the vertex.
            vertex_remove_vertexuse(b, v, other_vu)?;
            vertex_remove_vertexuse(b, v, vu)?;
            if n_vus == 2 {
                // If there were only two VU - convert vertex to a shell vertex
                let new_s = b.add_shell_record(crate::topology::Shell::default())?;
                let r = eu1::shell_region(b, shell)?.ok_or(null_deref("shell without region"))?;
                b.shell_mut(new_s)?.region = Some(r);
                region_post_insert_shell(b, r, new_s)?;
                b.vertex_mut(v)?.vertexuses.insert(0, vu);

                // Link up shell and vertexuse
                set_shell_list(b, new_s, ShellList::Vertexuse(Some(vu)))?;
                b.vertexuse_mut(vu)?.owner = VertexuseOwner::Shell(Some(new_s));

                // count the number of shell vertics made
                new_shell_vertex_count += 1;
            }
        } else {
            for eu in lu1::edgeuses(b, lu, None, None)? {
                if any_edgeuse.is_none() {
                    any_edgeuse = Some(eu);
                }
                let mate = eu1::mate(b, eu)?.ok_or_else(ner)?;
                let e = eu1::edge(b, eu)?.ok_or_else(ner)?;
                if eu1::face(b, mate)? != eu1::face(b, eu)? {
                    return Err(ser());
                }

                if edge::is_lamina(b, e)? {
                    // Don't delete edgeuses convert them to wire edge edgeuses
                    for x in [eu, mate] {
                        let r = b.edgeuse_mut(x)?;
                        r.owner = EdgeuseOwner::Shell(Some(shell));
                        r.ccw = None;
                        r.cw = None;
                    }
                    // TRUE = delete existing UVTrimCurve
                    eu1::set_uv_trim_curve(b, eu, eu1::UvTrimCurveArg::Null, 0.0, true, None)?;

                    // remember the chanded edges
                    new_wire_edges.push(e);
                } else {
                    // Easy just remove and delete edgeuses/vertexuses from the list
                    edge_remove_edgeuse(b, e, eu)?;
                    edge_remove_edgeuse(b, e, mate)?;
                    let (vu1, v1) = eu_vertex(b, eu)?;
                    vertex_remove_vertexuse(b, v1, vu1)?;
                    let (vu2, v2) = eu_vertex(b, mate)?;
                    vertex_remove_vertexuse(b, v2, vu2)?;
                    if any_edgeuse == Some(eu) {
                        any_edgeuse = None;
                    }
                }
            }
        }

        // The target loop and its loopuses are no longer connected to any edges or
        // vertices: remove the loopuse connections from the faceuses.
        if let Some(l) = l {
            b.face_mut(f)?.loops.retain(|&x| x != l);
        }
        let lfu = lu1::faceuse(b, lu)?.ok_or(null_deref("loop use without face use"))?;
        faceuse_remove_loopuse(b, lfu, lu)?;
        let ofu = lu1::faceuse(b, other_lu)?.ok_or(null_deref("loop use without face use"))?;
        faceuse_remove_loopuse(b, ofu, other_lu)?;
    }

    // Take care of face and faceuses - remove from lists
    shell_remove_faceuse(b, shell, fu)?;
    if let Some(os) = other_shell {
        shell_remove_faceuse(b, os, other_fu)?;
    }

    // Don't allow stale pointers to remain in the Faces.
    for x in [fu, other_fu] {
        if let Some(face) = eu1::faceuse_face(b, x)? {
            face2::set_faceuse(b, face, None)?;
        }
    }

    let mut shell_alive = true;
    if do_region_nesting {
        // If there are no vertices and no edges then the entire shell goes away.
        let mut vertex_count = verts.len();
        let mut lock = MarkLock::new(core::slice::from_mut(&mut k.context), 0, ALL_MARKS)?;
        let res = delete_face_nesting(
            k,
            b,
            marks,
            &mut lock,
            shell,
            any_edgeuse,
            &mut verts,
            &mut vertex_count,
            new_shell_vertex_count,
            &mut shell_alive,
        );
        lock.release(core::slice::from_mut(&mut k.context))?;
        res?;
    }

    // It's now OK to remove and delete the FaceToDelete
    destroy_face(b, env, f)?;

    if delete_edges_vertices {
        // delete every edge converted into a wire
        for e in new_wire_edges {
            deleted.push(EntityRef::Edge(e));
            delete_edge(k, b, env, e, None)?;
        }

        // remove vertices that have become shell vertices
        for v in orig_vertices {
            if vertex::is_shell_vertex(b, v)? {
                deleted.push(EntityRef::Vertex(v));
                delete_vertex(k, b, env, v)?;
            }
        }
    }
    let _ = shell_alive;
    Ok(deleted)
}

/// The region nesting part of [`delete_face`] (under its mark lock): drops the shell
/// when nothing is left in it, else splits the topology left in it into connected
/// shells (the last new shell's children go back to the original shell).
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn delete_face_nesting(
    k: &mut Kernel,
    b: &mut Brep,
    marks: &mut Marks,
    lock: &mut MarkLock,
    shell: ShellId,
    any_edgeuse: Option<EdgeuseId>,
    verts: &mut [Option<VertexId>],
    vertex_count: &mut usize,
    new_shell_vertex_count: usize,
    shell_alive: &mut bool,
) -> Result<()> {
    // (unsigned counts upstream)
    let remaining = |c: usize| c.wrapping_sub(new_shell_vertex_count);
    if any_edgeuse.is_none() && remaining(*vertex_count) == 0 {
        let r = eu1::shell_region(b, shell)?.ok_or(null_deref("shell without region"))?;
        region_remove_shell(b, r, shell)?;
        *shell_alive = false;
    } else {
        // See if shell is now a wire shell - if it is point it to an edgeuse
        if shell_list_size(b, shell)? == 0 {
            if let Some(eu) = any_edgeuse {
                set_shell_list(b, shell, ShellList::Edgeuse(Some(eu)))?;
            }
        }
        // (an empty list is only an SE(SM_ERR) upstream)
        let _ = shell_list(b, shell)?;

        // single/multiple shell check
        let mut need_traversal = true;
        for v in verts.iter().flatten().copied() {
            // subtract shell vertices - they already have been given their new shells
            if vertex::is_shell_vertex(b, v)? {
                *vertex_count = vertex_count.wrapping_sub(1);
                continue;
            }
            if need_traversal {
                tt::shell_traversal(k, b, marks, v, shell, None, None, lock)?;
                need_traversal = false;
            }
            // subtract marked vertices - they have a home in the original Shell
            if marks.is_marked(k, Entity::Vertex(v), lock.mark_type()) {
                *vertex_count = vertex_count.wrapping_sub(1);
            }
        }
    }

    // if any vertices no longer connect to pShell - need 1 or more new shells
    if remaining(*vertex_count) == 0 {
        return Ok(());
    }
    lock.new_mark(core::slice::from_mut(&mut k.context))?;
    let mut last_new_shell: Option<ShellId> = None;
    let mut count = verts.len();
    while count > 0 {
        // Now remove all vertices which have been touched by previous traversal.
        for slot in verts.iter_mut() {
            let Some(v) = *slot else { continue };
            if marks.is_marked(k, Entity::Vertex(v), lock.mark_type())
                || vertex::is_shell_vertex(b, v)?
            {
                *slot = None;
                count -= 1;
            }
        }
        if count == 0 {
            break;
        }

        // Now try to find a vertex to traverse again.
        let Some(v) = verts.iter().flatten().copied().next() else {
            break;
        };
        let r = eu1::shell_region(b, shell)?.ok_or(null_deref("shell without region"))?;
        let new_shell = b.add_shell_record(crate::topology::Shell {
            region: Some(r),
            ..crate::topology::Shell::default()
        })?;
        last_new_shell = Some(new_shell);
        region_post_insert_shell(b, r, new_shell)?;
        let mut faceuses = Vec::new();
        let mut edgeuses = Vec::new();
        tt::shell_traversal(
            k,
            b,
            marks,
            v,
            shell,
            Some(&mut edgeuses),
            Some(&mut faceuses),
            lock,
        )?;

        if faceuses.is_empty() && !edgeuses.is_empty() {
            // Wire shell
            set_shell_list(b, new_shell, ShellList::Edgeuse(edgeuses.first().copied()))?;
        } else if faceuses.is_empty() && edgeuses.is_empty() {
            // Vertex shell
            let vu = vertex::primary_vertexuse(b, v)?.ok_or(null_deref("vertex without use"))?;
            set_shell_list(b, new_shell, ShellList::Vertexuse(Some(vu)))?;
            b.vertexuse_mut(vu)?.owner = VertexuseOwner::Shell(Some(new_shell));
        } else {
            // Faceuse shell
            set_shell_list(b, new_shell, ShellList::Faceuses)?;
        }

        // Link all wire edges back to new shell
        for &eu in &edgeuses {
            if eu1::is_shell_edgeuse(b, eu)? {
                b.edgeuse_mut(eu)?.owner = EdgeuseOwner::Shell(Some(new_shell));
                if let ShellList::Edgeuse(Some(x)) = shell_list(b, shell)? {
                    if x == eu {
                        set_shell_list(b, shell, ShellList::Edgeuse(None))?;
                    }
                }
            }
        }

        // Transfer faceuses from the old shell to the new shell
        for &x in &faceuses {
            shell_remove_faceuse(b, shell, x)?;
            shell_post_insert_faceuse(b, new_shell, x)?;
            b.faceuse_mut(x)?.shell = Some(new_shell);
        }
    }

    // If it's still a Faceuse Shell, pShell should be empty and pLastNewShell should
    // contain something.
    if shell_list(b, shell)? == ShellList::Faceuses {
        let ok = shell_list_size(b, shell)? == 0
            && last_new_shell
                .map(|s| shell_list(b, s))
                .transpose()?
                .is_some_and(|l| {
                    !matches!(l, ShellList::Edgeuse(None) | ShellList::Vertexuse(None))
                });
        if !ok {
            return Err(ser());
        }
    }

    // rather than delete pShell - move entities in pLastNewShell to pShell and delete
    // pLastNewShell
    let last = last_new_shell.ok_or(null_deref("no new shell"))?;
    let r = eu1::shell_region(b, shell)?.ok_or(null_deref("shell without region"))?;
    shell_transfer_children(b, last, shell)?;
    region_remove_shell(b, r, last)?;
    Ok(())
}

// ---- wire edges -------------------------------------------------------------------------------

/// The curve the new wire edge gets: `curve` itself when no edge owns it, else a copy
/// (see the module Deviations: upstream's copy is a sliced `SmCurve`).
fn wire_curve(curve: &Curve) -> Curve {
    // (Curve's Clone resets the owner, as the copy constructor)
    curve.clone()
}

/// The identity of an edge's curve for notifications (its owner when owned).
fn curve_ref(env: &NotifyEnv<'_>, e: EdgeId) -> ObjectRef {
    (env.key_of)(EntityRef::Edge(e))
}

/// Binds `curve` (owned by nobody, or copied) to the new edge `e` (`SetCurve(.., FALSE)`
/// then `SetOwner(edge)`).
fn bind_wire_curve(b: &mut Brep, env: &mut NotifyEnv<'_>, e: EdgeId, curve: &Curve) -> Result<()> {
    let mut new_curve = if curve
        .owner()
        .is_some_and(|o| o.type_code == edge_type_code())
    {
        wire_curve(curve)
    } else {
        curve.clone()
    };
    let _ = new_curve.set_owner(None, None);
    let new_ref = curve_ref(env, e);
    edge::set_curve(
        b,
        e,
        Some(new_curve),
        SetCurveOptions {
            delete_old_curve: false,
            ..SetCurveOptions::default()
        },
        env,
        [Some(new_ref), None],
    )?;
    // connect Curve to Edge
    let edge_ref = arena_identity(CacheObject::Edge(e));
    if let Some(c) = b.edge_mut(e)?.curve.as_mut() {
        let _ = c.set_owner(Some(edge_ref), None);
    }
    Ok(())
}

/// `SmEdge_TYPE`.
const fn edge_type_code() -> i64 {
    edge::TYPE
}

/// `MakeWireEdge`: a new wire edge in `region` (the infinite region for `None`) from
/// `start` to `end` along `curve` over `interval` (`OrientType::Same`: the curve runs
/// from `start` to `end`), with the B-rep tolerance; a vertex further than a quarter
/// of the tolerance from its curve end refreshes its tolerance. Reported as
/// `SM_NO_ADD_TO_BREP`.
///
/// # Errors
///
/// Those of [`p2::make_edge_topology`] ([`SolidError::Unported`] for the shell lists,
/// `topology.shell`); those of the curve evaluation and the tolerance refresh.
#[allow(clippy::too_many_arguments)]
pub fn make_wire_edge(
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    region: Option<RegionId>,
    start: VertexId,
    end: VertexId,
    curve: &Curve,
    interval: &Extent1d,
    orientation: OrientType,
) -> Result<EdgeId> {
    // Check for region default.
    let region = region_or_infinite(b, region);

    // make new edge connected to vertices and region
    let e = p2::make_edge_topology(b, region, start, end)?;

    // Set edge parameters
    let t = b.tolerance();
    edge::set_tolerance(b, e, t, true, true)?;
    edge::set_interval(b, e, interval)?;
    edge::set_orientation(b, e, orientation)?;

    // bind the curve (or a copy of it) to the new edge
    bind_wire_curve(b, env, e, curve)?;

    // set vertex tolerances to include edge end points
    let p_start = curve.evaluate_point(interval.min())?;
    let mut tol_sq = b.tolerance();
    tol_sq = (tol_sq / 4.0) * (tol_sq / 4.0);
    if p_start.distance_squared(vertex::point(b, start)?) > tol_sq {
        vertex::refresh_tolerance(b, start, false)?;
    }
    let p_end = curve.evaluate_point(interval.max())?;
    if p_end.distance_squared(vertex::point(b, end)?) > tol_sq {
        vertex::refresh_tolerance(b, end, false)?;
    }

    // Notify of construction of new edge
    brep_notify(
        b,
        env,
        NotifyOperation::AddToBrep,
        [arg(EntityRef::Edge(e)), brep_arg(b), None],
    )?;
    Ok(e)
}

/// `MakeWireEdgeVertex`: a new wire edge in `region` (the infinite region for `None`)
/// from `start` to a new vertex at `end_point` along `curve` over `interval`, both with
/// the B-rep tolerance; both vertices refresh their tolerances. Reported as
/// `SM_NO_ADD_TO_BREP` (vertex, then edge). Returns `(edge, vertex)`.
///
/// # Errors
///
/// Those of [`p2::make_edge_vertex_topology`] ([`SolidError::Unported`] for the shell
/// lists, `topology.shell`); those of the tolerance refresh.
#[allow(clippy::too_many_arguments)]
pub fn make_wire_edge_vertex(
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    region: Option<RegionId>,
    start: VertexId,
    curve: &Curve,
    interval: &Extent1d,
    orientation: OrientType,
    end_point: Point3,
) -> Result<(EdgeId, VertexId)> {
    // Check for region default.
    let region = region_or_infinite(b, region);

    let (e, v) = p2::make_edge_vertex_topology(b, region, start)?;
    let vref = (env.key_of)(EntityRef::Vertex(v));
    let uref = |vu: VertexuseId| (env.key_of)(EntityRef::Vertexuse(vu));
    vertex::set_point(
        b,
        v,
        end_point,
        &mut *env.records,
        &mut *env.attributes,
        env.context.as_deref_mut(),
        vref,
        &uref,
    )?;

    // Set edge and vertex tolerance vector
    let t = b.tolerance();
    edge::set_tolerance(b, e, t, true, true)?;
    vertex::set_tolerance(b, v, t, true, true)?;

    // Set up interval and orientation
    edge::set_interval(b, e, interval)?;
    edge::set_orientation(b, e, orientation)?;

    // bind the curve (or a copy of it) to the new edge
    bind_wire_curve(b, env, e, curve)?;

    vertex::refresh_tolerance(b, start, false)?;
    vertex::refresh_tolerance(b, v, false)?;

    // Notify of construction of new edge and vertex
    brep_notify(
        b,
        env,
        NotifyOperation::AddToBrep,
        [arg(EntityRef::Vertex(v)), brep_arg(b), None],
    )?;
    brep_notify(
        b,
        env,
        NotifyOperation::AddToBrep,
        [arg(EntityRef::Edge(e)), brep_arg(b), None],
    )?;
    Ok((e, v))
}

// ---- SqueezeEdge --------------------------------------------------------------------------------

/// `SqueezeEdge`: squeezes the [vertex-edge-vertex] of edge `e` into the vertex
/// `survivor`: the edge's uses and their vertex uses are removed (loops of just this
/// edge become vertex loops, neighbours skip it, its UV trim curves go), the other
/// vertex's uses move to `survivor`, and the edge and other vertex are deleted. The
/// edge must not be closed.
///
/// # Errors
///
/// `SM_ERR` when `survivor` is not a vertex of `e`, `e` is closed, or a wire edge's
/// other vertex has fewer than two edges (or that edge fewer than two uses);
/// [`SolidError::Unported`]: `SmFace::Notify` (`topology.face_p3`) for an edge in a
/// face, the shell list (`topology.shell`) for a wire edge.
#[allow(clippy::too_many_lines)]
pub fn squeeze_edge(
    k: &mut Kernel,
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    e: EdgeId,
    survivor: VertexId,
) -> Result<()> {
    // get edge vertices
    let v1 = edge::vertex(b, e)?;
    let v2 = edge::other_vertex(b, e, v1)?;

    // check intput - pSurvivingVertex must be one of the edge's vertices
    if v1 != Some(survivor) && v2 != Some(survivor) {
        return Err(ser());
    }

    // check input - edge cannot be a closed loop
    if v1 == v2 {
        return Err(ser());
    }

    // notify Brep and edge they are about to be modified
    brep_notify(b, env, NotifyOperation::PreEdit, [brep_arg(b), None, None])?;
    let eref = (env.key_of)(EntityRef::Edge(e));
    let bref = p2::brep_ref(b);
    edge::notify(
        b,
        e,
        env,
        NotifyOperation::PreEdit,
        [Some(eref), Some(bref), None],
    )?;
    let cref = b.edge(e)?.curve.as_ref().map(|_| curve_ref(env, e));
    brep_notify(
        b,
        env,
        NotifyOperation::RmFromBrep,
        [
            arg(EntityRef::Edge(e)),
            brep_arg(b),
            cref.map(NotifyArg::Object),
        ],
    )?;

    // locals - vertex to delete, edge->edgeuses
    let del_vertex =
        edge::other_vertex(b, e, Some(survivor))?.ok_or(null_deref("edge without other vertex"))?;
    let edgeuses = edge::edgeuses(b, e, None, None)?;

    // Special case if pEdgeToSqueeze is a wire.
    if vertexuse::edge_is_wire(b, e)? {
        for &eu in &edgeuses {
            let Some(shell) = eu1::shell(b, eu)? else {
                continue;
            };
            if shell_list(b, shell)? != ShellList::Edgeuse(Some(eu)) {
                continue;
            }
            // We have to change the Shell's pointer to an edge use on the edge on the
            // other side of the vertex that's being deleted.
            let edges = vertex::edges(b, del_vertex, None)?;
            if edges.len() < 2 {
                return Err(ser());
            }
            let (e0, e1) = (
                edges.first().copied().ok_or_else(ser)?,
                edges.get(1).copied().ok_or_else(ser)?,
            );
            let surviving_edge = if e0 == e { e1 } else { e0 };
            let surviving_eus = edge::edgeuses(b, surviving_edge, None, None)?;
            if surviving_eus.len() < 2 {
                return Err(ser());
            }
            let (s0, s1) = (
                surviving_eus.first().copied().ok_or_else(ser)?,
                surviving_eus.get(1).copied().ok_or_else(ser)?,
            );
            let surviving_eu = if eu1::orientation(b, eu)? == eu1::orientation(b, s0)? {
                s0
            } else {
                s1
            };
            set_shell_list(b, shell, ShellList::Edgeuse(Some(surviving_eu)))?;
        }
    } else {
        // Not a wire edge: so it's in a Loop, on a Face.
        let mut lock = MarkLock::new(core::slice::from_mut(&mut k.context), 0, ALL_MARKS)?;
        let res = squeeze_loops(b, env, &edgeuses, e, survivor);
        lock.release(core::slice::from_mut(&mut k.context))?;
        res?;
    }

    // Remove and destroy all squeezeEdge->edgeuses and corresponding vertexuses.
    for &eu in &edgeuses {
        let (vu, v) = eu_vertex(b, eu)?;
        edge_remove_edgeuse(b, e, eu)?;
        vertex_remove_vertexuse(b, v, vu)?;
    }

    // inform public two vertices are being merged into one
    brep_notify(
        b,
        env,
        NotifyOperation::MergeInBrep,
        [
            arg(EntityRef::Vertex(survivor)),
            arg(EntityRef::Vertex(del_vertex)),
            brep_arg(b),
        ],
    )?;

    // Move all deleteVertex->vertexuses to survivingVertex.
    for vu in vertex::vertexuses(b, del_vertex)? {
        vertex_remove_vertexuse(b, del_vertex, vu)?;
        b.vertex_mut(survivor)?.vertexuses.push(vu);
        b.vertexuse_mut(vu)?.vertex = Some(survivor);
    }

    // Delete squeezeEdge and deleteVertex now that they are removed from the topology
    // graph.
    let vref = (env.key_of)(EntityRef::Vertex(del_vertex));
    vertex::notify(
        b,
        del_vertex,
        &mut *env.records,
        &mut *env.attributes,
        env.context.as_deref_mut(),
        NotifyOperation::Destruction,
        vref,
        [Some(vref), None, None],
        &|vu| ObjectRef {
            type_code: 0,
            key: crate::geom::attribute::ObjectKey(u64::from(vu.index())),
        },
    )?;
    b.remove_from_vertex_list(del_vertex);
    edge::notify(
        b,
        e,
        env,
        NotifyOperation::Destruction,
        [Some(eref), None, None],
    )?;
    let owned = b
        .edge(e)?
        .curve
        .as_ref()
        .is_some_and(|c| c.owner() == Some(arena_identity(CacheObject::Edge(e))));
    if owned {
        b.edge_mut(e)?.curve = None;
    }
    b.remove_from_edge_list(e);
    Ok(())
}

/// The loop part of [`squeeze_edge`]: every edge use of the squeezed edge is taken out
/// of its loop (its UV trim curve deleted; a loop of only this edge becomes a vertex
/// loop at `survivor`).
fn squeeze_loops(
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    edgeuses: &[EdgeuseId],
    e: EdgeId,
    survivor: VertexId,
) -> Result<()> {
    for &eu in edgeuses {
        if let Some(face) = eu1::face(b, eu)? {
            face_notify(
                b,
                face,
                env,
                NotifyOperation::PreEdit,
                [arg(EntityRef::Face(face)), brep_arg(b), None],
            )?;
        }

        // remove UVTrimCurves
        b.edgeuse_mut(eu)?.uv_trim_curve = None;

        // See if squeeze edge is a strut edge in CW / CCW direction.
        let eu_cw = eu1::cw_edgeuse(b, eu)?.ok_or(null_deref("edge use without CW"))?;
        let strut_start = eu1::edge(b, eu_cw)? == Some(e);
        let eu_ccw = eu1::ccw_edgeuse(b, eu)?.ok_or(null_deref("edge use without CCW"))?;
        let strut_end = eu1::edge(b, eu_ccw)? == Some(e);

        // Make sure loop's primary edgeuse is not pEU.
        let lu = eu1::loopuse(b, eu)?.ok_or(null_deref("edge use without loop use"))?;
        if b.loopuse(lu)?.start == LoopuseStart::Edgeuse(Some(eu)) {
            b.loopuse_mut(lu)?.start = LoopuseStart::Edgeuse(Some(eu_cw));
        }

        // When edgeuse is a strut on both ends ... convert loop into a vertex loop.
        if strut_start && strut_end {
            let vu = b.add_vertexuse_record(Vertexuse {
                vertex: Some(survivor),
                owner: VertexuseOwner::Loopuse(Some(lu)),
                ..Vertexuse::default()
            })?;
            b.vertex_mut(survivor)?.vertexuses.insert(0, vu);
            b.loopuse_mut(lu)?.start = LoopuseStart::Vertexuse(Some(vu));
        }

        // Change neighbor edgeuse pointers to skip this squeeze edge.
        b.edgeuse_mut(eu_cw)?.ccw = Some(eu_ccw);
        b.edgeuse_mut(eu_ccw)?.cw = Some(eu_cw);
    }
    Ok(())
}

// ---- TurnToNURBS --------------------------------------------------------------------------------

/// Whether `s` is a kind of `SmBSplineSurface` (`SM_CAST_PTR(SmBSplineSurface, ..)`).
const fn as_bspline_surface(s: &Surface) -> Option<&BSplineSurface> {
    match s.kind() {
        SurfaceKind::BSpline(x) => Some(x),
        SurfaceKind::Plane(x) => Some(x.base()),
        SurfaceKind::SurfOfRevolution(x) => Some(x.base()),
        _ => None,
    }
}

/// `TurnToNURBS`: makes every face surface and edge curve of the B-rep a plain
/// B-spline: a B-spline kind (`SmPlane`, `SmSurfOfRevolution`) gets its B-spline
/// copy (`ReplaceSurface`, which rebuilds the UV trim curves); a curve of another type
/// is copied (B-spline kinds), lifted (`SmCrvOnSurf`) or approximated. Surfaces of other
/// kinds are kept.
///
/// # Errors
///
/// [`SolidError::Unported`] from [`p2::replace_surface`] (`topology.face_p3`) for any
/// face with an analytic surface; those of the curve lift / approximation;
/// `SM_ERR_NULL_POINTER` when a curve could not be made a B-spline (`NER`).
pub fn turn_to_nurbs(k: &Kernel, b: &mut Brep, env: &mut NotifyEnv<'_>) -> Result<()> {
    brep_notify(b, env, NotifyOperation::PreEdit, [brep_arg(b), None, None])?;

    // for every Brep->Surface (GetSurfaces, then the surface's face)
    let faces: Vec<FaceId> = p1::faces(b, None)?
        .into_iter()
        .filter(|&f| {
            b.face(f)
                .ok()
                .and_then(|x| x.surface.as_ref())
                .is_some_and(|s| s.owner() == Some(arena_identity(CacheObject::Face(f))))
        })
        .collect();
    // (GetSurfaces dereferences every face's surface)
    p1::surfaces(b, None)?;
    for f in faces {
        let surface = b
            .face(f)?
            .surface
            .as_ref()
            .ok_or(null_deref("face without surface"))?;
        let Some(bss) = as_bspline_surface(surface) else {
            // not an SmOffsetSurface (no such kind yet): skip
            continue;
        };
        if surface.type_code() == BSplineSurface::TYPE {
            continue;
        }
        let new_bss = Surface::from(bss.clone());
        let face_ref = (env.key_of)(EntityRef::Face(f));
        p2::replace_surface(k, b, f, new_bss, face_ref, OldSurface::Delete, true)?;
    }

    // for every Brep->Curve
    let edges: Vec<EdgeId> = p1::edges(b, None)?
        .into_iter()
        .filter(|&e| {
            b.edge(e)
                .ok()
                .and_then(|x| x.curve.as_ref())
                .is_some_and(|c| c.owner() == Some(arena_identity(CacheObject::Edge(e))))
        })
        .collect();
    // (GetCurves dereferences every edge's curve)
    p1::curves(b, None)?;
    for e in edges {
        let curve = b.edge(e)?.curve.as_ref().ok_or_else(ner)?;
        if curve.type_code() == BSplineCurve::TYPE {
            // SetCurve(pCurve, FALSE) with the edge's own curve: nothing happens
            continue;
        }
        let edge_tol = b.edge(e)?.tolerance;
        let approx_tol = 0.5 * edge_tol;
        let mut bsc: Option<Curve> = None;

        // Copy pCurve if it's a kind of SmBSplineCurve
        if let Some(base) = curve_bspline_base(curve) {
            bsc = Some(Curve::from(base.clone()));
        }

        // LiftCurve is pCurve is an SmCrvOnSurf (status ignored)
        if let CurveKind::CrvOnSurf(cos) = curve.kind() {
            let base = Surface::from(cos.surface().copy());
            let ivl = b.edge(e)?.interval;
            match base.lift_curve(
                k,
                &base.natural_uv_domain(),
                cos.uv_curve(),
                &ivl,
                approx_tol,
                false,
            ) {
                Ok(l) => bsc = Some(Curve::from(l.curve)),
                Err(err) if err.is_unported() => return Err(err),
                Err(_) => {}
            }
        }

        // Approximate pCurve if we don't have a SmBSplineCurve yet (status ignored)
        if bsc.is_none() {
            let (knots, _) = match curve.knots(None) {
                Ok(x) => x,
                Err(err) if err.is_unported() => return Err(err),
                Err(_) => (Vec::new(), Vec::new()),
            };
            let opts = crate::curve::curve::p1::ApproximateCurveOptions::default();
            match curve.approximate_curve(k, &knots, approx_tol, &opts) {
                Ok(a) => bsc = Some(a.curve),
                Err(err) if err.is_unported() => return Err(err),
                Err(_) => {}
            }
        }

        // Must have a BSplineCurve here
        let bsc = bsc.ok_or_else(ner)?;

        // Update the edge: drop the old curve (TRUE = delete preExisting Curve)
        let r = curve_ref(env, e);
        edge::set_curve(
            b,
            e,
            None,
            SetCurveOptions {
                delete_old_curve: true,
                ..SetCurveOptions::default()
            },
            env,
            [None, Some(r)],
        )?;
        let mut new_curve = bsc;
        let _ = new_curve.set_owner(None, None);
        edge::set_curve(
            b,
            e,
            Some(new_curve),
            SetCurveOptions::default(),
            env,
            [Some(r), None],
        )?;
        let edge_ref = arena_identity(CacheObject::Edge(e));
        if let Some(c) = b.edge_mut(e)?.curve.as_mut() {
            let _ = c.set_owner(Some(edge_ref), None);
        }
    }
    Ok(())
}

/// The `SmBSplineCurve` part of a curve that is a kind of it (`SmLine`, the conics).
fn curve_bspline_base(c: &Curve) -> Option<&BSplineCurve> {
    match c.kind() {
        CurveKind::BSpline(x) => Some(x),
        CurveKind::Line(x) => Some(x.base()),
        CurveKind::Ellipse(x) => Some(x.base()),
        CurveKind::Parabola(x) => Some(x.base()),
        CurveKind::Hyperbola(x) => Some(x.base()),
        _ => None,
    }
}

// ---- OrientTrimmedSurfaces ----------------------------------------------------------------------

/// What [`orient_trimmed_surfaces`] found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OrientResult {
    /// `rbMaybeNotClosedSolid`: inconsistent ray votes, the B-rep may not be a closed
    /// solid.
    pub maybe_not_closed_solid: bool,
}

/// `OrientTrimmedSurfaces`: orients the faces so that their normals point out of
/// (`normals_outward`) or into the solid. Unless `just_unify_normals`, each face's
/// orientation is voted by firing rays from points inside it; then the faces reached
/// from it across manifold edges are collected and the smaller set of differently
/// oriented faces is reversed (`SmFace::SwapUV`). `faces_to_swap_uv`
/// (`pOptFacesToSwapUV`) are swapped first and count reversed in the vote.
///
/// # Errors
///
/// `SM_ERR` for a face without edges; `SM_ERR_ASSERT_FAILURE` for a non-orientable
/// B-rep; [`SolidError::Unported`]: the ray votes need the B-rep box (`query.brep`) and
/// edge use evaluation (`topology.edgeuse_p3`), the angle-limited collection
/// `SmEdgeuse::EvaluateBinormal` (`topology.edgeuse_p3`), `SmFace::SwapUV` its
/// `face_p3`/`face_p4` callees.
#[allow(clippy::too_many_lines)]
pub fn orient_trimmed_surfaces(
    k: &mut Kernel,
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    marks: &mut Marks,
    normals_outward: bool,
    just_unify_normals: bool,
    faces_to_swap_uv: Option<&[FaceId]>,
) -> Result<OrientResult> {
    let mut lock = MarkLock::new(core::slice::from_mut(&mut k.context), 0, ALL_MARKS)?;
    let mark = lock.mark_type();
    let res = orient_body(
        k,
        b,
        env,
        marks,
        mark,
        normals_outward,
        just_unify_normals,
        faces_to_swap_uv,
    );
    lock.release(core::slice::from_mut(&mut k.context))?;
    res
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn orient_body(
    k: &Kernel,
    b: &mut Brep,
    env: &mut NotifyEnv<'_>,
    marks: &mut Marks,
    mark: MarkType,
    normals_outward: bool,
    just_unify_normals: bool,
    faces_to_swap_uv: Option<&[FaceId]>,
) -> Result<OrientResult> {
    // init output
    let mut maybe_not_closed = false;

    let tolerance = b.tolerance();
    let faces = p1::faces(b, None)?;
    let mut faceuses_processed = 0usize;
    let mut swap_faces: Vec<FaceId> = Vec::new();
    let in_swap = |f: FaceId| faces_to_swap_uv.is_some_and(|l| l.contains(&f));

    // Check orientation of each face
    for &face in &faces {
        // if we have processed a faceuse from each face, we're done
        if faceuses_processed >= faces.len() {
            break;
        }

        // skipped processed faces
        if marks.is_marked(k, Entity::Face(face), mark) {
            continue;
        }

        // processing this face uses a faceuse
        faceuses_processed += 1;

        // init voting for rayfire
        let mut same_faces: Vec<FaceId> = Vec::new();
        let mut reverse_faces: Vec<FaceId> = Vec::new();

        // shouldn't be necessary, but avoid ever touching this face again
        marks.mark(k, Entity::Face(face), mark);

        // If this face needs swapping, do it. Aids in bookkeeping later.
        if in_swap(face) {
            face1::swap_uv(k, b, face)?;
        }

        let (reverse_votes, same_votes) = if just_unify_normals {
            (0u32, 10u32)
        } else {
            match orient_ray_votes(k, b, marks, mark, face, normals_outward, tolerance)? {
                RayVotes::Skip => continue,
                RayVotes::Votes {
                    reverse,
                    same,
                    maybe_not_closed: m,
                } => {
                    maybe_not_closed |= m;
                    (reverse, same)
                }
            }
        };

        if reverse_votes == same_votes {
            maybe_not_closed = true;
            continue;
        }

        // votes for deciding which set of faces to flip
        let mut n_reverse = 0u32;
        let mut n_same = 0u32;

        let mut faceuses = Vec::new();
        let up = face2::upward_faceuse(b, face)?.ok_or(null_deref("face without face use"))?;
        let start = if reverse_votes > same_votes {
            // collect all connected downward faceuses
            eu1::faceuse_mate(b, up)?.ok_or(null_deref("face use without mate"))?
        } else {
            // collect all connected upward faceuses
            up
        };
        eu1::faceuse_collect_adjacent(b, start, &mut faceuses, None)?;

        // if we get too many faceuses, use a geometric contraint to pick off fewer
        let remaining = faces.len().wrapping_sub(faceuses_processed);
        if faceuses.len() > remaining {
            let angle = crate::math::PI * (11.0 / 12.0);
            faceuses.clear();
            eu1::faceuse_collect_adjacent(b, start, &mut faceuses, Some(angle))?;
            if faceuses.len() > remaining {
                return Err(SolidError::Upstream {
                    operation: OP,
                    code: SM_ERR_ASSERT_FAILURE,
                    entity: None,
                });
            }
        }
        faceuses_processed += faceuses.len();

        if reverse_votes > same_votes {
            if same_votes > 0 {
                maybe_not_closed = true;
            }
            reverse_faces.push(face);
            n_reverse += 1;
        } else {
            if reverse_votes > 0 {
                maybe_not_closed = true;
            }
            same_faces.push(face);
            n_same += 1;
        }

        // Faceuses collected are on the positive side and should have same orientation.
        for &fu in &faceuses {
            let swapping = eu1::faceuse_face(b, fu)?.ok_or(null_deref("face use without face"))?;
            marks.mark(k, Entity::Face(swapping), mark);
            if eu1::faceuse_orientation(b, fu)? == OrientType::Opposite {
                if in_swap(swapping) {
                    n_same += 1;
                } else {
                    n_reverse += 1;
                }
                reverse_faces.push(swapping);
            } else {
                if in_swap(swapping) {
                    n_reverse += 1;
                } else {
                    n_same += 1;
                }
                same_faces.push(swapping);
            }
        }

        // If more sReverse votes than sSame votes, we flip the sSameFaces
        if n_reverse > n_same {
            swap_faces.extend(same_faces);
        } else {
            swap_faces.extend(reverse_faces);
        }
    }

    // SwapUV on faces to be reversed
    brep_notify(b, env, NotifyOperation::PreEdit, [brep_arg(b), None, None])?;
    for f in swap_faces {
        face1::swap_uv(k, b, f)?;
    }
    Ok(OrientResult {
        maybe_not_closed_solid: maybe_not_closed,
    })
}

/// The ray vote of one face in [`orient_trimmed_surfaces`].
enum RayVotes {
    /// The face separates two regions: it is done (swapped unless its upward face use
    /// faces the infinite region).
    Skip,
    /// The reverse / same votes and whether the rays were inconsistent.
    Votes {
        reverse: u32,
        same: u32,
        maybe_not_closed: bool,
    },
}

/// The ray firing of [`orient_trimmed_surfaces`] for `face`: up to two consistent
/// ray pairs from points inside the face (on trimmed iso curves through test points
/// of its edge uses), each voting whether the face normal points into the solid.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn orient_ray_votes(
    k: &Kernel,
    b: &mut Brep,
    marks: &mut Marks,
    mark: MarkType,
    face: FaceId,
    normals_outward: bool,
    tolerance: f64,
) -> Result<RayVotes> {
    // five more or less random points
    let test_param = [0.234_567, 0.567_891, 0.789_123, 0.156_789, 0.856_789];

    // Random vector
    let ray_vec = Vec3::new(1.23456, 2.34567, 3.45678).unitize()?;

    // Brep BoundingBox and BoundingSphere
    let bbox = brep_bounding_box(k, b, true, false, true)?;
    let radius = bbox.sphere_bound().radius;

    let fu = face2::upward_faceuse(b, face)?.ok_or(null_deref("face without face use"))?;
    let other_fu = eu1::faceuse_mate(b, fu)?.ok_or(null_deref("face use without mate"))?;

    // When Faceuse is a boundary between two different Regions
    let r1 = eu1::faceuse_region(b, fu)?;
    let r2 = eu1::faceuse_region(b, other_fu)?;
    if r1 != r2 {
        // when upwardFaceuse is connected to infiniteRegion: no work needs to be done
        if r1 == b.infinite_region() {
            return Ok(RayVotes::Skip);
        }
        // point into different Regions and the first is NOT the infinite region
        face1::swap_uv(k, b, face)?;
        marks.mark(k, Entity::Face(face), mark);
        return Ok(RayVotes::Skip);
    }

    // Get upward facuse and its edgeuses
    let fu = face2::upward_faceuse(b, face)?.ok_or(null_deref("face without face use"))?;
    let edgeuses = eu1::faceuse_edgeuses(b, fu, false, None)?;
    if edgeuses.is_empty() {
        return Err(ser()); // Face needs to have at least one edge
    }

    let mut reverse = 0u32;
    let mut same = 0u32;
    let mut bad_count = 0u32;
    let mut maybe_not_closed = false;
    let mut rays_fired = 0u32;
    'edgeuses: for &eu in &edgeuses {
        if rays_fired >= 2 {
            break;
        }
        for &t in &test_param {
            if rays_fired >= 2 {
                break;
            }
            // get edgeuse testPoint on the face.
            let (pnt, tan) = normalized_evaluate(k, b, eu, t, true, true)?;

            // build temporary isoParam Curve through the testPoint
            let surface = b
                .face(face)?
                .surface
                .as_ref()
                .ok_or(null_deref("face without surface"))?;
            let (dir, at) = if tan.x.abs() > tan.y.abs() {
                (crate::nurbs::b_spline_surface::SurfParamType::U, pnt.x)
            } else {
                (crate::nurbs::b_spline_surface::SurfParamType::V, pnt.y)
            };
            let iso = surface
                .create_iso_parametric_curve(k, dir, at, b.tolerance(), None, false)?
                .curve;

            // trim isoParamCurve to Face
            let trimmed = face2::create_curves_by_trimming(
                k,
                b,
                face,
                Some(tolerance),
                Some(&[Some(iso)]),
                None,
                true,
                false,
            )?;
            let trimmed = trimmed.curves_3d.unwrap_or_default();

            // skip isoParamCurves outside of the face
            let Some(curve) = trimmed.first() else {
                continue;
            };

            // get a point on the face far from its trim edges
            let ivl = curve.natural_interval();
            let point_in_face = curve.evaluate_point(ivl.evaluate(test_param[1]))?;

            // Get Surface Normal for this point.
            let dom = b.face(face)?.uv_domain;
            let sols = surface.global_point_solve::<CacheObject>(
                k,
                None,
                &dom,
                crate::geom::global_solver::SolverOperation::Minimize,
                point_in_face,
                b.tolerance(),
                None,
                crate::geom::global_solver::SolutionRequested::Single,
            )?;
            let sol = *sols.get(0).ok_or_else(ser)?;
            let uv = crate::math::Point2::new(sol.start.param(0)?, sol.start.param(1)?);
            let normal = surface.evaluate_normal(
                uv,
                crate::nurbs::knots::Side::Left,
                crate::nurbs::knots::Side::Left,
            )?;

            // for two rays leaving the surface in either direction
            let mut inside = [false; 2];
            for (ll, slot) in inside.iter_mut().enumerate() {
                let mut direction = if ll == 0 { 1.0 } else { -1.0 };
                let new_ray_vec = if normal.dot(ray_vec) < 0.0 {
                    direction = -direction;
                    (ray_vec - normal).unitize()?
                } else {
                    (ray_vec + normal).unitize()?
                };

                // build a ray line segment
                if !normals_outward {
                    direction = -direction;
                }
                let end = point_in_face + direction * radius * 3.0 * new_ray_vec;
                let line = Curve::from(BSplineCurve::create_line_segment(3, point_in_face, end)?);

                // get all ray/brep intersections (editing off while solving)
                let solutions = brep_curve_solve(k, b, &line, tolerance)?;

                // Count the solutions to determine if we are inside or outside
                let mut points: Vec<Point3> = Vec::new();
                for (num_variables, param) in solutions {
                    // when the solution was a face/curve intersection
                    if num_variables == 3 {
                        let p = line.evaluate_point(param)?;
                        if !points.iter().any(|q| p.distance(*q) < tolerance) {
                            points.push(p);
                        }
                    }
                }

                // count number of face/curve intersections (ignoring the point on this
                // face; an unsigned count upstream)
                let n = points.len().wrapping_sub(1);

                // point is inside if there were an odd number of intersections
                *slot = n % 2 == 1;
            }

            // Make sure rays are consistent
            if inside[0] == inside[1] {
                bad_count += 1;
                if bad_count == 5 {
                    break;
                }
                maybe_not_closed = true;
                continue;
            }

            // Now let's see if the normal is in the right direction
            if inside[0] {
                reverse += 1;
            } else {
                same += 1;
            }

            // increment the successfully cast ray count
            rays_fired += 1;
        }
        if bad_count > 5 {
            break 'edgeuses;
        }
    }
    Ok(RayVotes::Votes {
        reverse,
        same,
        maybe_not_closed,
    })
}

/// `SmTopologySolver::BrepCurveSolve(this, line, its natural interval, SM_SO_INTERSECT,
/// SM_SR_ALL, tolerance, SM_BIG_DOUBLE, NULL)` with `m_bEditingEnabled` off during the
/// solve: the `(number of variables, curve parameter)` of every solution. The B-rep's
/// spatial cache is built for the call (upstream's was dropped by the pre-edit
/// notification).
fn brep_curve_solve(
    k: &Kernel,
    b: &mut Brep,
    line: &Curve,
    tolerance: f64,
) -> Result<Vec<(usize, f64)>> {
    use crate::intersect::topology_solver::p1::{CurveRef, TopologySolver};
    use crate::intersect::topology_solver::p2::TopologySolveOptions;
    let editing = b.editing_enabled();
    b.set_editing_enabled(false);
    let res = (|| -> Result<Vec<(usize, f64)>> {
        let cache = crate::topology::brep_cache::BrepCache::new(crate::geom::a_object::BREP_TYPE)?;
        let keys = |o: CacheObject| arena_identity(o);
        let owners = |_: ObjectRef| None;
        let objects = crate::topology::brep_cache::BrepObjects {
            brep: b,
            brep_id: crate::topology::BrepId::from_index(0),
            cache: &cache,
            keys: &keys,
            owner_of: &owners,
        };
        let sols = TopologySolver::brep_curve_solve(
            k,
            &objects,
            Some(CacheObject::Brep),
            CurveRef {
                curve: line,
                handle: CacheObject::Other(ObjectRef {
                    type_code: BSplineCurve::TYPE,
                    key: crate::geom::attribute::ObjectKey(0),
                }),
            },
            &line.natural_interval(),
            TopologySolveOptions {
                operation: crate::geom::global_solver::SolverOperation::Intersect,
                requested: crate::geom::global_solver::SolutionRequested::All,
                tolerance_3d: tolerance,
                best_answer_so_far_sq: crate::math::BIG_DOUBLE,
                opt_vectors: &[],
            },
        )?;
        let mut out = Vec::new();
        for i in 0..sols.len() {
            let s = sols.get(i).ok_or_else(ser)?;
            out.push((s.num_variables, s.start.param(0)?));
        }
        Ok(out)
    })();
    b.set_editing_enabled(editing);
    res
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn with_env<R>(f: impl FnOnce(&mut NotifyEnv<'_>) -> R) -> R {
        let mut records = BTreeMap::new();
        let mut attributes = crate::geom::attribute::AttributeStore::default();
        let key_of = |e: EntityRef| {
            arena_identity(match e {
                EntityRef::Face(f) => CacheObject::Face(f),
                EntityRef::Edge(x) => CacheObject::Edge(x),
                EntityRef::Vertex(v) => CacheObject::Vertex(v),
                _ => CacheObject::Brep,
            })
        };
        let mut env = NotifyEnv {
            records: &mut records,
            attributes: &mut attributes,
            context: None,
            key_of: &key_of,
        };
        f(&mut env)
    }

    #[test]
    fn api_copy_of_null_is_invalid_input() {
        let mut k = Kernel::default();
        let r = with_env(|env| api_brep_copy(&mut k, env, None));
        assert!(matches!(
            r,
            Err(SolidError::Upstream {
                code: SM_ERR_INVALID_INPUT,
                ..
            })
        ));
    }

    #[test]
    fn constructor_stops_at_the_void_flag() {
        let mut k = Kernel::default();
        let e = new_brep(&mut k, tol::USE_DEFAULT, tol::USE_DEFAULT).unwrap_err();
        assert!(e.is_unported());
    }

    #[test]
    fn delete_face_needs_editing_and_a_face_of_the_brep() {
        let mut k = Kernel::default();
        let mut b = Brep::new();
        let s = b.add_shell().unwrap();
        let f = b.add_face(s).unwrap();
        let mut marks = Marks::default();
        let r = with_env(|env| {
            delete_face(
                &mut k,
                &mut b,
                env,
                &mut marks,
                f,
                DeleteFaceOptions::default(),
            )
        });
        assert!(matches!(r, Err(SolidError::Upstream { code: 1001, .. })));
        b.set_editing_enabled(true);
        b.remove_from_face_list(f);
        let r = with_env(|env| {
            delete_face(
                &mut k,
                &mut b,
                env,
                &mut marks,
                f,
                DeleteFaceOptions::default(),
            )
        });
        assert!(matches!(r, Err(SolidError::Upstream { code: 1001, .. })));
        assert!(DeleteFaceOptions::default().do_region_nesting);
    }

    #[test]
    fn turn_to_nurbs_of_an_empty_brep_is_a_no_op() {
        let k = Kernel::default();
        let mut b = Brep::new();
        with_env(|env| turn_to_nurbs(&k, &mut b, env)).unwrap();
        assert!(b.faces().is_empty());
    }

    #[test]
    fn region_list_head_moves_on_removal() {
        let mut b = Brep::new();
        let r0 = b.add_region_record(Region::default()).unwrap();
        let r1 = b.add_region_record(Region::default()).unwrap();
        b.post_insert_region(r0).unwrap();
        b.post_insert_region(r1).unwrap();
        assert_eq!(b.region_list_head(), Some(r0));
        b.remove_region_from_list(r0).unwrap();
        assert_eq!(b.region_list_head(), Some(r1));
        b.remove_region_from_list(r1).unwrap();
        assert_eq!(b.region_list_head(), None);
        assert!(p1::regions(&b, None).unwrap().is_empty());
    }
}
