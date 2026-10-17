//! Incomplete B-rep construction work and ported split-edge refusal decisions.
//!
//! Upstream: `source/SMLib/src/SmBrep.cpp`, `source/SMLib/inc/SmBrep.h`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`find_infinite_region`] | `SmBrep::FindInfiniteRegion` |
//! | [`orient_sheet_surfaces`] | `SmBrep::OrientSheetSurfaces` |
//! | [`make_shell_vertex`] | `SmBrep::MakeShellVertex` |
//! | [`make_vertex_loop`] | `SmBrep::MakeVertexLoop` |
//! | [`make_vertex_split_edge`] | `SmBrep::MakeVertexSplitEdge` |
//! | [`make_edge_in_face_with_vu`], [`drop_edge_curve`] | `SmBrep::MakeEdgeInFaceWithVU` |
//!
//! Deviations: typed ids replace pointers; editing state is an explicit argument
//! (upstream defaults to enabled). Split failures retain the nearby vertex in
//! [`SplitEdgeOutput`] alongside their status. Missing B-rep owning-list traversal,
//! shell-vertex topology and face/B-rep notifications stop at their owning units;
//! continuations behind these barriers are not implemented or certified. Interior
//! wire splitting and face insertion defer before allocating orphan records; the
//! native pre-barrier allocations/notifications are not replayed on deferral.
//! Owned geometry arguments are dropped on Rust errors. No input-driven recursion
//! is reachable: `FindInfiniteRegion` stops before its single shell-to-no-shell retry.
//! Native null dereferences return `InvalidInput`; nonfinite split parameters
//! retain the defined native refusal/output behavior. This unit stays `todo` until
//! the omitted bodies identified by `OMITTED` comments are ported. Editing state
//! must move to `Brep::editing_enabled` when its owner (part 1, PR #145) lands;
//! this module index must then retain the other independently ported parts.

use crate::curve::curve::Curve;
use crate::error::{Result, SolidError, SM_ERR};
use crate::geom::axis2_placement::Axis2Placement;
use crate::geom::extent1d::Extent1d;
use crate::kernel::Kernel;
use crate::math::{Point3, Vec3, EFF_ZERO_PARAM};
use crate::nurbs::b_spline_surface::p1::SurfParamType;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::surface::{Surface, SurfaceKind};
use crate::topology::edge;
use crate::topology::vertex;
use crate::topology::vertexuse;
use crate::topology::{
    Brep, EdgeId, FaceId, LoopId, RegionId, ShellId, VertexId, VertexuseId, VertexuseOwner,
};

const OP: &str = "topology::brep::p9";

/// Outputs of `MakeVertexSplitEdge`, including the endpoint on ordinary failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SplitEdgeOutput {
    /// Whether the operation split the edge (`SM_SUCCESS`). False means `SM_ERR`.
    pub split: bool,
    /// Reused edge and new edge, assigned only on successful completion.
    pub edges: Option<[EdgeId; 2]>,
    /// New split vertex, or the nearby endpoint when the split was refused.
    pub vertex: Option<VertexId>,
}

impl SplitEdgeOutput {
    /// The upstream status; dependency errors use the enclosing `Result`.
    #[must_use]
    pub const fn status(self) -> i32 {
        if self.split {
            0
        } else {
            SM_ERR
        }
    }
}

/// `FindInfiniteRegion`, including its optional shell and bounding box arguments.
///
/// The first call, `GetRegions`, requires the B-rep's native owning-list order.
/// An arena's allocation order cannot substitute for it. The ray selection and
/// shell retry continuation must be wired when part 1 supplies that list.
///
/// # Errors
/// Returns the owning unit's `Unported` error, without selecting a guessed region.
pub fn find_infinite_region(
    b: &Brep,
    shell: Option<ShellId>,
    bbox: Option<&crate::geom::extent3d::Extent3d>,
) -> Result<Option<RegionId>> {
    let _ = (b, shell, bbox);
    // OMITTED SmBrep.cpp:8018-8581: region selection, ray tests and shell retry.
    // Wiring GetRegions alone does not complete this function.
    // UNPORTED(topology.brep_p1): SmBrep::GetRegions
    Err(SolidError::unported(
        "SmBrep::GetRegions",
        "topology.brep_p1",
    ))
}

/// `OrientSheetSurfaces`. `same=true` is upstream's default orientation policy.
///
/// # Errors
/// `GetFaces` is the first call and remains owned by part 1; its membership check
/// precedes the part-2 sheet/oriented-sheet predicates and all UV swaps.
pub fn orient_sheet_surfaces(b: &mut Brep, face: Option<FaceId>, same: bool) -> Result<()> {
    let _ = (b, face, same);
    // OMITTED SmBrep.cpp:11632-11703: membership/sheet tests, UV swaps and walk.
    // UNPORTED(topology.brep_p1): SmBrep::GetFaces
    Err(SolidError::unported("SmBrep::GetFaces", "topology.brep_p1"))
}

/// `MakeShellVertex`: `region=None` selects the infinite region.
///
/// # Errors
/// The default region lookup belongs to part 1. With an explicit region, allocation
/// and linking first call part 2's `MakeShellVertexTopology`. No isolated vertex or
/// shell is fabricated while that dependency is unavailable.
pub fn make_shell_vertex(
    b: &mut Brep,
    region: Option<RegionId>,
    point: Point3,
) -> Result<(ShellId, VertexId)> {
    let Some(region) = region else {
        // OMITTED SmBrep.cpp:19077-19088: construction, point and notifications.
        // UNPORTED(topology.brep_p1): SmBrep::GetInfiniteRegion
        return Err(SolidError::unported(
            "SmBrep::GetInfiniteRegion",
            "topology.brep_p1",
        ));
    };
    b.region(region)?;
    let _ = point;
    // OMITTED SmBrep.cpp:19077-19088: construction, point and notifications.
    // UNPORTED(topology.brep_p2): SmBrep::MakeShellVertexTopology
    Err(SolidError::unported(
        "SmBrep::MakeShellVertexTopology",
        "topology.brep_p2",
    ))
}

/// `MakeVertexLoop`: creates a vertex loop with the supplied orientation
/// (upstream's default is `OrientType::Opposite`).
///
/// # Errors
/// The face's `PRE_EDIT` notification is the first operation. It belongs to part 3,
/// and must execute before allocating/linking loop uses or emitting ADD notifications.
pub fn make_vertex_loop(
    b: &mut Brep,
    face: FaceId,
    point: Point3,
    orientation: OrientType,
) -> Result<(LoopId, VertexId)> {
    b.face(face)?;
    let _ = (point, orientation);
    // OMITTED SmBrep.cpp:19115-19198: loop/use/vertex construction, owning-list
    // insertion, tolerances and notifications. PRE_EDIT alone cannot unblock it.
    face_pre_edit()
}

fn face_pre_edit<T>() -> Result<T> {
    // UNPORTED(topology.face_p3): SmFace::Notify
    Err(SolidError::unported("SmFace::Notify", "topology.face_p3"))
}

fn required<T>(v: Option<T>, reason: &'static str) -> Result<T> {
    v.ok_or_else(|| SolidError::invalid_input(OP, reason))
}

/// `MakeVertexSplitEdge`: preserve the scaled parameter boundary, the two-stage
/// length tests (2 and 10 samples), start-before-end precedence, and the nearby
/// endpoint returned together with `SM_ERR`.
///
/// `editing_enabled` is the caller's current B-rep editing state. The constructor's
/// default is true. Ordinary no-split results are successful Rust returns whose
/// [`SplitEdgeOutput::status`] is `SM_ERR`; this preserves outputs that C++ callers
/// consume despite that status. Dependency/invalid-graph errors remain `Err`.
///
/// # Errors
/// Invalid graph; evaluator errors; face notifications or the
/// unported UV-split helper reached by an interior split.
pub fn make_vertex_split_edge(
    k: &Kernel,
    b: &mut Brep,
    edge_id: EdgeId,
    parameter: f64,
    editing_enabled: bool,
) -> Result<SplitEdgeOutput> {
    let out = SplitEdgeOutput::default();
    if !editing_enabled {
        return Ok(out);
    }
    let scaled_zero = EFF_ZERO_PARAM * (1.0 + parameter.abs());
    let interval = b.edge(edge_id)?.interval;
    if !interval.contains_value(parameter, scaled_zero) {
        return Ok(out);
    }
    let mut near_start = parameter < interval.min() + scaled_zero;
    let mut near_end = parameter > interval.max() - scaled_zero;
    if !near_start && !near_end {
        let start_ivl = Extent1d::from_min_max(interval.min(), parameter);
        let end_ivl = Extent1d::from_min_max(parameter, interval.max());
        let curve = required(b.edge(edge_id)?.curve.as_ref(), "split edge has no curve")?;
        let dist_to_start = curve.approximate_length(&start_ivl, 2, false)?.0;
        let dist_to_end = curve.approximate_length(&end_ivl, 2, false)?.0;
        let start = required(
            edge::start_vertex(b, edge_id)?,
            "split edge has no start vertex",
        )?;
        let end = required(
            edge::other_vertex(b, edge_id, Some(start))?,
            "split edge has no end vertex",
        )?;
        let start_tol = b.vertex(start)?.tolerance;
        let end_tol = b.vertex(end)?.tolerance;
        if dist_to_start < start_tol
            && curve.approximate_length(&start_ivl, 10, false)?.0 < start_tol
        {
            near_start = true;
        }
        if dist_to_end < end_tol && curve.approximate_length(&end_ivl, 10, false)?.0 < end_tol {
            near_end = true;
        }
        if near_start {
            vertex::set_exact(b, start, false)?;
        }
        if near_end {
            vertex::set_exact(b, end, false)?;
        }
    }
    if near_start {
        return Ok(SplitEdgeOutput {
            vertex: edge::start_vertex(b, edge_id)?,
            ..out
        });
    }
    if near_end {
        return Ok(SplitEdgeOutput {
            vertex: edge::end_vertex(b, edge_id)?,
            ..out
        });
    }

    // PRE_EDIT of a wire edge reaches the implemented SmTopology handler. For
    // faces, the next operation is their PRE_EDIT; stop before changing geometry.
    let faces = edge::faces(b, edge_id, None)?;
    if !faces.is_empty() {
        // OMITTED SmBrep.cpp:21331-21611: edit notifications, allocation, split
        // geometry/topology, UV splitting, tolerance/attribute updates and outputs.
        return face_pre_edit();
    }
    let _ = k;
    // Interior wire splitting reaches the file-local UV-trim splitter, owned by
    // part 1, before the B-rep ADD/SPLIT notifications in part 2. Its wire no-op
    // branch must be supplied by its owner too, rather than silently bypassed.
    // OMITTED SmBrep.cpp:21331-21611: the complete interior-split body (including
    // native work before this dependency) remains unimplemented, not just this call.
    // UNPORTED(topology.brep_p1): sm_MakeVertexSplitUVTrimCurve
    Err(SolidError::unported(
        "sm_MakeVertexSplitUVTrimCurve",
        "topology.brep_p1",
    ))
}

/// Inputs for `MakeEdgeInFaceWithVU`. Optional geometry is owned, as the native
/// operation consumes its optional trim and drop-failure record.
#[derive(Debug, Clone)]
pub struct EdgeInFaceInput {
    /// Initial target face; a shared VU face may replace it upstream.
    pub face: Option<FaceId>,
    /// Start and end VUs; null is a defined ordinary error.
    pub vertices: [Option<VertexuseId>; 2],
    /// Required 3D curve.
    pub curve: Option<Curve>,
    /// Optional UV trim curve.
    pub uv_trim: Option<Curve>,
    /// Optional state consumed with the UV trim.
    pub drop_fail:
        Option<Box<crate::surface::surface::p2::DropCurveFail<crate::topology::ObjectHandle>>>,
    /// Parameter interval of the 3D curve.
    pub interval: Extent1d,
    /// Direction of the new edge relative to its curve.
    pub orientation: OrientType,
    /// Converted `SmZoneTol3d` used for the curve.
    pub curve_zone_tol_3d: f64,
    /// Caller-owned editing state; normally true.
    pub editing_enabled: bool,
}

/// The edge created by `MakeEdgeInFaceWithVU`, and the optional split loop/face.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeInFaceOutput {
    /// Newly created manifold edge.
    pub edge: EdgeId,
    /// Newly created loop when both vertices belong to the same loop.
    pub loop_: Option<LoopId>,
    /// Newly created face when insertion splits the original face.
    pub face: Option<FaceId>,
}

/// `MakeEdgeInFaceWithVU`'s defined null/editing/VU-compatibility failures.
///
/// # Errors
/// Null face/curve, disabled editing or incompatible VUs, then dependency errors
/// from curve projection/face editing. No topology success is simulated.
pub fn make_edge_in_face_with_vu(
    k: &Kernel,
    b: &mut Brep,
    input: EdgeInFaceInput,
) -> Result<EdgeInFaceOutput> {
    let face = input.face.ok_or_else(|| SolidError::null_pointer(OP))?;
    let curve = input.curve.ok_or_else(|| SolidError::null_pointer(OP))?;
    if !input.editing_enabled {
        return Err(SolidError::sm_err(OP));
    }
    let [Some(start), Some(end)] = input.vertices else {
        return Err(SolidError::sm_err(OP));
    };
    for vu in [start, end] {
        if !matches!(
            b.vertexuse(vu)?.owner,
            VertexuseOwner::Edgeuse(_) | VertexuseOwner::Loopuse(_)
        ) {
            return Err(SolidError::sm_err(OP));
        }
    }
    let start_faceuse = vertexuse::faceuse(b, start)?;
    if start_faceuse != vertexuse::faceuse(b, end)? {
        return Err(SolidError::sm_err(OP));
    }
    let fu = required(start_faceuse, "VU pair has no faceuse")?;
    let actual_face = required(b.faceuse(fu)?.face, "VU faceuse has no face")?;
    // Native recovery uses the VUs' face when the supplied classification is stale.
    let face = if face == actual_face {
        face
    } else {
        actual_face
    };
    let record = b.face(face)?;
    let surface = required(record.surface.as_ref(), "face has no surface")?;
    let domain = surface.natural_uv_domain();
    let mut uv_trim = input.uv_trim;
    let mut drop_fail = input.drop_fail;
    let mut max_dist = 0.0;
    let mut deviation = 0.0;
    if uv_trim.is_none() {
        drop_fail = None;
        let dropped = drop_edge_curve(
            k,
            surface,
            &record.uv_domain,
            &curve,
            &input.interval,
            record.tolerance,
        )?;
        let mut uv_curves = dropped.uv_curves;
        max_dist = dropped.max_drop_to_surf;
        deviation = dropped.max_approx_dev;
        if uv_curves.len() == 1 {
            uv_trim = uv_curves.pop();
        }
    }
    // SE(SM_ERR) above SetCurve only reports excessive drop error; it does not return.
    let start_loop = required(vertexuse::loopuse(b, start)?, "start VU has no loopuse")?;
    let end_loop = required(vertexuse::loopuse(b, end)?, "end VU has no loopuse")?;
    required(b.loopuse(start_loop)?.loop_, "start loopuse has no loop")?;
    required(b.loopuse(end_loop)?.loop_, "end loopuse has no loop")?;
    if let Some(trim) = uv_trim {
        let mut copy = trim.copy(k)?;
        // The pointer is declared SmBSplineCurve*, but this is a virtual call.
        let seam = trim.is_domain_boundary(&domain, crate::math::EFF_ZERO)?;
        let translation = match seam {
            SurfParamType::UMin => Vec3::new(domain.x_length(), 0.0, 0.0),
            SurfParamType::UMax => Vec3::new(-domain.x_length(), 0.0, 0.0),
            SurfParamType::VMin => Vec3::new(0.0, domain.y_length(), 0.0),
            SurfParamType::VMax => Vec3::new(0.0, -domain.y_length(), 0.0),
            _ => Vec3::new(0.0, 0.0, 0.0),
        };
        if seam != SurfParamType::Neither {
            let mut transform = Axis2Placement::default();
            transform.translate(translation);
            // Native ignores this ordinary status, but Unported must propagate.
            if let Err(e @ SolidError::Unported { .. }) = copy.transform(k, &transform, None) {
                return Err(e);
            }
        }
    }
    let _ = (
        max_dist,
        deviation,
        drop_fail,
        input.orientation,
        input.curve_zone_tol_3d,
    );
    // OMITTED SmBrep.cpp:12404-12412,12513-12517,12582-12627,12633-12634,12676-13484: native allocations,
    // edge/use/trim ownership and assignment, insertion, loop/face splitting,
    // tolerances, notifications and outputs. The seam-copy calculation above is
    // ported, but its copy is not attached to topology. This is not a complete
    // function whose body can be enabled by replacing face_pre_edit alone.
    face_pre_edit()
}

/// Geometry-only projection sequence used by `MakeEdgeInFaceWithVU`.
///
/// Tries face tolerance, then 10 and 300 times that tolerance, followed by
/// `DropAndTrimCurve` with 100 times the tolerance and both flags true. This
/// prefix is exposed independently so it can be compared before topology editing
/// dependencies become available. It does not create or insert an edge.
///
/// # Errors
/// Propagates projection errors and the actual missing callee's deferral.
pub fn drop_edge_curve(
    k: &Kernel,
    surface: &Surface,
    domain: &crate::geom::extent2d::Extent2d,
    curve: &Curve,
    interval: &Extent1d,
    face_tolerance: f64,
) -> Result<EdgeCurveDrop> {
    for factor in [1.0, 10.0, 300.0] {
        let (uv_curves, max_drop_to_surf, max_approx_dev) =
            drop_curve(k, surface, domain, curve, interval, face_tolerance * factor)?;
        if !uv_curves.is_empty() {
            return Ok(EdgeCurveDrop {
                uv_curves,
                max_drop_to_surf,
                max_approx_dev,
            });
        }
    }
    let dropped = surface.drop_and_trim_curve::<crate::topology::ObjectHandle>(
        k,
        domain,
        curve,
        interval,
        face_tolerance * 100.0,
        true,
        true,
        None,
    )?;
    Ok(EdgeCurveDrop {
        uv_curves: dropped.uv_curves.into_iter().map(Curve::from).collect(),
        max_drop_to_surf: dropped.max_drop_to_surf,
        max_approx_dev: dropped.max_approx_dev,
    })
}

/// Projection prefix outputs, without any topology mutation.
#[derive(Debug)]
pub struct EdgeCurveDrop {
    /// Projected UV curves; only a single result is adopted by the insertion body.
    pub uv_curves: Vec<Curve>,
    /// Largest distance of a dropped point from the curve.
    pub max_drop_to_surf: f64,
    /// Largest approximation deviation from the surface normal line.
    pub max_approx_dev: f64,
}

fn drop_curve(
    k: &Kernel,
    surface: &Surface,
    domain: &crate::geom::extent2d::Extent2d,
    curve: &Curve,
    interval: &Extent1d,
    tolerance: f64,
) -> Result<(Vec<Curve>, f64, f64)> {
    if let SurfaceKind::Plane(plane) = surface.kind() {
        // The real SmPlane override; its generic fallback retains the p3 deferral.
        let dropped = plane.drop_curve(k, domain, curve, interval, tolerance, true, true)?;
        return Ok((
            dropped.uv_curves,
            dropped.max_drop_to_surf,
            dropped.max_approx_dev,
        ));
    }
    // OMITTED caller continuation: SmBrep.cpp:12444-13484; the insertion body
    // after projection is incomplete even when this dependency is available.
    // UNPORTED(intersect.surface_drop_lib_p3): SmSurface::DropCurve
    Err(SolidError::unported(
        "SmSurface::DropCurve",
        "intersect.surface_drop_lib_p3",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extreme_parameters_do_not_panic_and_editing_refusal_clears_outputs() {
        let k = Kernel::default();
        let mut b = Brep::new();
        let v0 = b.add_vertex(Point3::new(0.0, 0.0, 0.0)).unwrap();
        let v1 = b.add_vertex(Point3::new(1.0, 0.0, 0.0)).unwrap();
        let e = b.add_edge(v0, v1).unwrap();
        b.edge_mut(e).unwrap().interval = Extent1d::from_min_max(0.0, 1.0);
        // Endpoint outputs walk the primary/mate edge uses, as native does.
        for (vertex, orientation) in [(v0, OrientType::Same), (v1, OrientType::Opposite)] {
            let vu = b
                .add_vertexuse_record(crate::topology::Vertexuse {
                    vertex: Some(vertex),
                    ..crate::topology::Vertexuse::default()
                })
                .unwrap();
            let eu = b
                .add_edgeuse_record(crate::topology::Edgeuse {
                    edge: Some(e),
                    owner: crate::topology::EdgeuseOwner::Shell(None),
                    vertexuse: Some(vu),
                    orientation,
                    ..crate::topology::Edgeuse::default()
                })
                .unwrap();
            b.vertexuse_mut(vu).unwrap().owner = VertexuseOwner::Edgeuse(Some(eu));
            b.vertex_mut(vertex).unwrap().vertexuses.push(vu);
            b.edge_mut(e).unwrap().edgeuses.push(eu);
        }
        for (p, vertex) in [
            (f64::NAN, None),
            (f64::INFINITY, Some(v1)),
            (f64::NEG_INFINITY, Some(v0)),
        ] {
            let result = make_vertex_split_edge(&k, &mut b, e, p, true).unwrap();
            assert_eq!(
                (result.status(), result.edges, result.vertex),
                (SM_ERR, None, vertex)
            );
            let disabled = make_vertex_split_edge(&k, &mut b, e, p, false).unwrap();
            assert_eq!(disabled.status(), SM_ERR);
            assert_eq!((disabled.edges, disabled.vertex), (None, None));
        }
        for p in [f64::MAX, -f64::MAX] {
            let outside = make_vertex_split_edge(&k, &mut b, e, p, true).unwrap();
            assert_eq!(
                (outside.status(), outside.edges, outside.vertex),
                (SM_ERR, None, None)
            );
        }
    }
}
