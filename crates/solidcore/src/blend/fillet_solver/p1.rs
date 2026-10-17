//! The fillet solver base: solver data, offset-surface bookkeeping, surface extension
//! domains, the solver's nonlinear function objects (sphere/plane solve, rail-UV-curve
//! and rail-rail intersections, clipped radius), and the fillet surface generator types.
//!
//! Upstream: `source/SMLib/src/SmFilletSolver.cpp`, `source/SMLib/inc/SmFilletSolver.h`.
//!
//! | Rust                                                   | Upstream                                              |
//! |--------------------------------------------------------|-------------------------------------------------------|
//! | [`FilletSolverOffsetsData`]                            | `SmFilletSolverOffsetsData` (constructor)             |
//! | [`FilletSolver`], [`FilletSolver::new`]                | `SmFilletSolver` (members, constructor; `Drop` is the destructor) |
//! | [`FilletSolver::finish_construction`]                  | `SmFilletSolver::FinishConstruction`                  |
//! | getters / setters of [`FilletSolver`]                  | `GetStatus`, `GetReverseTrim`, `GetFilletExecutive`, `GetFilletSurfaceGenerator`, `GetThisApproxTol3d`, `GetThisAngTolRad`, `GetConversionTol`, `GetTangencyTolerance`, `GetEdgeuse`, `GetFilletGeomForSurfaces`, `GetFirstFilletGeom`, `GetLastFilletGeom`, `GetOrientation`, `GetExtensionFactor`, `SetStatus`, `SetReverseTrim`, `SetExtensionFactor`, `SetFilletExecutive`, `SetFilletSurfaceGenerator`, `SetFilletGeomForSurfaces` (`GetCreationContext`: the `Kernel` argument) |
//! | [`FilletSolver::surface`], [`FilletSolver::set_surface`] | `GetSurface`, `SetSurface`                          |
//! | [`FilletSolver::fillet_geoms`]                         | `GetFilletGeoms`                                      |
//! | [`FilletSolver::self_intersection_handler`]            | `GetSelfIntersectionHandler`                          |
//! | [`FilletSolver::vertex`]                               | `GetVertex`                                           |
//! | [`FilletSolver::find_index_of_rail_on_face`]           | `FindIndexOfRailOnFace`                               |
//! | [`FilletSolver::find_index_of_rail_x_side_edgeuse`]    | `FindIndexOfRailXSideEdgeuse`                         |
//! | [`FilletSolver::set_surfaces_to_zero_offset`]          | `SetSurfacesToZeroOffset`                             |
//! | [`FilletSolver::reload_surface_offsets`]               | `ReloadSurfaceOffsets`                                |
//! | [`FilletSolver::set_extension_flags`]                  | `SetExtensionFlags`                                   |
//! | [`FilletSolver::set_offsets_data`], [`FilletSolver::offsets_data`] | `SetOffsetsData`, `GetOffsetsData`        |
//! | [`FilletSolver::setup_offset_extension`]               | `SetupOffsetExtension`                                |
//! | [`FilletSolver::compute_surface_values`]               | `ComputeSurfaceValues`                                |
//! | [`compute_uv_vectors`]                                 | `SmFilletSolver::ComputeUVVectors`                    |
//! | [`FilletSolver::split_fillet_geom_at_side_edge`]       | `SplitFilletGeomAtSideEdge`                           |
//! | [`FilletSolver::split_fillet_geom`]                    | `SplitFilletGeom`                                     |
//! | [`FilletSolver::trim_rails`]                           | `TrimRails`                                           |
//! | [`FilletSolver::TYPE`], [`FilletSolver::type_string`], [`FilletSolver::is_kind_of`] | `GetType`, `GetTypeString` (`SM_COMMON`), `IsKindOf` |
//! | [`FilletSolverOps`] (trait, base methods)              | the virtuals `GetJacobianSize`, `SetupOffsetValues`, `CanMakeAnalytic`, `OffsetRadiiCanChange`, `LoadJacobian`, `LoadInitialValues` |
//! | [`FilletSurfaceGenerator`]                             | `SmFilletSurfaceGenerator`, `SmLinearCrossSectionFSG`, `SmCircularCrossSectionFSG`, `SmBlendCurveCrossSectionFSG` (constructors, `GetFilletSurfaceGeneratorType`, `GetBlendScale`, `GetContinuity`; `Drop` is each destructor) |
//! | [`FilletSurfaceGenerator::create_surface`]             | `SmFilletSurfaceGenerator::CreateSurface` (base)      |
//! | [`FilletSphereSolveEnfo`]                              | `SmFilletSphereSolveENFO` (constructor, `SetPlaneAdjusts`, `Evaluate`) |
//! | [`RailUvCurveIntersectEnfo`]                           | `SmRailUVCurveIntersectENFO` (constructor, `Evaluate`) |
//! | [`RailRailIntersectEnfo`]                              | `SmRailRailIntersectENFO` (constructor, `Evaluate`)   |
//! | [`FindClippedRadiusEfo`]                               | `SmFindClippedRadiusEFO` (constructor, `Evaluate`)    |
//! | [`tol_ang_tol_rad`]                                  | `SmTol::GetAngTolRad(const SmFilletSolver*)` |
//! | [`check_edge_curve_domains`]                           | `sm_CheckEdgeCurveDomains`                            |
//! | [`test_analytic_surface`]                              | `sm_TestAnalyticSurface`                              |
//! | [`test_manifold_tangent_rollover`]                     | `sm_TestManifoldTangentRollover`                      |
//! | [`test_side_edge_convexity`]                           | `sm_TestSideEdgeConvexity`                            |
//! | [`find_next_good_interval`]                            | `sm_FindNextGoodInterval`                             |
//! | [`trim_rail`]                                          | `sm_TrimRail` (a rail edge of a [`Brep`])             |
//! | [`split_rail`], [`compute_support_edge`]               | `sm_SplitRail`, `sm_ComputeSupportEdge`               |
//! | (`Drop`)                                               | `~SmSelfIntersectionHandler`, `~SmMakeCurveBlendSIH`, `SmMakeSurfaceBlendSIH` constructor and destructor (no data) |
//!
//! Deviations:
//! - **Object model.** Virtual methods of the solver are the [`FilletSolverOps`] trait;
//!   [`FilletSolver`] implements the base versions. Offset surfaces are the co-ported
//!   [`OffsetSurface`] of `blend::fillet_custom_solver` held in an
//!   [`OffsetSurfaceSlot`] whose `key` stands for the `SmOffsetSurface*` upstream
//!   compares (`SetOffsetsData`); [`FilletSolverOffsetsData`] holds copies of the slots,
//!   so restoring a slot with a different key puts the saved surface back (upstream
//!   deletes the current surface and stores the saved pointer).
//! - **Fillet geometries, executive, corners.** `SmFilletGeom`, `SmFilletExecutive`,
//!   the pseudo-B-rep (`SmFilletEdge`/`SmFilletVertex`) and the fillet corners belong to
//!   unported units: the solver holds opaque [`FilletGeomRef`] / [`FilletExecutiveRef`]
//!   handles. The constructor's `new SmFilletGeom(this)` runs while the solver has no
//!   offset surfaces, so that geometry carries only defaults and is a bare
//!   [`FilletGeomRef`]. Every read of their data returns [`SolidError::Unported`]
//!   naming the owning unit; `SplitFilletGeomAtSideEdge`, `TrimRails`,
//!   `sm_SplitRail` and `sm_ComputeSupportEdge` stop at their first such call (their
//!   remaining bodies need those types and are completed with them).
//! - **Arguments.** Out-params become return values; `SmContext` arguments become the
//!   [`Kernel`]. `SmFindClippedRadiusEFO` takes the target distance its constructor
//!   reads from the `SmConstantDistanceFS` (`GetDistance`, unit
//!   `blend.fillet_standard_solver_p1`). `SplitFilletGeom` takes the raw
//!   `SmBoundaryTrimmingType` value.
//! - **Undefined inputs.** A `NULL` dereference upstream (a missing edge use, offset
//!   surface, executive, loop use) and an index past an array (`crX`, `rF`, the
//!   Jacobian, the fillet geometries, the interval arrays) are
//!   [`SolidError::InvalidInput`].
//! - **Iteration.** No loop here depends on more than the caller's array sizes; there
//!   is no recursion.

use crate::blend::fillet_custom_solver::{InitialValues, JacobianLoad, OffsetSurface, SurfaceId};
use crate::curve::curve::Curve;
use crate::error::{ignore_status, ignored, Result, SolidError};
use crate::geom::extent1d::{Extent1d, ExtentNd};
use crate::geom::extent2d::Extent2d;
use crate::geom::matrix::Matrix;
use crate::geom::object::OBJECT_TYPE;
use crate::geom::tol::ZONE_TOL_3D;
use crate::geom::vector2d::{Point2, Vec2};
use crate::intersect::curve_class::p1::CurveClassification;
use crate::intersect::point_class::{ClassObject, PointClassType};
use crate::intersect::surface_intersector::p1::TsectPnt;
use crate::kernel::Kernel;
use crate::math::{Point3, Vec3, EFF_ZERO, EFF_ZERO_SQ, PI};
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::nurbs::b_spline_curve_n_lib::TrimMode;
use crate::nurbs::b_spline_surface::SurfParamType;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::surface::p2::drop_vectors;
use crate::surface::surface::Surface;
use crate::topology::edgeuse::p1 as eu;
use crate::topology::{edge, vertexuse, Brep, EdgeId, EdgeuseId, FaceId, VertexId};

const OP: &str = "blend::fillet_solver::p1";

/// `SmFilletSolver_TYPE` (`TOPO_BASE_TYPE + 37`).
const FILLET_SOLVER_TYPE: i64 = 16037;

/// `static int cbiLinear` of `SetupOffsetExtension`: never changed, so the linear
/// extension branch is never taken.
const CBI_LINEAR: i32 = 0;

fn invalid(reason: &'static str) -> SolidError {
    SolidError::invalid_input(OP, reason)
}

/// Upstream `if (call != SM_SUCCESS) return SM_ERR;`: any failure becomes `SM_ERR`;
/// [`SolidError::Unported`] and [`SolidError::InvalidInput`] (a path that is undefined
/// upstream, not a status) pass through.
fn as_sm_err(e: SolidError, op: &'static str) -> SolidError {
    match e {
        SolidError::Unported { .. } | SolidError::InvalidInput { .. } => e,
        _ => SolidError::sm_err(op),
    }
}

// ---- small types ----------------------------------------------------------------------

/// `SmFilStatus`: the processing status of a solver (and of fillet geometries,
/// vertices and edges).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FilStatus {
    /// `SM_FIL_UNPROCESSED`.
    #[default]
    Unprocessed = 0,
    /// `SM_FIL_PROCESSED`.
    Processed = 1,
    /// `SM_FIL_FAILURE`.
    Failure = 2,
    /// `SM_FIL_SURF_INT_FAILURE`.
    SurfIntFailure = 3,
    /// `SM_FE_END_PNT_DROP_FAILURE`.
    EndPntDropFailure = 4,
    /// `SM_FE_TO_BE_SQUEEZED`.
    ToBeSqueezed = 5,
    /// `SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE`.
    CubicRailRailInterpolate = 6,
    /// `SM_FV_NO_INT_RAIL_EU`.
    NoIntRailEu = 7,
    /// `SM_FV_SOLVER_NOT_CONVERGE`.
    SolverNotConverge = 8,
    /// `SM_FV_TO_BE_DELETED`.
    ToBeDeleted = 9,
    /// `SM_FS_SURF_TRACING_FAILURE`.
    SurfTracingFailure = 10,
    /// `SM_FS_SELF_INT_HANDLING_FAILURE`.
    SelfIntHandlingFailure = 11,
    /// `SM_FS_SURF_SKINNING_FAILURE`.
    SurfSkinningFailure = 12,
}

/// `SmFilletSurfaceGeneratorType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilletSurfaceGeneratorType {
    /// `SM_FSG_UNKNOWN`.
    Unknown = 0,
    /// `SM_FSG_LINEAR`.
    Linear = 1,
    /// `SM_FSG_CIRCULAR`.
    Circular = 2,
    /// `SM_FSG_BLEND_CURVE`.
    BlendCurve = 3,
}

/// A fillet surface generator (`SmFilletSurfaceGenerator` and its derived classes):
/// the cross-section shape of the fillet surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilletSurfaceGenerator {
    /// `SmFilletSurfaceGenerator` itself.
    Base,
    /// `SmLinearCrossSectionFSG`: a ruled surface.
    Linear,
    /// `SmCircularCrossSectionFSG`.
    Circular {
        /// `m_bApproximateArcs`.
        approximate_arcs: bool,
        /// `m_dTolPercentRadius`.
        tol_percent_radius: f64,
        /// `m_bMirror`.
        mirror: bool,
        /// `m_bComplement`.
        complement: bool,
    },
    /// `SmEiffelTowerCrossSectionFSG` (unit `blend.fillet_custom_solver`).
    EiffelTower,
    /// `SmBlendCurveCrossSectionFSG`.
    BlendCurve {
        /// `m_lContinuity`: 1, 2 or 3 for G1, G2 or G3.
        continuity: u64,
        /// `m_dBlendScale`.
        blend_scale: f64,
    },
}

impl FilletSurfaceGenerator {
    /// `SmCircularCrossSectionFSG()` with its default arguments (`TRUE, 0.05, FALSE,
    /// FALSE`).
    #[must_use]
    pub const fn circular_default() -> Self {
        Self::Circular {
            approximate_arcs: true,
            tol_percent_radius: 0.05,
            mirror: false,
            complement: false,
        }
    }

    /// `SmBlendCurveCrossSectionFSG()` with its default arguments (`1.5, 1`).
    #[must_use]
    pub const fn blend_curve_default() -> Self {
        Self::BlendCurve {
            continuity: 1,
            blend_scale: 1.5,
        }
    }

    /// `GetFilletSurfaceGeneratorType` (virtual).
    #[must_use]
    pub const fn fillet_surface_generator_type(&self) -> FilletSurfaceGeneratorType {
        match self {
            Self::Base | Self::EiffelTower => FilletSurfaceGeneratorType::Unknown,
            Self::Linear => FilletSurfaceGeneratorType::Linear,
            Self::Circular { .. } => FilletSurfaceGeneratorType::Circular,
            Self::BlendCurve { .. } => FilletSurfaceGeneratorType::BlendCurve,
        }
    }

    /// `SmBlendCurveCrossSectionFSG::GetBlendScale` (`None` for the other kinds).
    #[must_use]
    pub const fn blend_scale(&self) -> Option<f64> {
        match self {
            Self::BlendCurve { blend_scale, .. } => Some(*blend_scale),
            _ => None,
        }
    }

    /// `SmBlendCurveCrossSectionFSG::GetContinuity` (`None` for the other kinds).
    #[must_use]
    pub const fn continuity(&self) -> Option<u64> {
        match self {
            Self::BlendCurve { continuity, .. } => Some(*continuity),
            _ => None,
        }
    }

    /// `CreateSurface` (virtual) on the rails and fillet points: `SM_ERR` for the base,
    /// the Eiffel tower surface for [`FilletSurfaceGenerator::EiffelTower`] (it reads
    /// only the solver's `GetThisApproxTol3d()`, passed as `approx_tol_3d`).
    ///
    /// # Errors
    ///
    /// `SM_ERR` for [`FilletSurfaceGenerator::Base`]; those of
    /// `SmEiffelTowerCrossSectionFSG::CreateSurface`; [`SolidError::Unported`] for the
    /// linear, circular and blend-curve generators.
    pub fn create_surface(
        &self,
        k: &Kernel,
        approx_tol_3d: f64,
        rail1: &mut BSplineCurve,
        rail2: &mut BSplineCurve,
        points: &[TsectPnt],
    ) -> Result<crate::nurbs::b_spline_surface::BSplineSurface> {
        match self {
            Self::Base => Err(SolidError::sm_err(OP)),
            Self::EiffelTower => {
                crate::blend::fillet_custom_solver::EiffelTowerCrossSectionFsg::new()
                    .create_surface(k, approx_tol_3d, rail1, rail2, points)
            }
            // UNPORTED(blend.fillet_standard_solver_p1): SmLinearCrossSectionFSG::CreateSurface
            Self::Linear => Err(SolidError::unported(
                "SmLinearCrossSectionFSG::CreateSurface",
                "blend.fillet_standard_solver_p1",
            )),
            // UNPORTED(blend.fillet_standard_solver_p2): SmCircularCrossSectionFSG::CreateSurface
            Self::Circular { .. } => Err(SolidError::unported(
                "SmCircularCrossSectionFSG::CreateSurface",
                "blend.fillet_standard_solver_p2",
            )),
            // UNPORTED(blend.fillet_standard_solver_p2): SmBlendCurveCrossSectionFSG::CreateSurface
            Self::BlendCurve { .. } => Err(SolidError::unported(
                "SmBlendCurveCrossSectionFSG::CreateSurface",
                "blend.fillet_standard_solver_p2",
            )),
        }
    }
}

/// An `SmFilletGeom*` held by the solver. The geometry's data belongs to the
/// `blend.fillet_geom_*` units and is not modelled; `index` tells the geometries apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FilletGeomRef {
    /// Creation order within the solver.
    pub index: usize,
}

/// An `SmFilletExecutive*` (unit `blend.fillet_executive_p1`, not modelled).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FilletExecutiveRef;

/// An `SmFilletEdge*` / `SmFilletVertex*` of the fillet pseudo-B-rep (unit
/// `blend.fillet_geom_p1`, not modelled).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FilletTopologyRef {
    /// Tells the entities apart.
    pub index: usize,
}

/// An offset surface owned by a solver, with the identity of the `SmOffsetSurface*`
/// upstream compares.
#[derive(Debug, Clone)]
pub struct OffsetSurfaceSlot {
    /// Equal keys are the same `SmOffsetSurface` object.
    pub key: u64,
    /// The surface.
    pub surface: OffsetSurface,
}

/// `SmFilletSolverOffsetsData`: saved offset surfaces and orientations (roll-over).
#[derive(Debug, Clone)]
pub struct FilletSolverOffsetsData {
    /// `m_pSurfaces`.
    pub surfaces: [Option<OffsetSurfaceSlot>; 2],
    /// `m_dOrientations`.
    pub orientations: [f64; 2],
}

impl Default for FilletSolverOffsetsData {
    /// No surfaces, orientations `1.0`.
    fn default() -> Self {
        Self {
            surfaces: [None, None],
            orientations: [1.0, 1.0],
        }
    }
}

// ---- the solver ------------------------------------------------------------------------

/// The fillet solver base (`SmFilletSolver`).
#[derive(Debug, Clone)]
pub struct FilletSolver {
    /// `m_pEdgeuses`: the two radial edge uses of the sector being filleted.
    pub edgeuses: [Option<EdgeuseId>; 2],
    /// `m_pExecutive`.
    pub executive: Option<FilletExecutiveRef>,
    /// `m_dThisApproxTol3d`.
    pub approx_tol_3d: f64,
    /// `m_dThisAngTolRad`.
    pub ang_tol_rad: f64,
    /// `m_dTangencyTolerance`.
    pub tangency_tolerance: f64,
    /// `m_dEdgeBlendFactor` (`1.0`).
    pub edge_blend_factor: f64,
    /// `m_pSurfaces`.
    pub surfaces: [Option<OffsetSurfaceSlot>; 2],
    /// `m_dOrientations` (not initialised upstream; `1.0` here, see [`FilletSolver::new`]).
    pub orientations: [f64; 2],
    /// `m_pFGForSurfaces`.
    pub fg_for_surfaces: Option<FilletGeomRef>,
    /// `m_dSavedOffsets` (not initialised upstream; `0.0` here).
    pub saved_offsets: [f64; 2],
    /// `m_dSurfaceExtensionFactor` (`0.5`).
    pub surface_extension_factor: f64,
    /// `m_dConversionTol` (`SM_EFF_ZERO / 10`).
    pub conversion_tol: f64,
    /// `m_bReverseTrim`.
    pub reverse_trim: bool,
    /// `m_bExtendBefore` (not initialised upstream; `false` here).
    pub extend_before: bool,
    /// `m_bExtendAfter` (not initialised upstream; `false` here).
    pub extend_after: bool,
    /// `m_pFSG`.
    pub fsg: Option<FilletSurfaceGenerator>,
    /// `m_vFilletGeoms`.
    pub fillet_geoms: Vec<FilletGeomRef>,
    /// `m_eStatus`.
    pub status: FilStatus,
    /// `m_vG1Knots`.
    pub g1_knots: Vec<f64>,
    /// Offset surfaces [`FilletSolver::set_surface`] displaced: upstream does not
    /// delete them, so a saved [`FilletSolverOffsetsData`] still points at them (with
    /// any change made while they were the solver's).
    pub detached: Vec<OffsetSurfaceSlot>,
}

impl FilletSolver {
    /// `SmFilletSolver_TYPE` (`GetType`).
    pub const TYPE: i64 = FILLET_SOLVER_TYPE;

    /// `SmFilletSolver(crContext, dThisApproxTol3d, dThisAngTolRad,
    /// dTangencyTolerance)`: no edge uses, surfaces or executive, one fillet geometry.
    /// Members upstream leaves uninitialised (`m_pEdgeuses`, `m_pExecutive`,
    /// `m_dOrientations`, `m_dSavedOffsets`, the extension flags) start as `None`,
    /// `1.0`, `0.0` and `false`; every derived constructor sets them before use.
    #[must_use]
    pub fn new(approx_tol_3d: f64, ang_tol_rad: f64, tangency_tolerance: f64) -> Self {
        Self {
            edgeuses: [None, None],
            executive: None,
            approx_tol_3d,
            ang_tol_rad,
            tangency_tolerance,
            edge_blend_factor: 1.0,
            surfaces: [None, None],
            orientations: [1.0, 1.0],
            fg_for_surfaces: None,
            saved_offsets: [0.0, 0.0],
            surface_extension_factor: 0.5,
            conversion_tol: EFF_ZERO / 10.0,
            reverse_trim: false,
            extend_before: false,
            extend_after: false,
            fsg: None,
            fillet_geoms: vec![FilletGeomRef { index: 0 }],
            status: FilStatus::Unprocessed,
            g1_knots: Vec::new(),
            detached: Vec::new(),
        }
    }

    /// `"SmFilletSolver_TYPE"` (`GetTypeString`).
    #[must_use]
    pub const fn type_string(&self) -> &'static str {
        "SmFilletSolver_TYPE"
    }

    /// True for `SmFilletSolver_TYPE` and `SmObject_TYPE` (`IsKindOf`).
    #[must_use]
    pub const fn is_kind_of(t: i64) -> bool {
        t == FILLET_SOLVER_TYPE || t == OBJECT_TYPE
    }

    /// `FinishConstruction`: installs copies of the offset surfaces into every fillet
    /// geometry.
    ///
    /// # Errors
    ///
    /// [`SolidError::Unported`] as soon as there is a surface to copy
    /// (`SmOffsetSurface::Copy`, then `SmFilletGeom::SetOffsetSurface`), or from
    /// [`FilletSolver::surface`].
    pub fn finish_construction(&self) -> Result<()> {
        for _fg in &self.fillet_geoms {
            for which in 0..2 {
                if self.surface(which)?.is_none() {
                    continue;
                }
                // UNPORTED(surface.offset_surface): SmOffsetSurface::Copy
                return Err(SolidError::unported(
                    "SmOffsetSurface::Copy",
                    "surface.offset_surface",
                ));
            }
        }
        Ok(())
    }

    /// `GetStatus`.
    #[must_use]
    pub const fn status(&self) -> FilStatus {
        self.status
    }

    /// `SetStatus`.
    pub fn set_status(&mut self, status: FilStatus) {
        self.status = status;
    }

    /// `GetReverseTrim`.
    #[must_use]
    pub const fn reverse_trim(&self) -> bool {
        self.reverse_trim
    }

    /// `SetReverseTrim`.
    pub fn set_reverse_trim(&mut self, reverse_trim: bool) {
        self.reverse_trim = reverse_trim;
    }

    /// `GetFilletExecutive`.
    #[must_use]
    pub const fn fillet_executive(&self) -> Option<FilletExecutiveRef> {
        self.executive
    }

    /// `SetFilletExecutive`.
    pub fn set_fillet_executive(&mut self, executive: Option<FilletExecutiveRef>) {
        self.executive = executive;
    }

    /// `GetFilletSurfaceGenerator`.
    #[must_use]
    pub const fn fillet_surface_generator(&self) -> Option<&FilletSurfaceGenerator> {
        self.fsg.as_ref()
    }

    /// `SetFilletSurfaceGenerator`.
    pub fn set_fillet_surface_generator(&mut self, fsg: Option<FilletSurfaceGenerator>) {
        self.fsg = fsg;
    }

    /// `GetThisApproxTol3d`.
    #[must_use]
    pub const fn this_approx_tol_3d(&self) -> f64 {
        self.approx_tol_3d
    }

    /// `GetThisAngTolRad`.
    #[must_use]
    pub const fn this_ang_tol_rad(&self) -> f64 {
        self.ang_tol_rad
    }

    /// `GetConversionTol`.
    #[must_use]
    pub const fn conversion_tol(&self) -> f64 {
        self.conversion_tol
    }

    /// `GetTangencyTolerance`.
    #[must_use]
    pub const fn tangency_tolerance(&self) -> f64 {
        self.tangency_tolerance
    }

    /// `GetEdgeuse(lIndex)`.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for an index above 1.
    pub fn edgeuse(&self, index: usize) -> Result<Option<EdgeuseId>> {
        self.edgeuses
            .get(index)
            .copied()
            .ok_or_else(|| invalid("edge use index out of range"))
    }

    /// `GetFilletGeomForSurfaces`.
    #[must_use]
    pub const fn fillet_geom_for_surfaces(&self) -> Option<FilletGeomRef> {
        self.fg_for_surfaces
    }

    /// `SetFilletGeomForSurfaces`.
    pub fn set_fillet_geom_for_surfaces(&mut self, fg: Option<FilletGeomRef>) {
        self.fg_for_surfaces = fg;
    }

    /// `GetFirstFilletGeom`.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] without fillet geometries.
    pub fn first_fillet_geom(&self) -> Result<FilletGeomRef> {
        self.fillet_geoms
            .first()
            .copied()
            .ok_or_else(|| invalid("no fillet geometry"))
    }

    /// `GetLastFilletGeom`.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] without fillet geometries.
    pub fn last_fillet_geom(&self) -> Result<FilletGeomRef> {
        self.fillet_geoms
            .last()
            .copied()
            .ok_or_else(|| invalid("no fillet geometry"))
    }

    /// `GetFilletGeoms`.
    #[must_use]
    pub fn fillet_geoms(&self) -> Vec<FilletGeomRef> {
        self.fillet_geoms.clone()
    }

    /// `GetOrientation(lRailIndex)`.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for a rail index above 1.
    pub fn orientation(&self, rail: usize) -> Result<f64> {
        self.orientations
            .get(rail)
            .copied()
            .ok_or_else(|| invalid("rail index out of range"))
    }

    /// `GetExtensionFactor`.
    #[must_use]
    pub const fn extension_factor(&self) -> f64 {
        self.surface_extension_factor
    }

    /// `SetExtensionFactor` (virtual; no override).
    pub fn set_extension_factor(&mut self, factor: f64) {
        self.surface_extension_factor = factor;
    }

    /// `GetSelfIntersectionHandler`: the executive's handler.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] without an executive (upstream dereferences
    /// `NULL`); [`SolidError::Unported`] otherwise (the executive's data, unit
    /// `blend.fillet_executive_p1`).
    pub fn self_intersection_handler(&self) -> Result<()> {
        if self.executive.is_none() {
            return Err(invalid("no fillet executive"));
        }
        // UNPORTED(blend.fillet_executive_p1): SmFilletExecutive::m_pSelfIntersectionHandler (member read)
        Err(SolidError::unported(
            "SmFilletExecutive::m_pSelfIntersectionHandler",
            "blend.fillet_executive_p1",
        ))
    }

    /// `GetSurface(lRailIndex)`: the fillet geometry's surface when one is set for the
    /// surfaces, else the solver's own.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for a rail index above 1;
    /// [`SolidError::Unported`] with a geometry for the surfaces
    /// (`SmFilletGeom::GetOffsetSurface`).
    pub fn surface(&self, rail: usize) -> Result<Option<&OffsetSurfaceSlot>> {
        if self.fg_for_surfaces.is_some() {
            // UNPORTED(blend.fillet_geom_p1): SmFilletGeom::GetOffsetSurface
            return Err(SolidError::unported(
                "SmFilletGeom::GetOffsetSurface",
                "blend.fillet_geom_p1",
            ));
        }
        self.surfaces
            .get(rail)
            .map(Option::as_ref)
            .ok_or_else(|| invalid("rail index out of range"))
    }

    pub(crate) fn surface_mut(&mut self, rail: usize) -> Result<Option<&mut OffsetSurfaceSlot>> {
        if self.fg_for_surfaces.is_some() {
            // UNPORTED(blend.fillet_geom_p1): SmFilletGeom::GetOffsetSurface
            return Err(SolidError::unported(
                "SmFilletGeom::GetOffsetSurface",
                "blend.fillet_geom_p1",
            ));
        }
        self.surfaces
            .get_mut(rail)
            .map(Option::as_mut)
            .ok_or_else(|| invalid("rail index out of range"))
    }

    /// `SetSurface(lRailIndex, pNewOffsetSurface)`.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for a rail index above 1;
    /// [`SolidError::Unported`] with a geometry for the surfaces
    /// (`SmFilletGeom::SetOffsetSurface`).
    pub fn set_surface(&mut self, rail: usize, surface: Option<OffsetSurfaceSlot>) -> Result<()> {
        if self.fg_for_surfaces.is_some() {
            // UNPORTED(blend.fillet_geom_p1): SmFilletGeom::SetOffsetSurface
            return Err(SolidError::unported(
                "SmFilletGeom::SetOffsetSurface",
                "blend.fillet_geom_p1",
            ));
        }
        let slot = self
            .surfaces
            .get_mut(rail)
            .ok_or_else(|| invalid("rail index out of range"))?;
        if let Some(old) = std::mem::replace(slot, surface) {
            self.detached.push(old);
        }
        Ok(())
    }

    /// `GetVertex(lIndex)`: the start (0) or end (other) vertex of the filleted edge.
    /// A missing edge signals `SM_ERR` upstream and returns `NULL`.
    ///
    /// # Errors
    ///
    /// Those of the topology queries.
    pub fn vertex(&self, b: &Brep, index: usize) -> Result<Option<VertexId>> {
        let Some(e0) = self.edgeuse(0)? else {
            return Ok(None);
        };
        let Some(e) = eu::edge(b, e0)? else {
            // SE(SM_ERR): signalled only
            return Ok(None);
        };
        let start = edge::start_vertex(b, e)?;
        if index == 0 {
            return Ok(start);
        }
        edge::other_vertex(b, e, start)
    }

    /// `FindIndexOfRailOnFace(pOrigFace)`: 0 when the first edge use's face is
    /// `face`, else 1 (0 without a first edge use, after signalling `SM_ERR`).
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for an edge use without loop use or face use
    /// (upstream dereferences `NULL`).
    pub fn find_index_of_rail_on_face(&self, b: &Brep, face: Option<FaceId>) -> Result<usize> {
        let Some(e0) = self.edgeuse(0)? else {
            return Ok(0);
        };
        if eu::loopuse(b, e0)?.is_none() {
            return Err(invalid("edge use without loop use"));
        }
        let fu = eu::faceuse(b, e0)?.ok_or_else(|| invalid("loop use without face use"))?;
        if eu::faceuse_face(b, fu)? == face {
            return Ok(0);
        }
        Ok(1)
    }

    /// `FindIndexOfRailXSideEdgeuse(pSideEU)`: the rail that the side edge use
    /// crosses.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] where upstream dereferences `NULL` (no first edge
    /// use, an edge use without edge, vertex use or counter-clockwise neighbour).
    pub fn find_index_of_rail_x_side_edgeuse(&self, b: &Brep, side_eu: EdgeuseId) -> Result<usize> {
        let mut rail = 0;
        let fil_eu = self
            .edgeuse(0)?
            .ok_or_else(|| invalid("no first edge use"))?;
        let side_edge =
            eu::edge(b, side_eu)?.ok_or_else(|| invalid("side edge use without edge"))?;
        if edge::is_closed(b, side_edge)? {
            // Such as when in 2x1 cases
            let v = vertex_of_edgeuse(b, fil_eu)?;
            let side_v = vertex_of_edgeuse(b, side_eu)?;
            if side_v == v {
                if eu::cw_edgeuse(b, fil_eu)? == Some(side_eu) {
                    rail = 1;
                }
            } else {
                let ccw = eu::ccw_edgeuse(b, fil_eu)?
                    .ok_or_else(|| invalid("edge use without counter-clockwise neighbour"))?;
                if eu::mate(b, ccw)? == Some(side_eu) {
                    rail = 1;
                }
            }
        } else {
            // Work from the faces of our two Edgeuses.
            rail = 0;
            if eu::face(b, fil_eu)? != eu::face(b, side_eu)? {
                rail = 1;
            }
        }
        Ok(rail)
    }

    /// `SetSurfacesToZeroOffset`: saves each surface's offset and sets it to zero.
    ///
    /// # Errors
    ///
    /// Those of [`FilletSolver::surface`].
    pub fn set_surfaces_to_zero_offset(&mut self) -> Result<()> {
        for i in 0..2 {
            let saved = match self.surface_mut(i)? {
                Some(s) => {
                    let d = s.surface.offset_distance;
                    s.surface.set_offset_distance(0.0);
                    Some(d)
                }
                None => None,
            };
            if let (Some(d), Some(slot)) = (saved, self.saved_offsets.get_mut(i)) {
                *slot = d;
            }
        }
        Ok(())
    }

    /// `ReloadSurfaceOffsets`: restores the offsets saved by
    /// [`FilletSolver::set_surfaces_to_zero_offset`].
    ///
    /// # Errors
    ///
    /// Those of [`FilletSolver::surface`].
    pub fn reload_surface_offsets(&mut self) -> Result<()> {
        for i in 0..2 {
            let d = self.saved_offsets.get(i).copied().unwrap_or(0.0);
            if let Some(s) = self.surface_mut(i)? {
                s.surface.set_offset_distance(d);
            }
        }
        Ok(())
    }

    /// `SetExtensionFlags`: extend each end whose corner is neither blending nor
    /// degenerate.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] where upstream dereferences `NULL` (missing edge
    /// uses, vertex uses or executive); [`SolidError::Unported`] for the corner
    /// lookup (`SmFilletExecutive::GetFilletCornerOfVertex`).
    pub fn set_extension_flags(&mut self, b: &Brep) -> Result<()> {
        let eu0 = self
            .edgeuse(0)?
            .ok_or_else(|| invalid("no first edge use"))?;
        let eu1 = self
            .edgeuse(1)?
            .ok_or_else(|| invalid("no second edge use"))?;
        let mut vert0 = vertex_of_edgeuse(b, eu0)?;
        let mut vert1 = vertex_of_edgeuse(b, eu1)?;

        // swap vertices for opposite oriented edgeuses
        if eu::orientation(b, eu0)? == OrientType::Opposite {
            std::mem::swap(&mut vert0, &mut vert1);
        }
        let _ = (vert0, vert1);

        // get fillet information for both vertices
        if self.executive.is_none() {
            return Err(invalid("no fillet executive"));
        }
        // UNPORTED(blend.fillet_executive_p1): SmFilletExecutive::GetFilletCornerOfVertex
        Err(SolidError::unported(
            "SmFilletExecutive::GetFilletCornerOfVertex",
            "blend.fillet_executive_p1",
        ))
    }

    /// `SetOffsetsData`: restores saved orientations and offset surfaces (a surface
    /// whose key differs from the saved one is replaced by the saved one).
    ///
    /// # Errors
    ///
    /// Those of [`FilletSolver::surface`] / [`FilletSolver::set_surface`].
    pub fn set_offsets_data(&mut self, data: &FilletSolverOffsetsData) -> Result<()> {
        for i in 0..2 {
            let orientation = data
                .orientations
                .get(i)
                .copied()
                .ok_or_else(|| invalid("rail index out of range"))?;
            if let Some(o) = self.orientations.get_mut(i) {
                *o = orientation;
            }
            let orig = data.surfaces.get(i).and_then(Option::as_ref);
            let current_key = self.surface(i)?.map(|s| s.key);
            if current_key != orig.map(|s| s.key) {
                // the current surface is deleted; the saved one is the live object
                // (detached since the save) when there is one
                let restored = orig.map(|o| {
                    self.detached
                        .iter()
                        .position(|d| d.key == o.key)
                        .map_or_else(|| o.clone(), |pos| self.detached.remove(pos))
                });
                let slot = self
                    .surfaces
                    .get_mut(i)
                    .ok_or_else(|| invalid("rail index out of range"))?;
                *slot = restored;
            }
        }
        Ok(())
    }

    /// `GetOffsetsData`: the current orientations and offset surfaces.
    ///
    /// # Errors
    ///
    /// Those of [`FilletSolver::surface`].
    pub fn offsets_data(&self) -> Result<FilletSolverOffsetsData> {
        let mut data = FilletSolverOffsetsData::default();
        for i in 0..2 {
            let o = self.orientation(i)?;
            if let Some(d) = data.orientations.get_mut(i) {
                *d = o;
            }
            let s = self.surface(i)?.cloned();
            if let Some(d) = data.surfaces.get_mut(i) {
                *d = s;
            }
        }
        Ok(data)
    }

    /// `SetupOffsetExtension(pSurf, rDomain)`: grows `domain` by the extension factor
    /// times its size at both ends (a fifth of that across a periodic direction), not
    /// across a singular side.
    ///
    /// # Errors
    ///
    /// Those of `IsPeriodic` and `IsSingularity`.
    pub fn setup_offset_extension(
        &self,
        k: &Kernel,
        surface: &Surface,
        domain: &mut Extent2d,
    ) -> Result<()> {
        // get length of extensions
        let domain_size = domain.size();
        let mut size = Vec2::new(
            self.surface_extension_factor * domain_size.x,
            self.surface_extension_factor * domain_size.y,
        );

        // Extend by a smaller amount if periodic.
        let periodic_extension_factor = 0.2;
        let linear_extension_factor = 50.0;
        if bool_status(surface.is_periodic(domain, SurfParamType::U))? {
            size.x *= periodic_extension_factor;
        } else if CBI_LINEAR > 0 && surface.degree(SurfParamType::U) == 1 {
            size.x *= linear_extension_factor;
        }
        if bool_status(surface.is_periodic(domain, SurfParamType::V))? {
            size.y *= periodic_extension_factor;
        } else if CBI_LINEAR > 0 && surface.degree(SurfParamType::V) == 1 {
            size.y *= linear_extension_factor;
        }

        // process the min domain boundaries
        let min = domain.min;
        let sing = singularity_status(surface.is_singularity(k, min, EFF_ZERO, false))?;
        if !sing.singular {
            // when surface is not singular - extend the min domain boundary
            domain.add_point(Point2::new(min.x - size.x, min.y - size.y));
        } else if sing.direction == SurfParamType::U {
            // singular in U direction only extend the u direction
            let mut new_min = domain.min;
            new_min.x -= size.x;
            domain.add_point(new_min);
        } else {
            // singular in V direction only extend the v direction
            let mut new_min = domain.min;
            new_min.y -= size.y;
            domain.add_point(new_min);
        }

        // process the max domain boundaries
        let max = domain.max;
        let sing = singularity_status(surface.is_singularity(k, max, EFF_ZERO, false))?;
        if !sing.singular {
            domain.add_point(Point2::new(max.x + size.x, max.y + size.y));
        } else if sing.direction == SurfParamType::U {
            let mut new_max = domain.max;
            new_max.x += size.x;
            domain.add_point(new_max);
        } else {
            let mut new_max = domain.max;
            new_max.y += size.y;
            domain.add_point(new_max);
        }
        Ok(())
    }

    /// `SplitFilletGeomAtSideEdge(pFilletGeom, pSplitFilVU, lWhichEnd,
    /// ppRetFilletGeom)`.
    ///
    /// # Errors
    ///
    /// [`SolidError::Unported`]: its first call (`SmFilletGeom::GetRailIndexOfVertexuse`).
    pub fn split_fillet_geom_at_side_edge(
        &mut self,
        _fillet_geom: FilletGeomRef,
        _split_vertexuse: FilletTopologyRef,
        _which_end: usize,
    ) -> Result<Option<FilletGeomRef>> {
        // UNPORTED(blend.fillet_geom_p6): SmFilletGeom::GetRailIndexOfVertexuse
        Err(SolidError::unported(
            "SmFilletGeom::GetRailIndexOfVertexuse",
            "blend.fillet_geom_p6",
        ))
    }

    /// `SplitFilletGeom(lFilletGeomIndex, eTrimType, crIvls1, crIvls2, crCC1, crCC2)`:
    /// one fillet geometry per interval pair, each with trimmed rails. Empty interval
    /// arrays do nothing.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for a geometry index past the solver's geometries or
    /// interval arrays of different lengths (upstream reads past the shorter one);
    /// [`SolidError::Unported`] for the geometry copies (`SmFilletGeom::SmFilletGeom`)
    /// and the rail vertices (`SmFilletGeom::GetRailVertex`).
    pub fn split_fillet_geom<O: ClassObject>(
        &mut self,
        fillet_geom_index: usize,
        _trim_type: i32,
        ivls1: &[Extent1d],
        ivls2: &[Extent1d],
        _cc1: &CurveClassification<O>,
        _cc2: &CurveClassification<O>,
    ) -> Result<()> {
        if ivls1.len() != ivls2.len() {
            return Err(invalid("interval arrays of different lengths"));
        }
        let fillet_geom = *self
            .fillet_geoms
            .get(fillet_geom_index)
            .ok_or_else(|| invalid("fillet geometry index out of range"))?;
        let mut new_geoms = Vec::new();
        for ii in 0..ivls1.len() {
            if ii + 1 < ivls1.len() {
                // UNPORTED(blend.fillet_geom_p5): SmFilletGeom::SmFilletGeom
                return Err(SolidError::unported(
                    "SmFilletGeom::SmFilletGeom",
                    "blend.fillet_geom_p5",
                ));
            }
            new_geoms.push(fillet_geom);
        }
        if new_geoms.is_empty() {
            return Ok(());
        }
        // UNPORTED(blend.fillet_geom_p6): SmFilletGeom::GetRailVertex
        Err(SolidError::unported(
            "SmFilletGeom::GetRailVertex",
            "blend.fillet_geom_p6",
        ))
    }

    /// `TrimRails`: trims every fillet geometry's rail curves.
    ///
    /// # Errors
    ///
    /// [`SolidError::Unported`] for any geometry (`SmFilletGeom::TrimRailCurves`).
    pub fn trim_rails(&mut self) -> Result<()> {
        if self.fillet_geoms.is_empty() {
            return Ok(());
        }
        // UNPORTED(blend.fillet_geom_p5): SmFilletGeom::TrimRailCurves
        Err(SolidError::unported(
            "SmFilletGeom::TrimRailCurves",
            "blend.fillet_geom_p5",
        ))
    }
}

/// An `SmBoolean` query whose failed evaluation upstream returns the status, read as
/// `TRUE` (`SER` inside a boolean function); [`SolidError::Unported`] propagates.
fn bool_status(r: Result<bool>) -> Result<bool> {
    match r {
        Ok(v) => Ok(v),
        Err(e @ SolidError::Unported { .. }) => Err(e),
        Err(_) => Ok(true),
    }
}

/// [`bool_status`] for `IsSingularity`: a failure reads as singular, direction
/// unknown (the V branch).
fn singularity_status(
    r: Result<crate::surface::surface::p5::Singularity>,
) -> Result<crate::surface::surface::p5::Singularity> {
    match r {
        Ok(v) => Ok(v),
        Err(e @ SolidError::Unported { .. }) => Err(e),
        Err(_) => Ok(crate::surface::surface::p5::Singularity {
            singular: true,
            direction: SurfParamType::Neither,
        }),
    }
}

fn vertex_of_edgeuse(b: &Brep, e: EdgeuseId) -> Result<Option<VertexId>> {
    let vu = eu::vertexuse(b, e)?.ok_or_else(|| invalid("edge use without vertex use"))?;
    vertexuse::vertex(b, vu)
}

// ---- virtual methods ---------------------------------------------------------------------

/// The virtual methods of `SmFilletSolver` (the base versions are the default methods)
/// with access to the base data.
pub trait FilletSolverOps {
    /// The `SmFilletSolver` base.
    fn solver(&self) -> &FilletSolver;

    /// The `SmFilletSolver` base, mutable.
    fn solver_mut(&mut self) -> &mut FilletSolver;

    /// `GetJacobianSize`: the base signals `SM_ERR` ("Called pure virtual method") and
    /// returns 0.
    fn jacobian_size(&self) -> usize {
        0
    }

    /// `SetupOffsetValues(rTsectPnt, pOffsetDist)`: the base returns `SM_ERR`.
    ///
    /// # Errors
    ///
    /// `SM_ERR` for the base.
    fn setup_offset_values(
        &mut self,
        _tsect: &TsectPnt,
        _offset_dist: Option<&mut f64>,
    ) -> Result<()> {
        Err(SolidError::sm_err(OP))
    }

    /// `OffsetRadiiCanChange`: `TRUE` for the base.
    fn offset_radii_can_change(&self) -> bool {
        true
    }

    /// `CanMakeAnalytic`: `!OffsetRadiiCanChange()` for the base.
    fn can_make_analytic(&self) -> bool {
        !self.offset_radii_can_change()
    }

    /// `LoadJacobian`: the base returns `SM_ERR` and touches nothing.
    ///
    /// # Errors
    ///
    /// `SM_ERR` for the base.
    fn load_jacobian(
        &mut self,
        _num_equations: &mut usize,
        _num_parameters: &mut usize,
        _load: &mut JacobianLoad<'_>,
        _f: &mut [f64],
        _jacobian: Option<&mut Matrix>,
    ) -> Result<bool> {
        Err(SolidError::sm_err(OP))
    }

    /// `LoadInitialValues(rbDoSurf2Calcs, lRailIndex, crSurface1, lSurface1Index,
    /// rpSurface2, rlSurface2Index, rExtents, rPeriodicities, rGuessT)`: the base
    /// returns `SM_ERR` and touches nothing.
    ///
    /// # Errors
    ///
    /// `SM_ERR` for the base.
    fn load_initial_values(
        &mut self,
        _k: &Kernel,
        _brep: Option<&Brep>,
        _do_surf2_calcs: bool,
        _rail_index: usize,
        _surface1: &OffsetSurface,
        _surface1_index: usize,
        _intervals: &mut ExtentNd,
        _periodicities: &mut Vec<bool>,
        _guess_t: &mut Vec<f64>,
    ) -> Result<Option<InitialValues>> {
        Err(SolidError::sm_err(OP))
    }

    /// `SetExtensionFlags` (virtual): the base version
    /// [`FilletSolver::set_extension_flags`].
    ///
    /// # Errors
    ///
    /// Those of [`FilletSolver::set_extension_flags`].
    fn set_extension_flags(&mut self, b: &Brep) -> Result<()> {
        self.solver_mut().set_extension_flags(b)
    }

    /// `GetType` (virtual): `SmFilletSolver_TYPE` for the base (each derived solver's
    /// `SM_COMMON` type belongs to its unit).
    fn type_code(&self) -> i64 {
        FilletSolver::TYPE
    }

    /// `IsKindOf` (virtual): the base's `SmFilletSolver_TYPE` / `SmObject_TYPE`.
    fn is_kind_of(&self, t: i64) -> bool {
        FilletSolver::is_kind_of(t)
    }
}

impl FilletSolverOps for FilletSolver {
    fn solver(&self) -> &FilletSolver {
        self
    }

    fn solver_mut(&mut self) -> &mut FilletSolver {
        self
    }
}

/// `GetSurface(rail)->GetBaseSurface()` as the identity `LoadJacobian` compares.
fn base_id(s: &FilletSolver, rail: usize) -> Result<Option<SurfaceId>> {
    Ok(s.surface(rail)?.map(|slot| slot.surface.base_id))
}

/// `Evaluate1stDerivatives(uv, TRUE, TRUE, ..)` of an offset surface.
fn offset_evaluate_1st(_s: &OffsetSurface, _uv: Point2) -> Result<(Point3, Vec3, Vec3)> {
    // UNPORTED(surface.offset_surface): SmOffsetSurface::Evaluate
    Err(SolidError::unported(
        "SmOffsetSurface::Evaluate",
        "surface.offset_surface",
    ))
}

/// `EvaluateNormal(uv, TRUE, TRUE, ..)` of an offset surface.
fn offset_evaluate_normal(_s: &OffsetSurface, _uv: Point2) -> Result<Vec3> {
    // UNPORTED(surface.offset_surface): SmOffsetSurface::Evaluate
    Err(SolidError::unported(
        "SmOffsetSurface::Evaluate",
        "surface.offset_surface",
    ))
}

fn x_at(x: &[f64], i: usize) -> Result<f64> {
    x.get(i)
        .copied()
        .ok_or_else(|| invalid("crX index out of range"))
}

fn set_f(f: &mut [f64], i: usize, v: f64) -> Result<()> {
    let slot = f
        .get_mut(i)
        .ok_or_else(|| invalid("rF index out of range"))?;
    *slot = v;
    Ok(())
}

fn f_at(f: &[f64], i: usize) -> Result<f64> {
    f.get(i)
        .copied()
        .ok_or_else(|| invalid("rF index out of range"))
}

fn set_j(j: &mut Matrix, r: usize, c: usize, v: f64) -> Result<()> {
    j.set_at(r, c, v)
        .map_err(|_| invalid("Jacobian index out of range"))
}

/// Zeroes the Jacobian and the matching `rF` rows, or all of `rF` without a Jacobian.
fn zero_outputs(f: &mut [f64], jacobian: Option<&mut Matrix>) -> Result<()> {
    if let Some(j) = jacobian {
        for ll in 0..j.num_rows() {
            for mm in 0..j.num_columns() {
                set_j(j, ll, mm, 0.0)?;
            }
            set_f(f, ll, 0.0)?;
        }
    } else {
        for v in f.iter_mut() {
            *v = 0.0;
        }
    }
    Ok(())
}

// ---- SmFilletSphereSolveENFO -----------------------------------------------------------

/// `SmFilletSphereSolveENFO`: the solver's equations plus the distance of the first
/// surface point to a plane (the obsolete sphere centre and radius seed the plane).
pub struct FilletSphereSolveEnfo<'a> {
    /// `m_vSphereCenter`.
    pub sphere_center: Point3,
    /// `m_dSphereRadius`.
    pub sphere_radius: f64,
    /// `m_vPlaneABC`: plane normal.
    pub plane_abc: Vec3,
    /// `m_dPlaneD`: plane offset.
    pub plane_d: f64,
    /// `m_bPlaneAdjusts`: recompute the plane from the current solution.
    pub plane_adjusts: bool,
    solver: &'a mut dyn FilletSolverOps,
}

impl<'a> FilletSphereSolveEnfo<'a> {
    /// `SmFilletSphereSolveENFO(crSphereCenter, dSphereRadius, crFilletSolver)`: the
    /// plane starts as `(centre, radius)`, not adjusting.
    pub fn new(
        sphere_center: Point3,
        sphere_radius: f64,
        solver: &'a mut dyn FilletSolverOps,
    ) -> Self {
        Self {
            sphere_center,
            sphere_radius,
            plane_abc: sphere_center,
            plane_d: sphere_radius,
            plane_adjusts: false,
            solver,
        }
    }

    /// `SetPlaneAdjusts`.
    pub fn set_plane_adjusts(&mut self, plane_adjusts: bool) {
        self.plane_adjusts = plane_adjusts;
    }

    /// `Evaluate(crX, rF, pOptJacobian, rbFoundAnswer)`: zeroes the outputs, lets the
    /// solver load every row but the first (`LoadJacobian` from row 1, parameter 2),
    /// then sets row 0 to the plane distance of the first offset surface's point at
    /// `crX[0..2]`. Returns `rbFoundAnswer`.
    ///
    /// # Errors
    ///
    /// `SM_ERR` when `LoadJacobian` fails; those of the evaluations;
    /// [`SolidError::InvalidInput`] for arrays too short (upstream reads or writes past
    /// them) or a missing first surface.
    pub fn evaluate(
        &mut self,
        x: &[f64],
        f: &mut [f64],
        mut jacobian: Option<&mut Matrix>,
    ) -> Result<bool> {
        const OP_E: &str = "blend::fillet_solver::p1::FilletSphereSolveEnfo::evaluate";
        if let Some(j) = jacobian.as_deref() {
            if f.len() < j.num_rows() {
                return Err(invalid("rF shorter than the Jacobian"));
            }
        }
        zero_outputs(f, jacobian.as_deref_mut())?;

        let mut num_equations = 1;
        let mut num_parameters = 2;
        let surface1 = base_id(self.solver.solver(), 0)?;
        let mut load = JacobianLoad {
            rail_index: 0,
            surface1,
            surf1_offset: 0,
            surface2: None,
            surf2_offset: 0,
            x,
        };
        let found_answer = self
            .solver
            .load_jacobian(
                &mut num_equations,
                &mut num_parameters,
                &mut load,
                f,
                jacobian.as_deref_mut(),
            )
            .map_err(|e| as_sm_err(e, OP_E))?;
        // rlSurf2Offset as LoadJacobian left it
        let surf2_offset = load.surf2_offset;

        // calc OffsetSurface0 current solution surface values
        let uv = Point2::new(x_at(x, 0)?, x_at(x, 1)?);
        let s0 = self
            .solver
            .solver()
            .surface(0)?
            .ok_or_else(|| invalid("no first offset surface"))?
            .surface
            .clone();
        let (sf, sduf, sdvf) = offset_evaluate_1st(&s0, uv)?;

        // update the plane to fit the current solution
        if self.plane_adjusts {
            let mut norm1 = sduf.cross(sdvf);
            if norm1.length_squared() < EFF_ZERO_SQ {
                norm1 = offset_evaluate_normal(&s0, uv)?;
            }
            let norm1 = norm1.unitize()?;

            let uv2 = Point2::new(x_at(x, surf2_offset)?, x_at(x, surf2_offset + 1)?);
            let s1 = self
                .solver
                .solver()
                .surface(1)?
                .ok_or_else(|| invalid("no second offset surface"))?
                .surface
                .clone();
            let (sg, sdug, sdvg) = offset_evaluate_1st(&s1, uv2)?;
            let mut norm2 = sdug.cross(sdvg);
            if norm2.length_squared() < EFF_ZERO_SQ {
                norm2 = offset_evaluate_normal(&s1, uv2)?;
            }
            let norm2 = norm2.unitize()?;

            let plane_orig = (sf + sg) / 2.0;
            let mut plane_abc = norm1.cross(norm2);

            // preserve plane normal orientation
            if plane_abc.dot(self.plane_abc) < 0.0 {
                plane_abc = -plane_abc;
            }
            self.plane_abc = plane_abc;
            self.plane_d = -self.plane_abc.dot(plane_orig);
        }

        // set rF 1st row
        set_f(f, 0, self.plane_abc.dot(sf) + self.plane_d)?;
        if let Some(j) = jacobian {
            set_j(j, 0, 0, self.plane_abc.dot(sduf))?;
            set_j(j, 0, 1, self.plane_abc.dot(sdvf))?;
        }

        let mut found = false;
        if found_answer {
            let scaled_tol = self.solver.solver().conversion_tol() * (1.0 + sf.max_dimension());
            if f_at(f, 0)?.abs() < scaled_tol {
                found = true;
            }
        }
        Ok(found)
    }
}

// ---- SmRailUVCurveIntersectENFO -------------------------------------------------------

/// `SmRailUVCurveIntersectENFO`: a fillet rail meeting a UV curve on one of the
/// surfaces; `crX = [t u v ...]` with `t` on the UV curve and `(u, v)` the rail's UV
/// point.
pub struct RailUvCurveIntersectEnfo<'a> {
    /// `m_crUVCurve`.
    pub uv_curve: &'a BSplineCurve,
    /// `m_vInterval`.
    pub interval: Extent1d,
    /// `m_lRailIndex`.
    pub rail_index: usize,
    solver: &'a mut dyn FilletSolverOps,
}

impl<'a> RailUvCurveIntersectEnfo<'a> {
    /// `SmRailUVCurveIntersectENFO(crUVCurve, crInterval, crSurfaceOfCurve, lRailIndex,
    /// crFilletSolver)` (the surface of the curve is only drawn upstream).
    pub fn new(
        uv_curve: &'a BSplineCurve,
        interval: Extent1d,
        rail_index: usize,
        solver: &'a mut dyn FilletSolverOps,
    ) -> Self {
        Self {
            uv_curve,
            interval,
            rail_index,
            solver,
        }
    }

    /// `Evaluate(crX, rF, pOptJacobian, rbFoundAnswer)`: rows 0 and 1 are the UV curve
    /// point at `crX[0]` minus `crX[1..3]`, the other rows (and Jacobian rows) are
    /// zeroed when a Jacobian is given, then the solver loads its rows from row 2,
    /// parameter 3. Returns `rbFoundAnswer`.
    ///
    /// # Errors
    ///
    /// `SM_ERR` when `LoadJacobian` fails; those of the curve evaluation;
    /// [`SolidError::InvalidInput`] for arrays too short.
    pub fn evaluate(
        &mut self,
        x: &[f64],
        f: &mut [f64],
        mut jacobian: Option<&mut Matrix>,
    ) -> Result<bool> {
        const OP_E: &str = "blend::fillet_solver::p1::RailUvCurveIntersectEnfo::evaluate";
        let pv = self.uv_curve.evaluate(x_at(x, 0)?, 1, true, false)?;
        let p0 = pv
            .first()
            .copied()
            .ok_or_else(|| invalid("short curve evaluation"))?;
        let p1 = pv
            .get(1)
            .copied()
            .ok_or_else(|| invalid("short curve evaluation"))?;

        set_f(f, 0, p0.x - x_at(x, 1)?)?;
        set_f(f, 1, p0.y - x_at(x, 2)?)?;
        if let Some(j) = jacobian.as_deref_mut() {
            set_j(j, 0, 0, p1.x)?;
            set_j(j, 0, 1, -1.0)?;
            set_j(j, 0, 2, 0.0)?;
            set_j(j, 1, 0, p1.y)?;
            set_j(j, 1, 1, 0.0)?;
            set_j(j, 1, 2, -1.0)?;
            for kk in 3..j.num_columns() {
                set_j(j, 0, kk, 0.0)?;
                set_j(j, 1, kk, 0.0)?;
            }
            for ll in 2..j.num_rows() {
                for mm in 0..j.num_columns() {
                    set_j(j, ll, mm, 0.0)?;
                }
                set_f(f, ll, 0.0)?;
            }
        }

        let mut num_equations = 2;
        let mut num_parameters = 3;
        let surface1 = base_id(self.solver.solver(), self.rail_index)?;
        let mut load = JacobianLoad {
            rail_index: self.rail_index,
            surface1,
            surf1_offset: 1,
            surface2: None,
            surf2_offset: 0,
            x,
        };
        let found_answer = self
            .solver
            .load_jacobian(
                &mut num_equations,
                &mut num_parameters,
                &mut load,
                f,
                jacobian,
            )
            .map_err(|e| as_sm_err(e, OP_E))?;

        let mut found = false;
        if found_answer {
            let scaled_tol = self.solver.solver().conversion_tol() * (1.0 + p0.max_dimension());
            if f_at(f, 0)?.abs() < scaled_tol && f_at(f, 1)?.abs() < scaled_tol {
                found = true;
            }
        }
        Ok(found)
    }
}

// ---- SmRailRailIntersectENFO ----------------------------------------------------------

/// `SmRailRailIntersectENFO`: the rails of two solvers meeting on a shared surface.
pub struct RailRailIntersectEnfo<'a> {
    /// `m_lRailIndex`.
    pub rail_index: usize,
    /// `m_lOtherRailIndex`.
    pub other_rail_index: usize,
    solver: &'a mut dyn FilletSolverOps,
    other_solver: &'a mut dyn FilletSolverOps,
}

impl<'a> RailRailIntersectEnfo<'a> {
    /// `SmRailRailIntersectENFO(lRailIndex, crFilletSolver, lOtherRailIndex,
    /// crOtherFilletSolver)`.
    pub fn new(
        rail_index: usize,
        solver: &'a mut dyn FilletSolverOps,
        other_rail_index: usize,
        other_solver: &'a mut dyn FilletSolverOps,
    ) -> Self {
        Self {
            rail_index,
            other_rail_index,
            solver,
            other_solver,
        }
    }

    /// `Evaluate(crX, rF, pOptJacobian, rbFoundAnswer)`: zeroes the Jacobian rows and
    /// matching `rF` rows, then both solvers load their rows (the second continuing
    /// from the counts the first left). Returns `rbFoundAnswer`.
    ///
    /// # Errors
    ///
    /// `SM_ERR` when either `LoadJacobian` fails; [`SolidError::InvalidInput`] for
    /// arrays too short.
    pub fn evaluate(
        &mut self,
        x: &[f64],
        f: &mut [f64],
        mut jacobian: Option<&mut Matrix>,
    ) -> Result<bool> {
        const OP_E: &str = "blend::fillet_solver::p1::RailRailIntersectEnfo::evaluate";
        if let Some(j) = jacobian.as_deref_mut() {
            for ll in 0..j.num_rows() {
                for mm in 0..j.num_columns() {
                    set_j(j, ll, mm, 0.0)?;
                }
                set_f(f, ll, 0.0)?;
            }
        }

        let mut num_equations = 0;
        let mut num_parameters = 2;
        let surface1 = base_id(self.solver.solver(), self.rail_index)?;
        let mut load = JacobianLoad {
            rail_index: self.rail_index,
            surface1,
            surf1_offset: 0,
            surface2: None,
            surf2_offset: 0,
            x,
        };
        let found1 = self
            .solver
            .load_jacobian(
                &mut num_equations,
                &mut num_parameters,
                &mut load,
                f,
                jacobian.as_deref_mut(),
            )
            .map_err(|e| as_sm_err(e, OP_E))?;

        // pSurface, lSurf1Offset and lSurf2Offset as the first call left them;
        // pSurface2 is reset to NULL
        let mut load2 = JacobianLoad {
            rail_index: self.other_rail_index,
            surface1: load.surface1,
            surf1_offset: load.surf1_offset,
            surface2: None,
            surf2_offset: load.surf2_offset,
            x,
        };
        let found2 = self
            .other_solver
            .load_jacobian(
                &mut num_equations,
                &mut num_parameters,
                &mut load2,
                f,
                jacobian,
            )
            .map_err(|e| as_sm_err(e, OP_E))?;
        Ok(found1 && found2)
    }
}

// ---- SmFindClippedRadiusEFO -----------------------------------------------------------

/// `SmFindClippedRadiusEFO`: the constant-distance fillet radius at which the
/// offset/offset intersection crosses an iso boundary of one offset surface.
pub struct FindClippedRadiusEfo<'a> {
    solver: &'a mut FilletSolver,
    surf_index: usize,
    /// `m_eSurfParam`.
    pub surf_param: SurfParamType,
    /// `m_dIsoParam`.
    pub iso_param: f64,
    /// `m_rUV`: the surface's UV point (updated).
    pub uv: Vec2,
    /// `m_rOtherUV`: the other surface's UV point (updated).
    pub other_uv: Vec2,
    /// `m_dOrientation`.
    pub orientation: f64,
    /// `m_dOtherOrientation`.
    pub other_orientation: f64,
    /// `m_OtherDomain`.
    pub other_domain: Extent2d,
    /// `m_dTargetDistance`.
    pub target_distance: f64,
    /// `m_dThisApproxTol3d`.
    pub approx_tol_3d: f64,
    /// `m_dGuessT`.
    pub guess_t: f64,
}

impl<'a> FindClippedRadiusEfo<'a> {
    /// `SmFindClippedRadiusEFO(pFS, lSurfIndex, sUVDomains, eSurfParam, dIsoParam,
    /// sUV)`, with the solver's `GetDistance()` as `target_distance`.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for a surface index above 1.
    pub fn new(
        solver: &'a mut FilletSolver,
        surf_index: usize,
        uv_domains: &[Extent2d; 2],
        surf_param: SurfParamType,
        iso_param: f64,
        uv: [Vec2; 2],
        target_distance: f64,
    ) -> Result<Self> {
        if surf_index > 1 {
            return Err(invalid("surface index out of range"));
        }
        let other = 1 - surf_index;
        let orientation = solver.orientation(surf_index)?;
        let other_orientation = solver.orientation(other)?;
        let this_uv = *uv
            .get(surf_index)
            .ok_or_else(|| invalid("surface index out of range"))?;
        let other_uv = *uv
            .get(other)
            .ok_or_else(|| invalid("surface index out of range"))?;
        let other_domain = *uv_domains
            .get(other)
            .ok_or_else(|| invalid("surface index out of range"))?;
        let approx_tol_3d = solver.this_approx_tol_3d();
        Ok(Self {
            solver,
            surf_index,
            surf_param,
            iso_param,
            uv: this_uv,
            other_uv,
            orientation,
            other_orientation,
            other_domain,
            target_distance,
            approx_tol_3d,
            guess_t: this_uv.x,
        })
    }

    fn set_offsets(&mut self, radius: f64) -> Result<()> {
        let (i, o) = (self.surf_index, 1 - self.surf_index);
        let d = radius * self.orientation;
        let od = radius * self.other_orientation;
        self.solver
            .surface_mut(i)?
            .ok_or_else(|| invalid("no offset surface"))?
            .surface
            .set_offset_distance(d);
        self.solver
            .surface_mut(o)?
            .ok_or_else(|| invalid("no other offset surface"))?
            .surface
            .set_offset_distance(od);
        Ok(())
    }

    /// `m_pOtherSurface->LocalCurveIntersect(m_OtherDomain, sIsoCurve, ...)` with the
    /// iso curve of the offset surface.
    fn intersect_iso(&self) -> Result<Option<(Point2, f64)>> {
        let _ = (
            self.other_domain,
            self.approx_tol_3d,
            self.other_uv,
            self.guess_t,
        );
        // UNPORTED(surface.offset_surface): SmOffsetSurface::Evaluate (SmIsoCurve of an offset surface)
        Err(SolidError::unported(
            "SmOffsetSurface::Evaluate",
            "surface.offset_surface",
        ))
    }

    fn base_points(&self, uv: Point2, other_uv: Point2) -> Result<f64> {
        let (i, o) = (self.surf_index, 1 - self.surf_index);
        let base = &self
            .solver
            .surface(i)?
            .ok_or_else(|| invalid("no offset surface"))?
            .surface
            .base;
        let other_base = &self
            .solver
            .surface(o)?
            .ok_or_else(|| invalid("no other offset surface"))?
            .surface
            .base;
        let p = evaluate_point_out_of_bounds(base, uv)?;
        let op = evaluate_point_out_of_bounds(other_base, other_uv)?;
        Ok(p.distance(op))
    }

    /// `Evaluate(dRadius, rdFOfT, rdFPrimeOfT, rbFoundAnswer)`: `F` is the rail point
    /// distance minus the target distance at the boundary crossing for `radius`, `F'`
    /// its forward difference over `SM_ZONE_TOL_3D`. Returns `(F, F', found)`.
    ///
    /// # Errors
    ///
    /// `SM_ERR` when the iso curve misses the other surface; those of the evaluations;
    /// [`SolidError::Unported`] (module notes).
    pub fn evaluate(&mut self, radius: f64) -> Result<(f64, f64, bool)> {
        const OP_E: &str = "blend::fillet_solver::p1::FindClippedRadiusEfo::evaluate";
        let mut radius = radius;
        let mut found = false;
        self.set_offsets(radius)?;
        let Some((other_uv_found, t_found)) = self.intersect_iso()? else {
            return Err(SolidError::sm_err(OP_E));
        };
        self.other_uv = other_uv_found;
        self.guess_t = t_found;
        if self.surf_param == SurfParamType::U {
            self.uv.x = self.iso_param;
            self.uv.y = t_found;
        } else {
            self.uv.x = t_found;
            self.uv.y = self.iso_param;
        }
        let distance = self.base_points(self.uv, self.other_uv)?;
        let f_of_t = distance - self.target_distance;
        if f_of_t.abs() < EFF_ZERO {
            found = true;
        }

        let step_size = ZONE_TOL_3D;
        radius += step_size;
        self.set_offsets(radius)?;
        let Some((other_uv_found, t_found)) = self.intersect_iso()? else {
            return Err(SolidError::sm_err(OP_E));
        };
        let uv_found = if self.surf_param == SurfParamType::U {
            Point2::new(self.iso_param, t_found)
        } else {
            Point2::new(t_found, self.iso_param)
        };
        let distance1 = self.base_points(uv_found, other_uv_found)?;
        let f_prime = (distance1 - self.target_distance - f_of_t) / step_size;
        Ok((f_of_t, f_prime, found))
    }
}

/// `EvaluatePoint` of a base surface with `SmBSplineSurface` out-of-bounds evaluation
/// temporarily enabled (`SmTemporaryChangeValue` on `GetOutOfBoundsEnabled`).
fn evaluate_point_out_of_bounds(base: &Surface, uv: Point2) -> Result<Point3> {
    let mut s = base.clone();
    if let crate::surface::surface::SurfaceKind::BSpline(b) = s.kind_mut() {
        *b.out_of_bounds_enabled_mut() = true;
    }
    s.evaluate_point(uv)
}

// ---- ComputeSurfaceValues / ComputeUVVectors --------------------------------------------

impl FilletSolver {
    /// `ComputeSurfaceValues(aUVValues, rTsectPnt)` dispatched on `ops` (its
    /// `SetupOffsetValues`): positions, first derivatives and unit normals of both
    /// offset surfaces, the curve position and the tangent plane angle (folded to at
    /// most `pi / 2`).
    ///
    /// # Errors
    ///
    /// Those of `SetupOffsetValues` (`SM_ERR` for the base solver) and of the
    /// evaluations; `SM_ERR` for a zero normal; [`SolidError::InvalidInput`] for a
    /// missing surface.
    pub fn compute_surface_values(
        ops: &mut dyn FilletSolverOps,
        uv_values: [Point2; 2],
        tsect: &mut TsectPnt,
    ) -> Result<()> {
        let snapshot = tsect.clone();
        ops.setup_offset_values(&snapshot, None)?;
        for (srf, uv) in uv_values.iter().enumerate() {
            let s = ops
                .solver()
                .surface(srf)?
                .ok_or_else(|| invalid("no offset surface"))?
                .surface
                .clone();
            let (p, du, dv) = offset_evaluate_1st(&s, *uv)?;
            let pv = tsect
                .surface_pv
                .get_mut(srf)
                .ok_or_else(|| invalid("surface index out of range"))?;
            pv[0][0] = p;
            pv[1][0] = du;
            pv[0][1] = dv;
            pv[2][0] = Vec3::new(0.0, 0.0, 0.0);
            pv[1][1] = Vec3::new(0.0, 0.0, 0.0);
            pv[0][2] = Vec3::new(0.0, 0.0, 0.0);
            let mut norm = du.cross(dv);
            if norm.length_squared() < EFF_ZERO_SQ {
                norm = offset_evaluate_normal(&s, *uv)?;
                if norm.length_squared() < EFF_ZERO_SQ {
                    return Err(SolidError::sm_err(OP));
                }
            }
            // sNorm.Unitize() unchecked: a failure leaves (0, 0, 1)
            let norm = norm.unitize().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
            pv[2][2] = norm;
            if let Some(uvp) = tsect.uv_curve_pv.get_mut(srf) {
                uvp[0] = *uv;
            }
        }
        tsect.curve_pv[0] = tsect.surface_pv[0][0][0];
        let n0 = tsect.surface_pv[0][2][2];
        let n1 = tsect.surface_pv[1][2][2];
        tsect.tangent_plane_angle_rad = n0.angle_between(n1)?;
        if tsect.tangent_plane_angle_rad > PI / 2.0 {
            tsect.tangent_plane_angle_rad = PI - tsect.tangent_plane_angle_rad;
        }
        Ok(())
    }
}

/// `SmFilletSolver::ComputeUVVectors(rTsectPnt)`: each surface's UV tangent of the
/// curve tangent (`smsurf_DropVectors` on the surface's first derivatives).
///
/// # Errors
///
/// Those of `smsurf_DropVectors` (the first surface's result is kept when the second
/// fails, as upstream).
pub fn compute_uv_vectors(tsect: &mut TsectPnt) -> Result<()> {
    for j in 0..2 {
        let pv = tsect
            .surface_pv
            .get(j)
            .ok_or_else(|| invalid("surface index out of range"))?;
        let du = pv[1][0];
        let dv = pv[0][1];
        let dropped = drop_vectors(du, dv, &[tsect.curve_pv[1]])?;
        let v = dropped
            .first()
            .copied()
            .ok_or_else(|| invalid("short drop result"))?;
        if let Some(uvp) = tsect.uv_curve_pv.get_mut(j) {
            uvp[1] = v;
        }
    }
    Ok(())
}

/// `SmTol::GetAngTolRad(const SmFilletSolver*)`: the solver's angle tolerance, the
/// system one without a solver.
#[must_use]
pub fn tol_ang_tol_rad(solver: Option<&FilletSolver>) -> f64 {
    solver.map_or_else(
        crate::geom::tol::ang_tol_rad,
        FilletSolver::this_ang_tol_rad,
    )
}

// ---- static helpers --------------------------------------------------------------------

/// `sm_CheckEdgeCurveDomains(pContext, pEdge, pCurve, bExtendBefore, bExtendAfter,
/// sDelCurve)`: when the edge's curve reaches beyond the edge's interval (by more than
/// `SM_EFF_ZERO`) at an end that is not extended, a copy trimmed to the edge interval
/// at those ends. Returns the copy (`TRUE`, `pCurve` the copy) or `None` (`FALSE`,
/// `pCurve` the edge's curve).
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for an edge without curve (upstream dereferences
/// `NULL`); the trim's status is ignored as upstream ([`SolidError::Unported`] still
/// propagates).
pub fn check_edge_curve_domains(
    k: &Kernel,
    b: &Brep,
    e: EdgeId,
    extend_before: bool,
    extend_after: bool,
) -> Result<Option<Curve>> {
    let curve = edge::curve(b, e)?.ok_or_else(|| invalid("edge without curve"))?;
    let edge_ivl = edge::interval(b, e)?;
    let curv_ivl = curve.natural_interval();
    let mut t0 = curv_ivl.min();
    let mut t1 = curv_ivl.max();
    let mut trim = false;
    if !extend_before && curv_ivl.min() < edge_ivl.min() - EFF_ZERO {
        trim = true;
        t0 = edge_ivl.min();
    }
    if !extend_after && curv_ivl.max() > edge_ivl.max() + EFF_ZERO {
        trim = true;
        t1 = edge_ivl.max();
    }
    if trim {
        if let Some(mut copy) = ignored(curve.copy(k))? {
            let mut trim_ivl = Extent1d::from_min_max(t0, t1);
            ignore_status(copy.trim(k, &mut trim_ivl, TrimMode::Notify))?;
            return Ok(Some(copy));
        }
    }
    Ok(None)
}

/// `sm_TestAnalyticSurface(pSurface)`: a plane, or a surface that is planar
/// (`IsPlanar()`, tolerance `SM_EFF_ZERO`) or a canonical cone. (`SmCone` is not a
/// surface kind here.)
///
/// # Errors
///
/// Those of `IsPlanar`; [`SolidError::Unported`] for the cone test of a non-planar
/// surface (`SmCone::IsNurbSurfaceCone`).
pub fn test_analytic_surface(surface: &Surface) -> Result<bool> {
    if matches!(
        surface.kind(),
        crate::surface::surface::SurfaceKind::Plane(_)
    ) {
        return Ok(true);
    }
    if surface.is_planar(EFF_ZERO)?.is_planar {
        return Ok(true);
    }
    // UNPORTED(surface.cone_p2): SmCone::IsNurbSurfaceCone
    Err(SolidError::unported(
        "SmCone::IsNurbSurfaceCone",
        "surface.cone_p2",
    ))
}

/// `sm_TestManifoldTangentRollover(pE, rbOppositeNormals, dTangencyTolRad,
/// iDebugLevel)`: whether the faces at the edge are tangent, and whether their
/// faceuse normals oppose. Returns `(tangent, opposite_normals)`.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for an edge without primary edge use;
/// [`SolidError::Unported`] for the binormals (`SmEdgeuse::EvaluateBinormal`).
pub fn test_manifold_tangent_rollover(
    b: &Brep,
    e: EdgeId,
    tangency_tol_rad: f64,
) -> Result<(bool, bool)> {
    let primary = eu::primary_edgeuse(b, e)?.ok_or_else(|| invalid("edge without edge use"))?;
    let test_param = edge::interval(b, e)?.evaluate(0.34);
    let _ = (primary, test_param, tangency_tol_rad);
    // UNPORTED(topology.edgeuse_p3): SmEdgeuse::EvaluateBinormal
    Err(SolidError::unported(
        "SmEdgeuse::EvaluateBinormal",
        "topology.edgeuse_p3",
    ))
}

/// `sm_TestSideEdgeConvexity(pSideEdge, bIsTangentialSideEdge)`: whether a radial
/// sector of the side edge is convex.
///
/// # Errors
///
/// [`SolidError::Unported`] for any edge use (`SmBrep::GetInfiniteRegion`).
pub fn test_side_edge_convexity(b: &Brep, side_edge: EdgeId, is_tangential: bool) -> Result<bool> {
    let eus = edge::edgeuses(b, side_edge, None, None)?;
    let _ = is_tangential;
    if eus.is_empty() {
        return Ok(false);
    }
    // UNPORTED(topology.brep_p1): SmBrep::GetInfiniteRegion
    Err(SolidError::unported(
        "SmBrep::GetInfiniteRegion",
        "topology.brep_p1",
    ))
}

/// `sm_FindNextGoodInterval(pCC, lStartSearch, rbFound, rlFoundIndex)`: the first
/// interval from `start` whose interior is on a face or an edge.
#[must_use]
pub fn find_next_good_interval<O: ClassObject>(
    cc: &CurveClassification<O>,
    start: usize,
) -> Option<usize> {
    cc.intervals()
        .iter()
        .enumerate()
        .skip(start)
        .find(|(_, ivl)| {
            let pc = ivl.mid.point_class();
            pc == PointClassType::Face || pc == PointClassType::Edge
        })
        .map(|(i, _)| i)
}

/// `sm_TrimRail(pRail, crTrimInterval)`: trims the rail's 3D curve and the UV trim
/// curves of its primary edge use and of the mate (`SmEdgeuse::GetUVTrimCurve`, which
/// reads the mate's curve when the edge use holds none), each to the interval the
/// previous trim may have snapped.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for a rail without primary edge use or mate (upstream
/// dereferences `NULL`); `SM_ERR_NULL_POINTER` for a curve that is not an
/// `SmBSplineCurve` or a missing UV curve (`NER`); those of the trims.
pub fn trim_rail(k: &Kernel, b: &mut Brep, rail: EdgeId, trim_interval: &Extent1d) -> Result<()> {
    const OP_T: &str = "blend::fillet_solver::p1::trim_rail";
    let prim = eu::primary_edgeuse(b, rail)?.ok_or_else(|| invalid("rail without edge use"))?;
    let mate = eu::mate(b, prim)?.ok_or_else(|| invalid("rail edge use without mate"))?;
    let is_bspline = edge::curve(b, rail)?.is_some_and(|c| c.is_kind_of(BSplineCurve::TYPE));
    if !is_bspline {
        return Err(SolidError::null_pointer(OP_T));
    }
    let (uv, uv_owner) = eu::uv_trim_curve(b, prim)?;
    let uv_owner = uv_owner
        .filter(|_| uv.is_some())
        .ok_or(SolidError::null_pointer(OP_T))?;
    let (orig, orig_owner) = eu::uv_trim_curve(b, mate)?;
    let orig_owner = orig_owner
        .filter(|_| orig.is_some())
        .ok_or(SolidError::null_pointer(OP_T))?;
    let mut trim = *trim_interval;
    let missing = || SolidError::null_pointer(OP_T);
    b.edge_mut(rail)?
        .curve
        .as_mut()
        .ok_or_else(missing)?
        .trim(k, &mut trim, TrimMode::Notify)?;
    b.edgeuse_mut(uv_owner)?
        .uv_trim_curve
        .as_mut()
        .ok_or_else(missing)?
        .trim(k, &mut trim, TrimMode::Notify)?;
    b.edgeuse_mut(orig_owner)?
        .uv_trim_curve
        .as_mut()
        .ok_or_else(missing)?
        .trim(k, &mut trim, TrimMode::Notify)?;
    Ok(())
}

/// `sm_SplitRail(pFG, pRailToSplit, rpNewVert, rpNewRail)`.
///
/// # Errors
///
/// [`SolidError::Unported`]: its first call (`SmFilletGeom::GetFilletSolver`).
pub fn split_rail(
    _fillet_geom: FilletGeomRef,
    _rail: FilletTopologyRef,
) -> Result<(FilletTopologyRef, FilletTopologyRef)> {
    // UNPORTED(blend.fillet_geom_p1): SmFilletGeom::GetFilletSolver
    Err(SolidError::unported(
        "SmFilletGeom::GetFilletSolver",
        "blend.fillet_geom_p1",
    ))
}

/// `sm_ComputeSupportEdge(crContext, pSupportEdge, pFilletSurface, dThisApproxTol3d,
/// dAngleTolerance, iDebugLevel)`.
///
/// # Errors
///
/// [`SolidError::Unported`]: the support edge is a pseudo-B-rep `SmFilletEdge`
/// (`SmFilletVertex::IsProcessed`).
pub fn compute_support_edge(
    _support_edge: FilletTopologyRef,
    _approx_tol_3d: f64,
    _angle_tolerance: f64,
) -> Result<()> {
    // UNPORTED(blend.fillet_geom_p1): SmFilletVertex::IsProcessed
    Err(SolidError::unported(
        "SmFilletVertex::IsProcessed",
        "blend.fillet_geom_p1",
    ))
}
