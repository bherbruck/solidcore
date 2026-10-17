//! Rail classification and rollover processing for a fillet solver.
//!
//! Upstream: `source/SMLib/src/SmFilletSolver.cpp` (5714–5822, 7161–7899).
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`FilletSolver::test_roll_over`] | `SmFilletSolver::TestRollOver` |
//! | [`FilletSolver::process_roll_over`] | `SmFilletSolver::ProcessRollOver` |
//!
//! Deviations: pseudo-B-rep objects use typed arenas and geometry handles have an
//! explicit resolver. Completed mutations survive failures. Only unavailable
//! callees are stubbed; the entire caller continuation is ported, including both
//! full-rollover calls, the segment walk and topology/surface/corner updates.
//! Internal short intervals are retained in descending order for refinement.
//! See `p5/recovery.rs` for the bounded segment-walk deviation. No recursion.

use super::p1::{FilletGeomRef, FilletSolver};
use crate::blend::fillet_geom::p1::{
    FilletBrep, FilletCornerId, FilletCornerType, FilletGeomId, FilletGeomType,
};
use crate::curve::curve::p2::OwnerChange;
use crate::error::{ignored, EntityRef, Result, SolidError};
use crate::geom::object::ObjectRef;
use crate::intersect::curve_class::p1::CurveClassification;
use crate::intersect::point_class::PointClassType;
use crate::kernel::Kernel;
use crate::query::face::p2::{curve_on_classify, CurveOnClassifyObjects};
use crate::surface::surf_of_revolution::SurfOfRevolution;
use crate::topology::{edge, edgeuse::p1 as eu, edgeuse::p2, Brep, EdgeId, EdgeuseId};

#[path = "p5/recovery.rs"]
mod recovery;

const OP: &str = "blend::fillet_solver::p5";

/// Resolve handles used by `TestRollOver`, without assuming the two B-reps' arena
/// indices identify the same objects. Curves returned here must be resolvable by
/// the inherited classification geometry resolver.
pub trait RolloverObjects: CurveOnClassifyObjects<EntityRef> {
    /// Identity of a rail's 3D curve in the pseudo-B-rep.
    fn rail_curve(&self, rail: EdgeId) -> EntityRef;
    /// Identity of the original face's UV curve on the rail edgeuse's mate.
    fn rail_mate_uv_curve(&self, mate: EdgeuseId) -> Option<EntityRef>;
    /// The actual virtual `GetCornerType` result for the specified corner.
    ///
    /// # Errors
    /// A resolver must report an unresolved corner, rather than invent its type.
    fn corner_type(&self, corner: FilletCornerId) -> Result<FilletCornerType>;
}

/// Caller-owned object identities and notification routing for new rail curves.
/// Arena indices are never assumed to be object-store keys.
pub trait RolloverOwnership {
    /// The new owner's actual object identity and its optional B-rep identity.
    fn edge_owner(&self, edge: EdgeId) -> Result<(ObjectRef, Option<ObjectRef>)>;
    /// Deliver the `SM_NO_CHANGE_OWNER` notification produced by `Curve::set_owner`.
    fn owner_changed(&mut self, change: OwnerChange) -> Result<()>;
}

impl FilletSolver {
    /// Classify both rails against their original faces, squeeze tiny end
    /// intervals, refine their classifications and process rollover. Iterates a
    /// snapshot of the solver's geometry list, as upstream does. `geometry_id`
    /// resolves each solver-local handle into the caller's pseudo-B-rep arena.
    ///
    /// # Errors
    /// Missing rails/curves/mates are invalid inputs (upstream dereferences them).
    /// Classification and unported recovery errors propagate; completed earlier
    /// changes and the caller's adjusted-corner list are retained.
    pub fn test_roll_over(
        &mut self,
        k: &Kernel,
        target: &Brep,
        fillets: &mut FilletBrep,
        objects: &impl RolloverObjects,
        geometry_id: &impl Fn(FilletGeomRef) -> Result<FilletGeomId>,
        adjusted_corners: &mut Vec<FilletCornerId>,
        ownership: &mut impl RolloverOwnership,
    ) -> Result<()> {
        let geoms = self.fillet_geoms();
        for reference in geoms {
            let id = geometry_id(reference)?;
            for ii in 0..2 {
                let rail = fillets
                    .geom(id)?
                    .rail(ii)
                    .ok_or_else(|| SolidError::invalid_input(OP, "TestRollOver: missing rail"))?;
                let curve = edge::curve(fillets.brep(), rail)?.ok_or_else(|| {
                    SolidError::invalid_input(OP, "TestRollOver: missing rail curve")
                })?;
                let interval = curve.natural_interval();
                let primary = eu::primary_edgeuse(fillets.brep(), rail)?.ok_or_else(|| {
                    SolidError::invalid_input(OP, "TestRollOver: missing primary edgeuse")
                })?;
                let mate = eu::mate(fillets.brep(), primary)?
                    .ok_or_else(|| SolidError::invalid_input(OP, "TestRollOver: missing mate"))?;
                let mut cc = CurveClassification::with_curve_interval(
                    k,
                    objects.rail_curve(rail),
                    interval,
                    self.approx_tol_3d,
                );
                cc.set_uv_curve1(objects.rail_mate_uv_curve(mate));
                fillets.fillet_edge_mut(rail)?.set_curve_class(Some(cc));
                let face = fillets.fillet_edge(rail)?.original_face().ok_or_else(|| {
                    SolidError::invalid_input(OP, "TestRollOver: missing original face")
                })?;
                // Install the classification before calling the fallible operation:
                // upstream leaves that object attached even when classification fails.
                let cc = fillets
                    .fillet_edge_mut(rail)?
                    .curve_class
                    .as_mut()
                    .ok_or_else(|| SolidError::null_pointer(OP))?;
                curve_on_classify(k, target, face, objects, true, cc, true, false, true)?;
                finish_classification(
                    self,
                    k,
                    target,
                    fillets,
                    id,
                    ii,
                    rail,
                    &mut MissingClassification,
                )?;
            }
            self.process_roll_over(
                k,
                target,
                fillets,
                id,
                &|corner| objects.corner_type(corner),
                adjusted_corners,
                ownership,
            )?;
        }
        Ok(())
    }

    /// Process an existing pair of rail classifications. Blend patches and
    /// single-interval rails that both remain classified need no repair.
    /// `corner_type` resolves the corner on rail 0's start vertex only when full
    /// rollover is needed; `adjusted_corners` is append-only upstream output.
    ///
    /// # Errors
    /// A missing classification or corner returns `SM_ERR_NULL_POINTER`. Unsupported
    /// full-rollover corner types return `SM_ERR`. Recovery through unavailable
    /// units returns `Unported`; invalid handles are reported by the arena.
    pub fn process_roll_over(
        &mut self,
        k: &Kernel,
        target: &Brep,
        fillets: &mut FilletBrep,
        id: FilletGeomId,
        corner_type: &impl Fn(FilletCornerId) -> Result<FilletCornerType>,
        adjusted_corners: &mut Vec<FilletCornerId>,
        ownership: &mut impl RolloverOwnership,
    ) -> Result<()> {
        recovery::Recovery {
            solver: self,
            k,
            target,
            fillets,
            ownership,
        }
        .process(id, corner_type, adjusted_corners, &mut recovery::Missing)
    }
}

trait ClassificationOps {
    fn connect(
        &mut self,
        _k: &Kernel,
        _fillets: &mut FilletBrep,
        _rail: EdgeId,
        _interval: usize,
        _at_end: bool,
        _allow_removal_of_ends: bool,
    ) -> Result<()> {
        // UNPORTED(intersect.curve_class_p2): SmCurveClassification::ConnectIntervals
        Err(SolidError::unported(
            "SmCurveClassification::ConnectIntervals",
            "intersect.curve_class_p2",
        ))
    }
    fn refine(
        &mut self,
        _solver: &mut FilletSolver,
        _k: &Kernel,
        _target: &Brep,
        _fillets: &mut FilletBrep,
        _geom: FilletGeomId,
        _which: usize,
        _rail: EdgeId,
        _small: &[usize],
    ) -> Result<()> {
        // UNPORTED(blend.fillet_solver_p3): sm_RefineRailCurveClassification
        Err(SolidError::unported(
            "sm_RefineRailCurveClassification",
            "blend.fillet_solver_p3",
        ))
    }
}
struct MissingClassification;
impl ClassificationOps for MissingClassification {}

fn finish_classification(
    solver: &mut FilletSolver,
    k: &Kernel,
    target: &Brep,
    fillets: &mut FilletBrep,
    geom: FilletGeomId,
    which: usize,
    rail: EdgeId,
    ops: &mut impl ClassificationOps,
) -> Result<()> {
    let small = squeeze_rail(k, fillets, rail, solver.approx_tol_3d, ops)?;
    // Refinement is warning-only upstream, but retain its completed mutations
    // and propagate an unavailable implementation rather than faking success.
    ignored(ops.refine(solver, k, target, fillets, geom, which, rail, &small))?;
    Ok(())
}

fn squeeze_rail(
    k: &Kernel,
    fillets: &mut FilletBrep,
    rail: EdgeId,
    tolerance: f64,
    ops: &mut impl ClassificationOps,
) -> Result<Vec<usize>> {
    let count = fillets
        .fillet_edge(rail)?
        .curve_class()
        .ok_or_else(|| SolidError::null_pointer(OP))?
        .size();
    let mut small = Vec::new();
    if count > 1 {
        for index in (0..count).rev() {
            let interval = fillets
                .fillet_edge(rail)?
                .curve_class()
                .ok_or_else(|| SolidError::null_pointer(OP))?
                .sub_interval(index)?;
            let curve =
                edge::curve(fillets.brep(), rail)?.ok_or_else(|| SolidError::null_pointer(OP))?;
            let start = curve.evaluate_point(interval.min())?;
            let end = curve.evaluate_point(interval.max())?;
            if start.distance(end) > 50.0 * tolerance {
                continue;
            }
            if index == 0 {
                ops.connect(k, fillets, rail, 0, false, true)?;
            } else if index == count - 1 {
                ops.connect(k, fillets, rail, index, true, true)?;
            } else {
                small.push(index);
            }
        }
    }
    Ok(small)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::fillet_geom::p1::{FilletEdge, FilletGeom, FilletVertex};
    use crate::intersect::point_class::PointClassType;
    use crate::math::Vec3;

    struct NoOwnership;
    impl RolloverOwnership for NoOwnership {
        fn edge_owner(&self, _: EdgeId) -> Result<(ObjectRef, Option<ObjectRef>)> {
            Err(SolidError::invalid_input(OP, "unexpected owner lookup"))
        }
        fn owner_changed(&mut self, _: OwnerChange) -> Result<()> {
            Err(SolidError::invalid_input(
                OP,
                "unexpected owner notification",
            ))
        }
    }

    fn classified_rails(k: &Kernel, fb: &mut FilletBrep) -> FilletGeomId {
        let mut geom = FilletGeom::new(k, None, FilletGeomType::Default).unwrap();
        for i in 0..2 {
            let a = fb.brep_mut().add_vertex(Vec3::ZERO).unwrap();
            let b = fb.brep_mut().add_vertex(Vec3::X).unwrap();
            let edge = fb.brep_mut().add_edge(a, b).unwrap();
            let vu = fb
                .brep_mut()
                .add_vertexuse_record(crate::topology::Vertexuse {
                    vertex: Some(a),
                    ..crate::topology::Vertexuse::default()
                })
                .unwrap();
            let eu = fb
                .brep_mut()
                .add_edgeuse_record(crate::topology::Edgeuse {
                    edge: Some(edge),
                    vertexuse: Some(vu),
                    ..crate::topology::Edgeuse::default()
                })
                .unwrap();
            fb.brep_mut().edge_mut(edge).unwrap().edgeuses.push(eu);

            fb.insert_fillet_vertex(a, FilletVertex::new(k, Some(FilletCornerId(42 + i as u32))));
            let mut record = FilletEdge::with_corner(None);
            let mut cc = CurveClassification::new(k);
            cc.set_size(k, 1).unwrap();
            cc.at_mut(0)
                .unwrap()
                .mid
                .set_class_object(PointClassType::Edge, Some(EntityRef::Edge(edge)));
            record.set_curve_class(Some(cc));
            fb.insert_fillet_edge(edge, record);
            geom.set_rail(i, Some(edge));
        }
        fb.add_geom(geom)
    }

    #[test]
    fn resolver_is_lazy_and_failure_preserves_caller_output() {
        let k = Kernel::default();
        let mut fb = FilletBrep::new();
        let id = classified_rails(&k, &mut fb);
        let mut solver = FilletSolver::new(1e-5, 1e-3, 1e-3);
        let target = Brep::new();
        let mut adjusted = vec![FilletCornerId(9), FilletCornerId(9)];
        solver
            .process_roll_over(
                &k,
                &target,
                &mut fb,
                id,
                &|_| panic!("known rails must not resolve a corner"),
                &mut adjusted,
                &mut NoOwnership,
            )
            .unwrap();
        let rail = fb.geom(id).unwrap().rail(1).unwrap();
        fb.fillet_edge_mut(rail)
            .unwrap()
            .curve_class
            .as_mut()
            .unwrap()
            .at_mut(0)
            .unwrap()
            .mid
            .set_class_object(PointClassType::Unknown, None);
        let refusal = SolidError::invalid_input(OP, "unresolved corner");
        let result = solver.process_roll_over(
            &k,
            &target,
            &mut fb,
            id,
            &|corner| {
                assert_eq!(corner, FilletCornerId(42));
                Err(refusal.clone())
            },
            &mut adjusted,
            &mut NoOwnership,
        );
        assert_eq!(result, Err(refusal));
        assert_eq!(adjusted, [FilletCornerId(9), FilletCornerId(9)]);
        let result = solver.process_roll_over(
            &k,
            &target,
            &mut fb,
            id,
            &|_| Ok(FilletCornerType::OneByOne),
            &mut adjusted,
            &mut NoOwnership,
        );
        assert!(matches!(
            result,
            Err(SolidError::Unported {
                unit: "blend.fillet_solver_p3",
                ..
            })
        ));
    }
}
