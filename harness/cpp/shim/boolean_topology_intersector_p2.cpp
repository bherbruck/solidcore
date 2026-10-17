// SPDX-License-Identifier: Apache-2.0
// difftest shim for unit `boolean.topology_intersector_p2`: the SmTopologyIntersector members
// owned by the unit (source/SMLib/src/SmTopologyIntersector.cpp,
// source/SMLib/inc/SmTopologyIntersector.h) on real SmBrep objects.
//
// Part 2 runs on the same intersector object and two-B-rep script as part 1, so this file
// includes part 1's shim (boolean_topology_intersector_p1.cpp) with SC_TI_EXTRA_OPS: every
// op of part 1 works unchanged and the script ops 41 to 51 below are part 2's. The entry
// point is sc_boolean_topology_intersector_p2_eval(op, path, in, n_in, out, cap, out_len);
// layouts are documented in crates/solidcore-ffi/src/units/boolean_topology_intersector_p2.rs,
// which validates every input (on top of part 1's validation).
#define SC_TI_EXTRA_OPS 1
#define SC_TI_EVAL_NAME sc_boolean_topology_intersector_p2_eval
#include "boolean_topology_intersector_p1.cpp"

namespace
{
enum TiScriptOp2 : int64_t
{
    S_REMOVE_REL = 41,
    S_SQUEEZE = 42,
    S_DELETE_FACES = 43,
    S_FIX_GAPS = 44,
    S_INTERSECT_TOPO = 45,
    S_SET_RELS = 46,
    S_IIR = 47,
    S_FIXUP_CLASS = 48,
    S_FIXUP_SPLIT = 49,
    S_EDGE_EDGE = 50,
    S_EDGE_FACE_COIN = 51,
};

bool is_vertex(const SmObject* p) { return p != nullptr && p->IsKindOf(SmVertex_TYPE); }

// A mate that upstream casts to SmVertex*: fine when null or a vertex.
bool vertex_mate_ok(const SmObject* p) { return p == nullptr || is_vertex(p); }

// A classification: `curve t0 t1 tol n (a b pc3*)*`, each pc `class h t has_t dev`.
// Returns false (after reading everything) when a vertex classification names no vertex.
bool read_cc(Rd& rd, const Two& t, const SmContext& ctx, SmCurveClassification*& cc)
{
    SmCurve* c = (SmCurve*)rd_h(rd, t);
    double t0 = rd.num(), t1 = rd.num(), tol = rd.num();
    int64_t n = rd.idx();
    cc = new (ctx) SmCurveClassification(c, SmExtent1d(t0, t1), nullptr, tol);
    cc->SetSize((ULONG)n);
    bool ok = c != nullptr;
    for (int64_t i = 0; i < n; i++)
    {
        double a = rd.num(), b = rd.num();
        (*cc)[(ULONG)i] = SmCurveInterval(SmExtent1d(a, b), &ctx, cc);
        SmPointClassification* pcs[3] = {&(*cc)[(ULONG)i].m_vStart, &(*cc)[(ULONG)i].m_vMid, &(*cc)[(ULONG)i].m_vEnd};
        for (SmPointClassification* pc : pcs)
        {
            int64_t cls = rd.idx();
            SmObject* obj = rd_h(rd, t);
            double tp = rd.num();
            bool has_t = rd.flag();
            double dev = rd.num();
            pc->SetClassObject((SmPointClassificationType)cls, obj);
            if (has_t)
                pc->SetTParam(tp);
            pc->SetDeviation(dev);
            if (cls == SM_PC_VERTEX && !is_vertex(obj))
                ok = false;
        }
    }
    return ok;
}

void put_cc(std::vector<double>& o, const Two& t, const SmCurveClassification& cc)
{
    o.push_back((double)cc.GetSize());
    for (ULONG i = 0; i < cc.GetSize(); i++)
    {
        o.push_back(cc[i].GetInterval().GetMin());
        o.push_back(cc[i].GetInterval().GetMax());
        put_pc_full(o, t, cc[i].m_vStart);
        put_pc_full(o, t, cc[i].m_vMid);
        put_pc_full(o, t, cc[i].m_vEnd);
    }
}

template <class T> void rd_list(Rd& rd, const Two& t, SmTArray<T*>& a)
{
    int64_t n = rd.idx();
    for (int64_t i = 0; i < n; i++)
        a.Add((T*)rd_h(rd, t));
}

// The edges of a vertex all have a curve and vertices (FixGaps, EdgeFaceCoincidence
// dereference them).
bool vertex_edges_ok(SmVertex* v)
{
    SmTArray<SmEdge*> es;
    v->GetEdges(es);
    for (ULONG i = 0; i < es.GetSize(); i++)
        if (!edge_ok(es[i]))
            return false;
    return true;
}

bool vertex_faces_ok(SmVertex* v)
{
    SmTArray<SmFace*> fs;
    v->GetFaces(fs);
    for (ULONG i = 0; i < fs.GetSize(); i++)
        if (!face_ok(fs[i]))
            return false;
    return true;
}
// FixGaps' scan up to its first squeeze candidate (read-only): whether CombineCoincidentVertices
// would run with a null B-rep or a mate that is not a vertex (the steps before it return early).
bool fix_gaps_first_candidate_ok(SmTopologyIntersector* ti, const SmTArray<SmVertex*>& verts)
{
    ULONG n = verts.GetSize();
    for (ULONG ii = 0; ii < n; ii++)
        for (ULONG jj = ii + 1; jj < n; jj++)
        {
            SmVertex *v1 = verts[ii], *v2 = verts[jj];
            double tol = 1.01 * (v1->GetTolerance() + v2->GetTolerance());
            if (!(v1->GetPoint().DistanceBetween(v2->GetPoint()) < tol))
                continue;
            SmTArray<SmEdge*> e1, e2;
            v1->GetEdges(e1);
            v2->GetEdges(e2);
            bool common = false;
            for (ULONG k = 0; k < e1.GetSize(); k++)
            {
                ULONG idx;
                if (e2.FindElement(e1[k], idx) && e1[k]->GetCurve()->ApproximateLength(e1[k]->GetInterval(), 10) > tol)
                    common = true;
            }
            if (common)
                continue;
            SmObject* o1 = ti->GetOtherMate(v1);
            SmObject* o2 = ti->GetOtherMate(v2);
            if (o1 == nullptr || o2 == nullptr)
                return true; // NER returns SM_ERR_NULL_POINTER first
            return is_vertex(o1) && is_vertex(o2) && ti->GetPrimaryBrep() != nullptr && ti->GetOtherBrep() != nullptr;
        }
    return true;
}

// CheckForSqueezeVertices: whether a combination would run with a null B-rep (each block
// combines in its B-rep unless the previous relationship is better, which returns SM_ERR).
bool squeeze_breps_ok(SmTopologyIntersector* ti, SmVertex* bv, SmVertex* ov)
{
    bool need_other = false, need_brep = false;
    SmVertex* other_v = (SmVertex*)ti->GetOtherMate(bv);
    SmVertex* brep_v = (SmVertex*)ti->GetBrepMate(ov);
    if (other_v && other_v != ov)
    {
        double d_old = bv->GetPoint().DistanceBetween(other_v->GetPoint());
        double d_new = bv->GetPoint().DistanceBetween(ov->GetPoint());
        if (d_new > d_old + SM_EFF_ZERO_SQRT)
            return true;
        need_other = true;
    }
    if (brep_v && brep_v != bv)
    {
        double d_old = ov->GetPoint().DistanceBetween(brep_v->GetPoint());
        double d_new = ov->GetPoint().DistanceBetween(bv->GetPoint());
        if (!(d_new > d_old + SM_EFF_ZERO_SQRT))
            need_brep = true;
    }
    return (!need_other || ti->GetOtherBrep() != nullptr) && (!need_brep || ti->GetPrimaryBrep() != nullptr);
}
} // namespace

namespace
{
bool run_extra_op(int64_t op, Rd& rd, Two& t, SmTopologyIntersector* ti, SmProtoTopologyManager*& mgr,
                  int64_t ctor_kind, std::vector<double>& o)
{
    const SmContext& ctx = the_context();
    switch (op)
    {
    case S_REMOVE_REL:
    {
        SmTopology* a = (SmTopology*)rd_h(rd, t);
        SmTopology* b = (SmTopology*)rd_h(rd, t);
        // (IsMatedPair compares pointers only; without maps nothing is mated)
        o.push_back(st(ti->RemoveRelationship(a, b)));
        return true;
    }
    case S_SQUEEZE:
    {
        SmVertex* bv = (SmVertex*)rd_h(rd, t);
        SmVertex* ov = (SmVertex*)rd_h(rd, t);
        if (!is_vertex(bv) || !is_vertex(ov) || !vertex_mate_ok(ti->GetOtherMate(bv)) ||
            !vertex_mate_ok(ti->GetBrepMate(ov)) || !squeeze_breps_ok(ti, bv, ov))
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        SmTArray<SmVertex*> dv, sv;
        SmTArray<SmEdge*> de;
        SmStatus s = ti->CheckForSqueezeVertices(bv, ov, dv, sv, de);
        o.push_back(st(s));
        if (s != SM_SUCCESS)
            return true;
        o.push_back((double)dv.GetSize());
        o.push_back((double)sv.GetSize());
        o.push_back((double)de.GetSize());
        return true;
    }
    case S_DELETE_FACES:
    {
        SmTArray<SmFace*> faces;
        rd_list(rd, t, faces);
        bool has_keep = rd.flag();
        SmTArray<SmEdge*> keep;
        rd_list(rd, t, keep);
        bool ok = true;
        for (ULONG i = 0; i < faces.GetSize(); i++)
            if (faces[i] == nullptr)
                ok = false;
        if (!ok)
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        o.push_back(st(ti->DeleteFaces(faces, has_keep ? &keep : nullptr)));
        return true;
    }
    case S_FIX_GAPS:
    {
        SmTArray<SmVertex*> verts;
        rd_list(rd, t, verts);
        bool ok = true;
        for (ULONG i = 0; i < verts.GetSize(); i++)
            if (!is_vertex(verts[i]) || !vertex_edges_ok(verts[i]))
                ok = false;
        if (ok && !fix_gaps_first_candidate_ok(ti, verts))
            ok = false;
        if (!ok)
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        SmBoolean modified = FALSE;
        SmTArray<SmVertex*> dv;
        SmTArray<SmEdge*> de;
        SmStatus s = ti->FixGaps(verts, modified, dv, de);
        o.push_back(st(s));
        if (s != SM_SUCCESS)
            return true;
        o.push_back(flg(modified));
        o.push_back((double)dv.GetSize());
        o.push_back((double)de.GetSize());
        return true;
    }
    case S_INTERSECT_TOPO:
    case S_SET_RELS:
    {
        // (IntersectTopology reads both B-reps' faces; SetRelationships' Relate needs the maps)
        if (mgr == nullptr || (op == S_INTERSECT_TOPO && (ti->GetPrimaryBrep() == nullptr || ti->GetOtherBrep() == nullptr)) ||
            (op == S_SET_RELS && ctor_kind != 0))
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        SmStatus s = (op == S_INTERSECT_TOPO) ? ti->IntersectTopology() : ti->SetRelationships();
        o.push_back(st(s));
        o.push_back((double)mgr->GetProtoVertices().GetSize());
        o.push_back((double)mgr->GetProtoEdges().GetSize());
        o.push_back((double)mgr->GetProtoFaces().GetSize());
        return true;
    }
    case S_IIR:
    {
        if (ctor_kind != 0 || ti->GetPrimaryBrep() == nullptr || ti->GetOtherBrep() == nullptr)
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        // (the manager IntersectInsertRelate makes replaces the installed one, which stays
        // owned by the script; it is the script's last op)
        o.push_back(st(ti->IntersectInsertRelate()));
        return true;
    }
    case S_FIXUP_CLASS:
    case S_FIXUP_SPLIT:
    {
        int64_t index = (op == S_FIXUP_SPLIT) ? rd.idx() : 0;
        SmCurveClassification *cb = nullptr, *co = nullptr;
        bool ok_b = read_cc(rd, t, ctx, cb);
        bool ok_o = read_cc(rd, t, ctx, co);
        struct Del
        {
            SmCurveClassification*& a;
            SmCurveClassification*& b;
            ~Del()
            {
                delete a;
                delete b;
            }
        } del{cb, co};
        if (!ok_b || !ok_o || (op == S_FIXUP_SPLIT && ((ULONG)index >= cb->GetSize() || (ULONG)index >= co->GetSize())))
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        SmBoolean changed = FALSE;
        SmStatus s = (op == S_FIXUP_CLASS) ? ti->FixupClassMates(*cb, *co)
                                           : ti->FixupSplitVertexPair((ULONG)index, *cb, *co, changed);
        o.push_back(st(s));
        if (s != SM_SUCCESS)
            return true;
        if (op == S_FIXUP_SPLIT)
            o.push_back(flg(changed));
        put_cc(o, t, *cb);
        put_cc(o, t, *co);
        return true;
    }
    case S_EDGE_EDGE:
    {
        SmTArray<SmEdge*> e1, e2;
        rd_list(rd, t, e1);
        rd_list(rd, t, e2);
        SmBoolean manifold = rd.bflag();
        SmBoolean ignore = rd.bflag();
        bool ok = true;
        for (ULONG i = 0; i < e1.GetSize(); i++)
            if (e1[i] != nullptr && !edge_ok(e1[i]))
                ok = false;
        for (ULONG i = 0; i < e2.GetSize(); i++)
            if (e2[i] != nullptr && !edge_ok(e2[i]))
                ok = false;
        if (!ok)
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        SmTArray<SmEdge*> s1, n1, s2, n2;
        SmTArray<SmVertex*> nv;
        double mx = 0.0, mn = 0.0;
        SmStatus s = ti->EdgeEdgeIntersect(e1, e2, s1, n1, s2, n2, nv, mx, mn, manifold, ignore);
        o.push_back(st(s));
        if (s != SM_SUCCESS)
            return true;
        put_list(o, t, as_objs(s1));
        o.push_back((double)n1.GetSize());
        put_list(o, t, as_objs(s2));
        o.push_back((double)n2.GetSize());
        o.push_back((double)nv.GetSize());
        o.push_back(mx);
        o.push_back(mn);
        return true;
    }
    case S_EDGE_FACE_COIN:
    {
        SmTArray<SmVertex*> verts;
        SmTArray<SmEdge*> excl;
        rd_list(rd, t, verts);
        rd_list(rd, t, excl);
        SmBoolean ignore = rd.bflag();
        SmBoolean nesting = rd.bflag();
        bool has_null = false;
        for (ULONG i = 0; i < verts.GetSize(); i++)
            if (verts[i] == nullptr)
                has_null = true;
        bool ok = true;
        for (ULONG i = 0; i < verts.GetSize(); i++)
        {
            if (verts[i] == nullptr)
                continue;
            if (!is_vertex(verts[i]) || !vertex_edges_ok(verts[i]) || !vertex_faces_ok(verts[i]))
            {
                ok = false;
                continue;
            }
            // a null other vertex matches a null entry of the list and is then dereferenced
            SmTArray<SmEdge*> es;
            verts[i]->GetEdges(es);
            for (ULONG j = 0; j < es.GetSize(); j++)
                if (has_null && es[j]->GetOtherVertex(verts[i]) == nullptr)
                    ok = false;
        }
        if (!ok)
        {
            o.push_back(ST_NULL_DEREF);
            return true;
        }
        SmTArray<SmEdge*> ie;
        SmTArray<SmFace*> pf, nf, sf;
        double mx = 0.0, mn = 0.0;
        SmStatus s = ti->EdgeFaceCoincidence(verts, excl, ie, pf, nf, sf, mx, mn, ignore, nesting);
        o.push_back(st(s));
        if (s != SM_SUCCESS)
            return true;
        put_list(o, t, as_objs(ie));
        put_list(o, t, as_objs(pf));
        o.push_back((double)nf.GetSize());
        put_list(o, t, as_objs(sf));
        o.push_back(mx);
        o.push_back(mn);
        return true;
    }
    default: return false;
    }
}
} // namespace
