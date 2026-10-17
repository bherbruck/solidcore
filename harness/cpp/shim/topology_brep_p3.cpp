// SPDX-License-Identifier: Apache-2.0
// difftest shim for unit `topology.brep_p3`: the SmBrep members of the unit
// (source/SMLib/src/SmBrep.cpp) and SmApiBrepCopy (source/SM_API/src/SmApiQueries.cpp)
// on real SmBrep objects built by the shared topology fixture (sc_topology_fixture.h),
// built and normalised exactly as the topology.brep_p2 shim does, so that its Export
// and Extras ops describe the B-rep this shim runs on.
//
// One entry point, sc_topology_brep_p3_eval(op, path, in, n_in, out, cap, out_len).
// `in` is the fixture's B-rep block, then `editing` (-1 keep, 0/1 set
// m_bEditingEnabled), then the op's arguments; layouts are documented in
// crates/solidcore-ffi/src/units/topology_brep_p3.rs, which validates every input.
// Outputs start with `0` (built) or `-1 status` (the build failed). Entities are named
// by their index in the fixture's walk of the B-rep before the op (the B-rep's face,
// edge and vertex lists registered first). After an op that may change the topology,
// only pointers captured before it that the op cannot delete are read (faces and
// edges for Orient / TurnToNURBS, which delete none); the counts come from a fresh
// walk.
#include "sc_topology_fixture.h"

#include <SmApiQueries.h>

namespace
{
enum P3Op : int32_t
{
    P3_CONSTRUCT = 2,
    P3_COPY = 3,
    P3_MERGE = 4,
    P3_COPY_VERTICES = 5,
    P3_DELETE_FACE = 6,
    P3_WIRE = 7,
    P3_ORIENT = 8,
    P3_SQUEEZE = 9,
    P3_TURN_NURBS = 10,
    P3_API_COPY_NULL = 11,
};

struct EdgePeek : SmEdge
{
    static SmCurve* SmEdge::*curve() { return &EdgePeek::m_pCurve; }
};

struct BrepPeek : SmBrep
{
    static double SmBrep::*mse() { return &BrepPeek::m_dThisModelSizeEstimate; }
    static double SmBrep::*lsr() { return &BrepPeek::m_dThisLargeSmallSizeRatio; }
};

struct RegPeek : SmRegion
{
    static SmBoolean SmRegion::*void_flag() { return &RegPeek::m_bIsVoidFlag; }
};

// A plain SmBSplineCurve of `c`'s NURB, or nullptr when `c` already is one (or a line)
// or has no NURB (as the topology.brep_p2 shim).
SmBSplineCurve* plain_curve(SmCurve* c)
{
    if (c == nullptr || !c->IsKindOf(SmBSplineCurve_TYPE) || c->GetType() == SmBSplineCurve_TYPE ||
        c->GetType() == SmLine_TYPE)
        return nullptr;
    SmBSplineCurve* b = (SmBSplineCurve*)c;
    gw_CURVE* nurb = b->GetGwNurbPointer();
    if (nurb == nullptr)
        return nullptr;
    return new (the_context()) SmBSplineCurve(sm_AllocateAndCopyNurbCurve(nurb), b->GetDim(), FALSE);
}

// Gives every analytic face and edge a plain B-spline copy owned by it (the
// topology.brep_p2 shim's normalisation, so that the Rust import of its export is exact).
void normalise(Graph& g)
{
    for (SmFace* f : g.faces)
    {
        SmSurface* s = f->*FPeek::srf();
        if (s == nullptr || !s->IsKindOf(SmBSplineSurface_TYPE) || s->GetType() == SmBSplineSurface_TYPE ||
            s->GetType() == SmPlane_TYPE)
            continue;
        SmBSplineSurface* b = (SmBSplineSurface*)s;
        gw_SURFACE* nurb = b->GetGwNurbPointer();
        if (nurb == nullptr)
            continue;
        SmBSplineSurface* nb = new (the_context()) SmBSplineSurface(sm_AllocateAndCopyNurbSurface(nurb), FALSE, NULL);
        nb->GetOutOfBoundsEnabled() = b->GetOutOfBoundsEnabled();
        nb->SetBSplineSurfaceForm(b->GetBSplineSurfaceForm());
        if (s->GetOwner() == (SmObject*)f)
            nb->SetOwner(f);
        f->*FPeek::srf() = nb;
    }
    for (SmEdge* e : g.edges)
    {
        SmCurve* c = e->*EdgePeek::curve();
        SmBSplineCurve* nc = plain_curve(c);
        if (nc == nullptr)
            continue;
        if (c->GetOwner() == (SmObject*)e)
            nc->SetOwner(e);
        e->*EdgePeek::curve() = nc;
    }
}

template <class T> T* at(std::vector<T*>& v, int64_t i)
{
    return (i >= 0 && i < (int64_t)v.size()) ? v[(size_t)i] : nullptr;
}

// The fixture B-rep as the brep_p2 shim builds it, registered in list order and walked.
SmBrep* prepare(Rd& rd, const char* path, Graph& g, SmStatus& bst)
{
    SmBrep* brep = build(rd, path, bst);
    if (brep == nullptr)
        return nullptr;
    int64_t editing = rd.idx();
    if (editing >= 0)
        brep->m_bEditingEnabled = editing ? TRUE : FALSE;
    SmTArray<SmFace*> fs;
    SmTArray<SmEdge*> es;
    SmTArray<SmVertex*> vs;
    brep->GetFaces(fs);
    brep->GetEdges(es);
    brep->GetVertices(vs);
    for (ULONG i = 0; i < fs.GetSize(); i++)
        g.face(fs[i]);
    for (ULONG i = 0; i < es.GetSize(); i++)
        g.edge(es[i]);
    for (ULONG i = 0; i < vs.GetSize(); i++)
        g.vertex(vs[i]);
    walk(brep, g, true);
    normalise(g);
    return brep;
}

// `faces edges vertices regions` of a fresh walk of `brep`.
void put_counts(std::vector<double>& o, SmBrep* brep)
{
    SmTArray<SmFace*> fs;
    SmTArray<SmEdge*> es;
    SmTArray<SmVertex*> vs;
    SmTArray<SmRegion*> rs;
    brep->GetFaces(fs);
    brep->GetEdges(es);
    brep->GetVertices(vs);
    brep->GetRegions(rs);
    o.push_back((double)fs.GetSize());
    o.push_back((double)es.GetSize());
    o.push_back((double)vs.GetSize());
    o.push_back((double)rs.GetSize());
}

void run(int32_t op, Rd& rd, const char* path, std::vector<double>& o)
{
    const SmContext& ctx = the_context();
    if (op == P3_CONSTRUCT)
    {
        // no fixture: a new B-rep with the given size estimate and ratio, in a context of
        // its own (the constructor passes the estimate and ratio to its context)
        double mse = rd.num(), lsr = rd.num();
        SmContext* own = new SmContext();
        SmBrep* b = new (*own) SmBrep(mse, lsr);
        o.push_back(0.0);
        o.push_back(0.0);
        o.push_back((double)b->GetTolerance());
        o.push_back(b->*BrepPeek::mse());
        o.push_back(b->*BrepPeek::lsr());
        o.push_back(flg(b->m_bEditingEnabled));
        SmRegion* inf = b->GetInfiniteRegion();
        o.push_back(inf ? flg(inf->*RegPeek::void_flag()) : -1.0);
        put_counts(o, b);
        delete b;
        delete own;
        return;
    }
    if (op == P3_API_COPY_NULL)
    {
        SmBrep* r = nullptr;
        o.push_back(0.0);
        o.push_back(st((SmStatus)SmApiBrepCopy(NULL, r)));
        delete r;
        return;
    }
    SmStatus bst;
    Built b;
    Graph g;
    b.brep = prepare(rd, path, g, bst);
    if (b.brep == nullptr)
    {
        o.push_back(-1.0);
        o.push_back(st(bst));
        return;
    }
    SmBrep* brep = b.brep;
    o.push_back(0.0);
    switch (op)
    {
    case P3_COPY:
    {
        SmBoolean analytics = rd.bflag();
        bool api = rd.flag();
        SmBrep* c = nullptr;
        SmStatus s = SM_SUCCESS;
        if (api)
            s = (SmStatus)SmApiBrepCopy(brep, c);
        else
            c = new (ctx) SmBrep(*brep, NULL, NULL, TRUE, analytics);
        o.push_back(st(s));
        o.push_back((double)c->GetTolerance());
        o.push_back(flg(c->m_bEditingEnabled));
        put_counts(o, c);
        delete c;
        break;
    }
    case P3_MERGE:
    {
        // merge the fixture B-rep into a second build of it
        SmBoolean analytics = rd.bflag();
        Rd rd2{rd.in, 0};
        SmStatus bst2;
        Built b2;
        Graph g2;
        b2.brep = prepare(rd2, path, g2, bst2);
        if (b2.brep == nullptr)
        {
            o.push_back(st(bst2));
            break;
        }
        o.push_back(st(b2.brep->MergeBrep(*brep, NULL, NULL, analytics)));
        put_counts(o, b2.brep);
        break;
    }
    case P3_COPY_VERTICES:
    {
        int64_t n = rd.idx();
        SmTArray<SmVertex*> vs;
        for (int64_t i = 0; i < n; i++)
            vs.Add(at(g.vertices, rd.idx()));
        Rd rd2{rd.in, 0};
        SmStatus bst2;
        Built b2;
        Graph g2;
        b2.brep = prepare(rd2, path, g2, bst2);
        if (b2.brep == nullptr)
        {
            o.push_back(st(bst2));
            break;
        }
        SmTArray<SmVertex*> nv;
        o.push_back(st(brep->CopyVertices(vs, b2.brep, &nv)));
        o.push_back((double)nv.GetSize());
        put_counts(o, b2.brep);
        break;
    }
    case P3_DELETE_FACE:
    {
        SmFace* f = at(g.faces, rd.idx());
        SmBoolean del = rd.bflag();
        SmBoolean nest = rd.bflag();
        SmRegion* keep = at(g.regions, rd.idx());
        SmTArray<SmTopology*> gone;
        o.push_back(st(brep->DeleteFace(f, del, nest, &gone, keep)));
        o.push_back((double)gone.GetSize());
        put_counts(o, brep);
        break;
    }
    case P3_WIRE:
    {
        int64_t which = rd.idx();
        SmRegion* r = at(g.regions, rd.idx());
        SmVertex* v1 = at(g.vertices, rd.idx());
        SmVertex* v2 = at(g.vertices, rd.idx());
        SmBSplineCurve* c = read_curve(rd, ctx);
        double t0 = rd.num(), t1 = rd.num();
        SmOrientType orient = (SmOrientType)rd.idx();
        SmVector3d end = rd.vec();
        SmEdge* e = nullptr;
        SmVertex* v = nullptr;
        SmStatus s;
        if (which == 0)
            s = brep->MakeWireEdge(r, v1, v2, c, SmExtent1d(t0, t1), orient, e);
        else
            s = brep->MakeWireEdgeVertex(r, v1, c, SmExtent1d(t0, t1), orient, end, e, v);
        // the curve belongs to the B-rep once an edge took it (also on a later failure)
        bool bound = false;
        {
            SmTArray<SmEdge*> es;
            brep->GetEdges(es);
            for (ULONG i = 0; i < es.GetSize(); i++)
                bound = bound || es[i]->GetCurve() == (SmCurve*)c;
        }
        if (!bound)
            delete c;
        o.push_back(st(s));
        put_counts(o, brep);
        break;
    }
    case P3_ORIENT:
    {
        SmBoolean outward = rd.bflag();
        SmBoolean unify = rd.bflag();
        int64_t n = rd.idx();
        bool has_list = n >= 0;
        SmTArray<SmFace*> swap;
        for (int64_t i = 0; i < n; i++)
            swap.Add(at(g.faces, rd.idx()));
        SmBoolean maybe = FALSE;
        SmStatus s = brep->OrientTrimmedSurfaces(outward, maybe, unify, has_list ? &swap : NULL);
        o.push_back(st(s));
        o.push_back(flg(maybe));
        // Orient deletes nothing: the faces registered before the op are live
        for (SmFace* f : g.faces)
        {
            SmExtent2d d = f->GetUVDomain();
            o.push_back(d.GetMin().x);
            o.push_back(d.GetMin().y);
            o.push_back(d.GetMax().x);
            o.push_back(d.GetMax().y);
            SmFaceuse* fu = f->GetUpwardFaceuse();
            o.push_back((double)g.find(2, fu));
        }
        break;
    }
    case P3_SQUEEZE:
    {
        SmEdge* e = at(g.edges, rd.idx());
        int64_t which = rd.idx();
        SmVertex* any = at(g.vertices, rd.idx());
        SmVertex* v = which == 0 ? e->GetVertex() : which == 1 ? e->GetOtherVertex(e->GetVertex()) : any;
        o.push_back(st(brep->SqueezeEdge(e, v)));
        put_counts(o, brep);
        break;
    }
    case P3_TURN_NURBS:
    {
        o.push_back(st(brep->TurnToNURBS()));
        // TurnToNURBS deletes no face or edge
        for (SmEdge* e : g.edges)
            put_curve(o, e->GetCurve());
        for (SmFace* f : g.faces)
        {
            SmSurface* s = f->GetSurface();
            o.push_back(s ? (double)s->GetType() : -1.0);
        }
        break;
    }
    default: o.push_back(-9.0); break;
    }
}
} // namespace

SC_EXPORT int32_t sc_topology_brep_p3_eval(int32_t op, const char* path, const double* in, int64_t n_in,
                                           double* out, int64_t cap, int64_t* out_len)
{
    if (in == nullptr || n_in < 1 || out_len == nullptr || path == nullptr)
        return SC_ERR_ARG;
    SC_TRY
    Rd rd{in, 0};
    std::vector<double> o;
    run(op, rd, path, o);
    return sc_copy_out(o.data(), (int64_t)o.size(), out, cap, out_len);
    SC_CATCH
}
