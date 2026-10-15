// SPDX-License-Identifier: Apache-2.0
// query.brep: call the actual SM_API surface entry points, preserving output
// sentinels on failure. Input readers reused from surface_surface_cache_p3.cpp.
#include "sc_shim.h"
#include "StdAfx.h"
#include <SmApiGeneral.h>
#include <SmApiSurfaces.h>
#include <SmBSplineSurface.h>
#include <SmContext.h>
#include <SmCurveBoundedSurface.h>
#include <SmNurbsSrf.h>
#include <SmPlane.h>
#include <SmSurface.h>
#include <SmTArray.h>
#include <nurbs.h>
#include <memory>
#include <vector>
namespace sc_query_brep {
struct Rd
{
    const double* in;
    int64_t at;
    double num() { return in[at++]; }
    int64_t idx() { return (int64_t)in[at++]; }
    bool flag() { return in[at++] != 0.0; }
    SmVector3d vec()
    {
        double x = num();
        double y = num();
        double z = num();
        return SmVector3d(x, y, z);
    }
    SmExtent2d domain()
    {
        double a = num();
        double b = num();
        double c = num();
        double d = num();
        return SmExtent2d(a, b, c, d);
    }
    SmExtent3d box()
    {
        SmVector3d lo = vec();
        SmVector3d hi = vec();
        SmExtent3d e;
        e.SetMinMax(lo, hi);
        return e;
    }
};

gw_SURFACE* read_nurb(Rd& rd)
{
    gw_INDEX n = (gw_INDEX)rd.idx();
    gw_INDEX m = (gw_INDEX)rd.idx();
    gw_DEGREE p = (gw_DEGREE)rd.idx();
    gw_DEGREE q = (gw_DEGREE)rd.idx();
    gw_INDEX r = (gw_INDEX)rd.idx();
    gw_INDEX s = (gw_INDEX)rd.idx();
    gw_SURFACE* sur = sm_AllocateNurbSurface(n, m, p, q, r, s);
    for (gw_INDEX i = 0; i <= n; i++)
        for (gw_INDEX j = 0; j <= m; j++)
        {
            sur->net->Pw[i][j].x = rd.num();
            sur->net->Pw[i][j].y = rd.num();
            sur->net->Pw[i][j].z = rd.num();
            sur->net->Pw[i][j].w = rd.num();
        }
    for (gw_INDEX i = 0; i <= r; i++)
        sur->knu->U[i] = rd.num();
    for (gw_INDEX i = 0; i <= s; i++)
        sur->knv->U[i] = rd.num();
    return sur;
}

// `oob form nurb`.
SmBSplineSurface* read_bspline(Rd& rd, const SmContext& ctx)
{
    bool oob = rd.flag();
    SmBSplineSurfaceForm form = (SmBSplineSurfaceForm)rd.idx();
    SmBSplineSurface* s = new (ctx) SmBSplineSurface(read_nurb(rd), FALSE, NULL);
    s->GetOutOfBoundsEnabled() = oob ? TRUE : FALSE;
    s->SetBSplineSurfaceForm(form);
    return s;
}

// A surface `kind ...`: 0 B-spline `oob form nurb`; 1 bounded plane `o x y sx sy D`;
// 2 curve-bounded surface (implicit outer, no boundary curves) over a B-spline block.
SmSurface* read_surface(Rd& rd, const SmContext& ctx)
{
    int64_t kind = rd.idx();
    if (kind == 1)
    {
        SmPoint3d o = rd.vec();
        SmVector3d x = rd.vec();
        SmVector3d y = rd.vec();
        double sx = rd.num();
        double sy = rd.num();
        SmExtent2d d = rd.domain();
        return new (ctx) SmPlane(o, x, y, SmVector2d(sx, sy), d, &ctx);
    }
    SmBSplineSurface* b = read_bspline(rd, ctx);
    if (kind == 0)
        return b;
    // the curve-bounded surface owns (deletes) its basis
    SmTArray<SmCurve*> c3d(ctx), cuv(ctx);
    return new (ctx) SmCurveBoundedSurface(*b, &c3d, &cuv, TRUE);
}

}
SC_EXPORT int32_t sc_query_brep_surface(int32_t op, const double* input, int64_t n_in,
    double* out, int64_t cap, int64_t* out_len) {
    if (op < 0 || op > 2 || !input || n_in < 12) return SC_ERR_ARG;
    SC_TRY
    sc_query_brep::Rd rd{input, 0};
    const SmContext& ctx = *SmApiGetOrCreateContext();
    bool present = rd.flag();
    std::unique_ptr<SmSurface> s(present ? sc_query_brep::read_surface(rd, ctx) : nullptr);
    double u = rd.num(), v = rd.num();
    SmVector2d uv(u, v);
    SmVector3d p = rd.vec(), du = rd.vec(), dv = rd.vec();
    if (rd.at != n_in) return SC_ERR_ARG;
    SmStatus status;
    if (op == 0) status = SmApiEvaluateSurfacePoint(s.get(), uv, p);
    else if (op == 1) status = SmApiEvaluateSurfaceNormal(s.get(), uv, p);
    else status = SmApiEvaluateSurfaceDerivatives(s.get(), uv, p, du, dv);
    std::vector<double> result{status == SM_SUCCESS ? 0.0 : (double)status,
        p.x,p.y,p.z,du.x,du.y,du.z,dv.x,dv.y,dv.z};
    return sc_copy_out(result.data(), (int64_t)result.size(), out, cap, out_len);
    SC_CATCH
}
