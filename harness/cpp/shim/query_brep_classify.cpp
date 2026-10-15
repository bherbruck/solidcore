// SPDX-License-Identifier: Apache-2.0
// Actual point classification on a newly constructed, geometrically empty B-rep.
// The constructor supplies its real infinite region; no region stand-in is used.
#include "sc_shim.h"
#include "StdAfx.h"
#include <SmApiGeneral.h>
#include <SmBrep.h>
#include <SmPointClass.h>
#include <SmRegion.h>
#include <SmTArray.h>
#include <memory>
#include <vector>
// Expose only the real SmRegion constructor/destructor for fixture creation;
// every queried operation and field remains the upstream implementation.
namespace {
struct FixtureRegion : SmRegion {
    FixtureRegion() : SmRegion() {}
    ~FixtureRegion() override = default;
};
}
SC_EXPORT int32_t sc_query_brep_classify(const double* point, double tolerance, int32_t boundary, int32_t region_count,
    double* out, int64_t cap, int64_t* out_len) {
    if(!point || boundary<0 || boundary>1 || region_count<1 || region_count>4) return SC_ERR_ARG;
    SC_TRY
    const SmContext& ctx=*SmApiGetOrCreateContext();
    std::unique_ptr<SmBrep> brep(new (ctx) SmBrep());
    for(int i=1;i<region_count;i++) {
        std::unique_ptr<FixtureRegion> region(new (brep.get()) FixtureRegion());
        SmStatus insertion=brep->PostInsert(region.get());
        if(insertion!=SM_SUCCESS) return (int32_t)insertion;
        region.release();
    }
    SmTArray<SmRegion*> regions; brep->GetRegions(regions);
    if(regions.GetSize()!=(ULONG)region_count) return SC_ERR_ARG;
    SmPointClassification pc(SmZoneTol3d(17.0),&ctx);
    pc.SetGap3d(31.0); pc.SetTParam(47.0,FALSE); pc.SetUVParam(SmPoint2d(53.0,59.0));
    pc.SetPreSnapParam(61.0); pc.SetSSSPoint(SmPoint3d(67.0,71.0,73.0));
    SmStatus status=brep->Point3DClassify(SmPoint3d(point[0],point[1],point[2]),SmZoneTol3d(tolerance),boundary?TRUE:FALSE,pc);
    SmPoint2d uv=pc.GetUVParam();SmPoint3d sss=pc.GetSSSPoint();
    std::vector<double> result{status==SM_SUCCESS?0.0:(double)status,(double)pc.GetPointClass(),
        pc.GetObject()==regions[0]?0.0:-1.0,pc.GetDeviation(),pc.GetTParam(),uv.x,uv.y,
        pc.AreParametersSet()?1.0:0.0,pc.GetPreSnapParam(),pc.GetSSSPointFlag()?1.0:0.0,
        sss.x,sss.y,sss.z,(double)pc.GetTolerance()};
    return sc_copy_out(result.data(),(int64_t)result.size(),out,cap,out_len);
    SC_CATCH
}
