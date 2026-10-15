// SPDX-License-Identifier: Apache-2.0
// query.face_p1: calls the actual SmApiEvaluateCurve entry point.
// Upstream: source/SM_API/src/SmApiCurves.cpp:1043-1080.
#include "sc_shim.h"
#include "StdAfx.h"
#include <SmApiCurves.h>
#include <SmContext.h>
#include <SmCurve.h>
#include <SmLine.h>
#include <SmHermiteCurve.h>
#include <memory>

SC_EXPORT int32_t sc_query_face_p1_curve(int32_t kind, const double* data,
                                        double parameter, int32_t mask,
                                        double* out, int32_t* operation_status)
{
    if (!data || !out || !operation_status || kind < 0 || kind > 2 || mask < 0 || mask > 7)
        return SC_ERR_ARG;
    SC_TRY
    SmContext context;
    auto vec = [&](int i) { return SmVector3d(data[i], data[i+1], data[i+2]); };
    std::unique_ptr<SmCurve> curve;
    if (kind == 1) curve.reset(new (context) SmLine(vec(0), vec(6), 3, &context));
    if (kind == 2) curve.reset(new (context) SmHermiteCurve(vec(0), vec(3), vec(6), vec(9), 3));
    SmVector3d point(11, 13, 17), d1(19, 23, 29), d2(31, 37, 41);
    SmVector3d* p = (mask & 1) ? &point : nullptr;
    SmVector3d* q = (mask & 2) ? &d1 : nullptr;
    SmVector3d* r = (mask & 4) ? &d2 : nullptr;
    *operation_status = SmApiEvaluateCurve(curve.get(), parameter, p, q, r);
    const SmVector3d values[3] = {point, d1, d2};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out[i*3+j] = values[i][j];
    return 0;
    SC_CATCH
}
