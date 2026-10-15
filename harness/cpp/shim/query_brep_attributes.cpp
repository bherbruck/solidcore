// SPDX-License-Identifier: Apache-2.0
// query.brep: real SmPolyDecimate attribute constructors and virtual MakeCopy.
// Public member-pointer accessors expose protected payloads without casting a
// base object to a fictitious derived instance or changing upstream sources.
#include "sc_shim.h"
#include "StdAfx.h"
#include <SmApiGeneral.h>
#include <SmPolyDecimate.h>
#include <SmContext.h>
#include <memory>
#include <vector>
namespace {
struct DecAccess : SmDecimateVertexAttr {
    static auto sort() { return &DecAccess::m_lSortIndex; }
    static auto cls() { return &DecAccess::m_eVertClass; }
    static auto distance() { return &DecAccess::m_dVertexDistance; }
    static auto error() { return &DecAccess::m_dMaxFaceError; }
    static auto normal() { return &DecAccess::m_vAveragePlaneNormal; }
};
struct QvAccess : SmQuadricVertexAttr {
    static auto matrix() { return &QvAccess::m_Q; }
    static auto linear() { return &QvAccess::m_b; }
    static auto constant() { return &QvAccess::m_c; }
};
struct QeAccess : SmQuadricEdgeAttr {
    static auto sort() { return &QeAccess::m_lSortIndex; }
};
void put(std::vector<double>& o, const SmVector3d& v) { o.insert(o.end(),{v.x,v.y,v.z}); }
SmVector3d vec(const double* d) { return SmVector3d(d[0],d[1],d[2]); }
void payload(std::vector<double>& o, const SmDecimateVertexAttr& a) {
    o.push_back(a.*DecAccess::distance()); o.push_back(a.*DecAccess::error());
    put(o,a.*DecAccess::normal());
}
void payload(std::vector<double>& o, const SmQuadricVertexAttr& a) {
    for (const auto& row : a.*QvAccess::matrix()) put(o,row);
    put(o,a.*QvAccess::linear()); o.push_back(a.*QvAccess::constant());
}
void payload(std::vector<double>& o, const SmQuadricEdgeAttr& a) { put(o,a.GetPoint()); o.push_back(a.GetError()); }
}
SC_EXPORT int32_t sc_query_brep_attributes(int32_t kind, uint64_t id, int32_t behavior,
    int32_t mutate, uint64_t sort, int32_t cls, const double* data,
    double* out, int64_t cap, int64_t* out_len, uint64_t* ints) {
    if(kind<0 || kind>2 || behavior<0 || behavior>4 || mutate<0 || mutate>1 || cls<0 || cls>5 || !data || !ints) return SC_ERR_ARG;
    SC_TRY
    const SmContext& ctx=*SmApiGetOrCreateContext();
    std::vector<double> o;
    std::unique_ptr<SmAttribute> original, copy;
    uint64_t source_sort=0, copy_sort=0, source_class=0, copy_class=0;
    if(kind==0) {
        auto a=std::unique_ptr<SmDecimateVertexAttr>(new (ctx) SmDecimateVertexAttr((ULONG)id,(SmAttributeBehaviorType)behavior));
        payload(o,*a);
        if(mutate) {
            a.get()->*DecAccess::sort()=(ULONG)sort;
            a.get()->*DecAccess::cls()=(SmPolyVertexClass)cls;
            a.get()->*DecAccess::distance()=data[0]; a.get()->*DecAccess::error()=data[1];
            a.get()->*DecAccess::normal()=vec(data+2);
        }
        copy.reset(a->MakeCopy(ctx));
        if(!copy) return SC_ERR_ARG;
        auto& c=*static_cast<SmDecimateVertexAttr*>(copy.get());
        payload(o,*a); payload(o,c);
        source_sort=a.get()->*DecAccess::sort(); copy_sort=c.*DecAccess::sort();
        source_class=a.get()->*DecAccess::cls(); copy_class=c.*DecAccess::cls();
        original=std::move(a);
    } else if(kind==1) {
        auto a=std::unique_ptr<SmQuadricVertexAttr>(new (ctx) SmQuadricVertexAttr((ULONG)id,(SmAttributeBehaviorType)behavior));
        payload(o,*a);
        if(mutate) {
            auto& q=a.get()->*QvAccess::matrix();
            for(int k=0;k<3;k++) q[k]=vec(data+3*k);
            a.get()->*QvAccess::linear()=vec(data+9); a.get()->*QvAccess::constant()=data[12];
        }
        copy.reset(a->MakeCopy(ctx));
        if(!copy) return SC_ERR_ARG;
        payload(o,*a); payload(o,*static_cast<SmQuadricVertexAttr*>(copy.get()));
        original=std::move(a);
    } else {
        auto a=std::unique_ptr<SmQuadricEdgeAttr>(new (ctx) SmQuadricEdgeAttr((ULONG)id,(SmAttributeBehaviorType)behavior));
        payload(o,*a);
        if(mutate) { a->SetSortIndex((ULONG)sort); a->SetPoint(vec(data)); a->SetError(data[3]); }
        copy.reset(a->MakeCopy(ctx));
        if(!copy) return SC_ERR_ARG;
        auto& c=*static_cast<SmQuadricEdgeAttr*>(copy.get());
        payload(o,*a); payload(o,c);
        // Do not read the constructor's indeterminate sort field.
        if(mutate) source_sort=a->GetSortIndex();
        copy_sort=c.GetSortIndex(); original=std::move(a);
    }
    ints[0]=original->GetAttributeID(); ints[1]=copy->GetAttributeID();
    ints[2]=(uint64_t)original->GetBehavior(); ints[3]=(uint64_t)copy->GetBehavior();
    ints[4]=source_sort; ints[5]=copy_sort; ints[6]=source_class; ints[7]=copy_class;
    return sc_copy_out(o.data(),(int64_t)o.size(),out,cap,out_len);
    SC_CATCH
}
