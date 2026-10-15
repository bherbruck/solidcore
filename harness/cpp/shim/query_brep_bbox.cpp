// SPDX-License-Identifier: Apache-2.0
// Real B-rep bounding-box operations. Construction/export reuses the common
// traversal reader; export never calls a bounding-box operation under test.
#include "sc_edgeuse_graph.h"
#include <array>
namespace {
// A B-rep query reads ALL owned vertices/edges/faces, including topology not
// reachable from region shells. The shared P1 walk deliberately walks only
// region-reachable topology, so complete its registry before exporting or edits.
void bbox_walk(SmBrep* brep,Graph& graph) {
    walk(brep,graph);
    SmTArray<SmVertex*> vertices;brep->GetVertices(vertices);
    SmTArray<SmEdge*> edges;brep->GetEdges(edges);
    SmTArray<SmFace*> faces;brep->GetFaces(faces);
    for(ULONG i=0;i<vertices.GetSize();i++) graph.vertex(vertices[i]);
    for(ULONG i=0;i<edges.GetSize();i++) graph.edge(edges[i]);
    for(ULONG i=0;i<faces.GetSize();i++) graph.face(faces[i]);
    // Resolve use records and links of newly registered owned objects too.
    walk(brep,graph);
}
}
SC_EXPORT int32_t sc_query_brep_bbox(int32_t op,const char* path,const double* input,int64_t n_in,
    double* out,int64_t cap,int64_t* out_len) {
    if(op<0 || op>1 || !input || n_in<8) return SC_ERR_ARG;
    SC_TRY
    Rd rd{input,0}; Built built; SmStatus status=SM_SUCCESS;
    const SmContext& ctx=the_context();
    if(rd.flag()) built.brep=build(rd,path,status);
    else built.brep=new (ctx) SmBrep();
    if(!built.brep || status!=SM_SUCCESS) return status==SM_SUCCESS?SC_ERR_ARG:(int32_t)status;
    SmBrep* brep=built.brep;
    int64_t nv=rd.idx(),ne=rd.idx();
    double vt=rd.num(),et=rd.num(),ft=rd.num();
    bool drop=rd.flag();
    for(int64_t i=0;i<ne;i++) {
        SmPoint3d a((double)i*0.01,0.1,0.0),b((double)i*0.01+1.0,0.1,0.0);
        SmBSplineCurve* curve=nullptr;SmEdge* edge=nullptr;
        status=SmBSplineCurve::CreateLineSegment(ctx,3,a,b,curve);
        if(status!=SM_SUCCESS) return (int32_t)status;
        status=brep->CreateWireEdgeFromCurve(curve,curve->GetNaturalInterval(),edge);
        if(status!=SM_SUCCESS) return (int32_t)status;
    }
    for(int64_t i=0;i<nv;i++) {
        SmShell* shell=nullptr;SmVertex* vertex=nullptr;
        status=brep->MakeShellVertex(brep->GetInfiniteRegion(),SmPoint3d((double)i*0.01,(double)(i%7)*0.001,0.0),shell,vertex);
        if(status!=SM_SUCCESS) return (int32_t)status;
    }
    Graph graph;bbox_walk(brep,graph);
    for(auto* v:graph.vertices) v->SetTolerance(SmZoneTol3d(vt),FALSE,FALSE);
    for(auto* e:graph.edges) e->SetTolerance(SmZoneTol3d(et),FALSE,FALSE);
    for(auto* f:graph.faces) f->SetTolerance(SmZoneTol3d(ft),FALSE,FALSE);
    if(drop) for(auto* e:graph.edges) {
        SmTArray<SmFace*> faces;e->GetFaces(faces);
        if(faces.GetSize()==0) e->SetCurve(NULL,TRUE,FALSE,FALSE);
    }
    int64_t count=rd.idx();std::vector<std::array<int,4>> steps;
    for(int64_t i=0;i<count;i++) {
        int code=(int)rd.idx();bool approx=rd.flag(),tight=rd.flag(),expand=rd.flag();
        steps.push_back({code,approx?1:0,tight?1:0,expand?1:0});
    }
    if(rd.at!=n_in) return SC_ERR_ARG;
    std::vector<double> result;
    if(op==0) {
        Graph final_graph;bbox_walk(brep,final_graph);result.push_back(0.0);
        export_graph(brep,final_graph,result);
    } else {
        result.push_back((double)graph.vertices.size());result.push_back((double)graph.edges.size());
        result.push_back((double)graph.faces.size());
        for(const auto& step:steps) {
            SmExtent3d box;box.SetMinMax(SmPoint3d(-17,-19,-23),SmPoint3d(29,31,37));
            if(step[0]==0) status=brep->CalculateBoundingBox(box,step[1],step[2],step[3]);
            else if(step[0]==1) status=brep->CalculateTightBoundingBox(box);
            else if(step[0]==2) status=brep->LockBoundingBox(step[2]);
            else if(step[0]==3) {brep->UnLockBoundingBox();status=SM_SUCCESS;}
            else if(step[0]==4) status=brep->CalculateBoundingBox(box);
            else return SC_ERR_ARG;
            result.push_back(st(status));put_vec(result,box.GetMin());put_vec(result,box.GetMax());
        }
    }
    return sc_copy_out(result.data(),(int64_t)result.size(),out,cap,out_len);
    SC_CATCH
}
