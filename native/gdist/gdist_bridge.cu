#include <cuda_runtime.h>
#include <cub/device/device_reduce.cuh>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <numeric>
#include <memory>
#include <chrono>
#include <stdexcept>
#include <limits>
#include <cstring>
#include "MyCuda/MainCodeDistanceQuery.cu"
namespace py=pybind11;
#define CK(call) do {cudaError_t e=(call);if(e!=cudaSuccess)throw std::runtime_error(std::string(#call)+": "+cudaGetErrorString(e));}while(0)
template<class T> T* alloc(size_t n){T*p;CK(cudaMalloc((void**)&p,n*sizeof(T)));return p;}
struct Mesh {
 int n,deep;float3*vertices;g_box*nodes;std::vector<float3> sorted;
 Mesh(py::array_t<float,py::array::c_style|py::array::forcecast> input){
  auto b=input.request();if(b.ndim!=3||b.shape[1]!=3||b.shape[2]!=3||b.shape[0]<1)throw std::runtime_error("Need full triangle array (N,3,3)");
  n=b.shape[0];deep=0;for(int t=n;t;t>>=1)++deep;
  std::vector<float3> original(3*n);std::memcpy(original.data(),b.ptr,3*n*sizeof(float3));
  std::vector<int> order(n);std::iota(order.begin(),order.end(),0);
  auto center=[&](int tri,int axis){float3 a=original[3*tri],b=original[3*tri+1],c=original[3*tri+2];float3 lo=fminf(a,fminf(b,c)),hi=fmaxf(a,fmaxf(b,c));return ((float*)&lo)[axis]+((float*)&hi)[axis];};
  auto partition=[&](auto&&self,int l,int r,int d)->void{
   if(d==deep)return;float lo[3]={INFINITY,INFINITY,INFINITY},hi[3]={-INFINITY,-INFINITY,-INFINITY};
   for(int i=l;i<r;++i)for(int j=0;j<3;++j){float c=center(order[i],j);lo[j]=std::min(lo[j],c);hi[j]=std::max(hi[j],c);}
   int axis=0;for(int j=1;j<3;++j)if(hi[j]-lo[j]>hi[axis]-lo[axis])axis=j;
   int mid=l+(r-l+1)/2;std::nth_element(order.begin()+l,order.begin()+mid,order.begin()+r,[&](int a,int b){return center(a,axis)<center(b,axis);});
   self(self,l,mid,d+1);self(self,mid,r,d+1);
  };partition(partition,0,n,1);
  sorted.resize(3*n);for(int i=0;i<n;++i)for(int j=0;j<3;++j)sorted[3*i+j]=original[3*order[i]+j];
  vertices=alloc<float3>(3*n);nodes=alloc<g_box>(1<<deep);
  CK(cudaMemcpy(vertices,sorted.data(),3*n*sizeof(float3),cudaMemcpyHostToDevice));
  int*mutex=alloc<int>(1<<deep);CK(cudaMemset(mutex,0,(1<<deep)*sizeof(int)));
  kernel_updateBV<<<((1<<(deep-1))+127)/128,128>>>(nodes,vertices,mutex,deep,n);
  CK(cudaDeviceSynchronize());CK(cudaFree(mutex));
 }
 ~Mesh(){cudaFree(vertices);cudaFree(nodes);}
};
struct Workspace {
 float3* av=nullptr;g_box*an=nullptr;int*mutex=nullptr;size_t capacity=0;
 g_bvtt_SoA first,second;float*out,*best,*reduced;int*length,*ids;void*scratch=nullptr;size_t scratch_bytes=0;
 cudaEvent_t begin,end;std::vector<float3> host_tri;
 Workspace(){
  first.a=alloc<int>(GD_CAP);first.b=alloc<int>(GD_CAP);first.min=alloc<float>(GD_CAP);
  second.a=alloc<int>(GD_CAP);second.b=alloc<int>(GD_CAP);second.min=alloc<float>(GD_CAP);
  out=alloc<float>(4*size_t(GD_CAP));best=alloc<float>(1);reduced=alloc<float>(1);length=alloc<int>(1);ids=alloc<int>(2);
  cub::DeviceReduce::Min(nullptr,scratch_bytes,out,reduced,4*GD_CAP);CK(cudaMalloc(&scratch,scratch_bytes));
  CK(cudaEventCreate(&begin));CK(cudaEventCreate(&end));
 }
 void reserve(int n,int deep){if(size_t(n)<=capacity)return;cudaFree(av);cudaFree(an);cudaFree(mutex);capacity=n;
  av=alloc<float3>(3*n);an=alloc<g_box>(1<<deep);mutex=alloc<int>(1<<deep);
 }
 ~Workspace(){cudaFree(av);cudaFree(an);cudaFree(mutex);cudaFree(first.a);cudaFree(first.b);cudaFree(first.min);cudaFree(second.a);cudaFree(second.b);cudaFree(second.min);cudaFree(out);cudaFree(best);cudaFree(reduced);cudaFree(length);cudaFree(ids);cudaFree(scratch);cudaEventDestroy(begin);cudaEventDestroy(end);}
};
static std::unique_ptr<Workspace> workspace;
__global__ void gd_final(float*best,float*reduced){best[0]=fminf(best[0],reduced[0]);}
__global__ void gd_small(float3*av,float3*bv,int na,int nb,float3x3 r,float3 tr,float*out){
 int k=blockIdx.x*blockDim.x+threadIdx.x;if(k>=na*nb)return;int ia=k/nb,ib=k%nb;
 float3 a[3],b[3];for(int j=0;j<3;++j){a[j]=r*av[3*ia+j]+tr;b[j]=bv[3*ib+j];}out[k]=TriDistPQP(a,b);
}
int choose_depth(int da,int db,int len,int now,int target,int maxstep){
 int remaining=now<da?std::min(da-now,db-now):db-now;if(remaining<=0)return 0;
 int base=now<da?4:2;int step=1;size_t threads=size_t(len)*base;
 while(threads<size_t(target)&&step<remaining&&step<maxstep){++step;threads*=base;}return std::min(step,remaining);
}
// Pose is relative A->B; author's native triangle bounds and BVTT kernels remain in use.
std::pair<float,float> query(std::shared_ptr<Mesh>a,std::shared_ptr<Mesh>b,const float*mat,int target,int maxstep,int small_threshold){
 if(!workspace)workspace=std::make_unique<Workspace>();auto&w=*workspace;
 g_transf t;t._off=make_float3(mat[9],mat[10],mat[11]);t._trf=make_float3x3(make_float3(mat[0],mat[3],mat[6]),make_float3(mat[1],mat[4],mat[7]),make_float3(mat[2],mat[5],mat[8]));
 // All caller inputs have A triangle count <= B; swapping done in Python input preparation.
 if(a->n>b->n)throw std::runtime_error("gDist input order must have A.n<=B.n");
 w.reserve(a->n,a->deep);CK(cudaEventRecord(w.begin));
 if(size_t(a->n)*b->n<=size_t(small_threshold)||a->deep==1){
  int count=a->n*b->n;if(count>4*GD_CAP)throw std::runtime_error("tiny-mesh product overflow");
  gd_small<<<(count+127)/128,128>>>(a->vertices,b->vertices,a->n,b->n,t._trf,t._off,w.out);
  cub::DeviceReduce::Min(w.scratch,w.scratch_bytes,w.out,w.best,count);
 }else{
  CK(cudaMemset(w.mutex,0,(1<<a->deep)*sizeof(int)));
  kernel_updateVtx<<<(a->n*3+255)/256,256>>>(a->vertices,w.av,t,a->n*3);
  kernel_updateBV<<<((1<<(a->deep-1))+127)/128,128>>>(w.an,w.av,w.mutex,a->deep,a->n);
  kernel_gpu_init<<<1,1>>>(w.an,b->nodes,w.av,b->vertices,w.first,w.length,w.length,0,0,w.best,0.f);
  auto in=w.first,out=w.second;int len=1,now=1;
  while(now<b->deep&&len){
   int step=choose_depth(a->deep,b->deep,len,now,target,maxstep);
   size_t threads=size_t(len)*(1u<<(step*(now<a->deep?2:1)));
   if(threads>size_t(INT_MAX))throw std::runtime_error("BVTT launch overflow");
   CK(cudaMemset(w.length,0,sizeof(int)));
   if(now<a->deep)kernel_fast_traversal<<<(threads+127)/128,128>>>(w.an,b->nodes,w.av,b->vertices,a->n,b->n,now,in,out,w.length,w.best,len,step);
   else kernel_fast_traversal_onlyB<<<(threads+127)/128,128>>>(w.an,b->nodes,w.av,b->vertices,a->n,b->n,a->deep,now,in,out,w.length,w.best,len,step);
   CK(cudaMemcpy(&len,w.length,sizeof(int),cudaMemcpyDeviceToHost));
   if(len>GD_CAP)throw std::runtime_error("BVTT capacity exceeded, no truncated result returned");
   now+=step;std::swap(in,out);
  }
  if(len){
   kernel_narrow_test<<<(4*len+127)/128,128>>>(w.an,b->nodes,w.av,b->vertices,in,w.out,w.ids,w.ids+1,w.best,a->deep,b->deep,a->n,b->n,4*len);
   cub::DeviceReduce::Min(w.scratch,w.scratch_bytes,w.out,w.reduced,4*len);gd_final<<<1,1>>>(w.best,w.reduced);
  }
 }
 float value;CK(cudaMemcpy(&value,w.best,sizeof(float),cudaMemcpyDeviceToHost));CK(cudaEventRecord(w.end));CK(cudaEventSynchronize(w.end));float ms;CK(cudaEventElapsedTime(&ms,w.begin,w.end));
 if(!std::isfinite(value)||value<0)throw std::runtime_error("Invalid gDist result");
 return {std::sqrt(value),ms};
}
#include "gdist_parallel.inc"
PYBIND11_MODULE(gdist_native,m){
 py::class_<Mesh,std::shared_ptr<Mesh>>(m,"Mesh").def(py::init<py::array_t<float,py::array::c_style|py::array::forcecast>>()).def_readonly("triangles",&Mesh::n);
 m.def("query",[](std::shared_ptr<Mesh>a,std::shared_ptr<Mesh>b,py::array_t<float,py::array::c_style|py::array::forcecast> pose,int target,int maxstep,int small){if(pose.size()!=12)throw std::runtime_error("relative transform length 12 required");return query(a,b,pose.data(),target,maxstep,small);},py::arg("a"),py::arg("b"),py::arg("pose"),py::arg("target")=8192,py::arg("maxstep")=4,py::arg("small")=1024);
 m.def("batch",[](std::vector<std::shared_ptr<Mesh>>a,std::vector<std::shared_ptr<Mesh>>b,py::array_t<float,py::array::c_style|py::array::forcecast> poses,int target,int maxstep,int small){if(a.size()!=b.size()||poses.size()!=12*a.size())throw std::runtime_error("batch shape mismatch");py::array_t<float> values(a.size());auto v=values.mutable_data();double ms=0;auto start=std::chrono::steady_clock::now();{py::gil_scoped_release release;for(size_t i=0;i<a.size();++i){auto r=query(a[i],b[i],poses.data()+12*i,target,maxstep,small);v[i]=r.first;ms+=r.second;}}double wall=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return py::make_tuple(values,ms,wall);},py::arg("a"),py::arg("b"),py::arg("poses"),py::arg("target")=8192,py::arg("maxstep")=4,py::arg("small")=1024);
 m.def("batch_parallel",&parallel_batch,py::arg("a"),py::arg("b"),py::arg("poses"),py::arg("target")=8192,py::arg("maxstep")=4,py::arg("small")=1024,py::arg("concurrency")=32,py::arg("seeds")=0);
 m.def("synchronize",[](){CK(cudaDeviceSynchronize());});
}
