
__global__ void gd_seed_vertices(float3* av,float3* bv,int na,int nb,int seeds,float*best){
 int tid=blockIdx.x*blockDim.x+threadIdx.x;float d=best[0];
 if(tid<seeds*seeds){int ia=(tid/seeds)*(3*na-1)/(seeds-1),ib=(tid%seeds)*(3*nb-1)/(seeds-1);d=norm2(av[ia]-bv[ib]);}
 typedef cub::BlockReduce<float,128> Reduce;__shared__ typename Reduce::TempStorage temp;
 float minimum=Reduce(temp).Reduce(d,cub::Min());if(threadIdx.x==0)atomicMin(best,minimum);
}
struct AsyncSlot : Workspace {
 cudaStream_t stream;int*hlen;float*hvalue;int phase=0,now=1,len=1;g_bvtt_SoA in,next;
 std::shared_ptr<Mesh>a,b;
 AsyncSlot(){CK(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));CK(cudaMallocHost((void**)&hlen,sizeof(int)));CK(cudaMallocHost((void**)&hvalue,sizeof(float)));}
 ~AsyncSlot(){cudaStreamDestroy(stream);cudaFreeHost(hlen);cudaFreeHost(hvalue);}
 void start(std::shared_ptr<Mesh>aa,std::shared_ptr<Mesh>bb,const float*mat,int small,int seeds){
  a=aa;b=bb;now=1;len=1;in=first;next=second;
  g_transf t;t._off=make_float3(mat[9],mat[10],mat[11]);t._trf=make_float3x3(make_float3(mat[0],mat[3],mat[6]),make_float3(mat[1],mat[4],mat[7]),make_float3(mat[2],mat[5],mat[8]));
  if(size_t(a->n)*b->n<=size_t(small)||a->deep==1){
   int count=a->n*b->n;if(count>4*GD_CAP)throw std::runtime_error("small-product capacity overflow");
   gd_small<<<(count+127)/128,128,0,stream>>>(a->vertices,b->vertices,a->n,b->n,t._trf,t._off,out);
   cub::DeviceReduce::Min(scratch,scratch_bytes,out,best,count,stream);
   CK(cudaMemcpyAsync(hvalue,best,sizeof(float),cudaMemcpyDeviceToHost,stream));phase=3;
  }else{
   CK(cudaMemsetAsync(mutex,0,(1<<a->deep)*sizeof(int),stream));
   kernel_updateVtx<<<(a->n*3+255)/256,256,0,stream>>>(a->vertices,av,t,a->n*3);
   kernel_updateBV<<<((1<<(a->deep-1))+127)/128,128,0,stream>>>(an,av,mutex,a->deep,a->n);
   kernel_gpu_init<<<1,1,0,stream>>>(an,b->nodes,av,b->vertices,first,length,length,0,0,best,0.f);
   if(seeds>1 && size_t(a->n)*b->n>=100000)gd_seed_vertices<<<(seeds*seeds+127)/128,128,0,stream>>>(av,b->vertices,a->n,b->n,seeds,best);
   phase=1;
  }
 }
 void advance(int target,int maxstep){
  if(phase!=1)return;
  if(len==0||now>=b->deep){
   if(len){
    kernel_narrow_test<<<(4*len+127)/128,128,0,stream>>>(an,b->nodes,av,b->vertices,in,out,ids,ids+1,best,a->deep,b->deep,a->n,b->n,4*len);
    cub::DeviceReduce::Min(scratch,scratch_bytes,out,reduced,4*len,stream);gd_final<<<1,1,0,stream>>>(best,reduced);
   }
   CK(cudaMemcpyAsync(hvalue,best,sizeof(float),cudaMemcpyDeviceToHost,stream));phase=3;return;
  }
  int step=bounded_depth(a->deep,b->deep,len,now,target,maxstep);
  if(step==0){
   finish_frontier(*this,a,b,in,len,now,stream);
   CK(cudaMemcpyAsync(hvalue,best,sizeof(float),cudaMemcpyDeviceToHost,stream));phase=3;return;
  }
  size_t threads=size_t(len)*(1u<<(step*(now<a->deep?2:1)));
  if(threads>size_t(INT_MAX))throw std::runtime_error("parallel BVTT launch overflow");
  CK(cudaMemsetAsync(length,0,sizeof(int),stream));
  if(now<a->deep)kernel_fast_traversal<<<(threads+127)/128,128,0,stream>>>(an,b->nodes,av,b->vertices,a->n,b->n,now,in,next,length,best,len,step);
  else kernel_fast_traversal_onlyB<<<(threads+127)/128,128,0,stream>>>(an,b->nodes,av,b->vertices,a->n,b->n,a->deep,now,in,next,length,best,len,step);
  CK(cudaMemcpyAsync(hlen,length,sizeof(int),cudaMemcpyDeviceToHost,stream));now+=step;std::swap(in,next);phase=2;
 }
 void synchronize(){
  CK(cudaStreamSynchronize(stream));
  if(phase==2){len=*hlen;if(len<0||len>GD_CAP)throw std::runtime_error("parallel BVTT capacity exceeded: "+std::to_string(len)+", mesh="+std::to_string(a->n)+"x"+std::to_string(b->n)+"; result not returned");phase=1;}
  else if(phase==3)phase=0;
 }
};
static std::vector<std::unique_ptr<AsyncSlot>> slots;
py::tuple parallel_batch(std::vector<std::shared_ptr<Mesh>>aa,std::vector<std::shared_ptr<Mesh>>bb,py::array_t<float,py::array::c_style|py::array::forcecast> poses,int target,int maxstep,int small,int concurrency,int seeds){
 if(aa.size()!=bb.size()||poses.size()!=12*aa.size()||concurrency<1||concurrency>128)throw std::runtime_error("Invalid parallel batch");
 py::array_t<float> values(aa.size());float*dest=values.mutable_data();const float*mat=poses.data();
 auto started=std::chrono::steady_clock::now();
 {py::gil_scoped_release release;
  while(int(slots.size())<concurrency)slots.emplace_back(std::make_unique<AsyncSlot>());
  // Warm-up reserves the largest observed source mesh per stream; allocations never recur in steady-state trials.
  int largest=1,depth=1;for(auto a:aa)if(a->n>largest){largest=a->n;depth=a->deep;}
  for(int k=0;k<concurrency;++k)slots[k]->reserve(largest,depth);
  try {
  for(size_t offset=0;offset<aa.size();offset+=concurrency){
   int used=std::min(size_t(concurrency),aa.size()-offset);
   for(int k=0;k<used;++k)slots[k]->start(aa[offset+k],bb[offset+k],mat+12*(offset+k),small,seeds);
   bool unfinished=true;
   while(unfinished){
    for(int k=0;k<used;++k)slots[k]->advance(target,maxstep);
    unfinished=false;
    for(int k=0;k<used;++k)if(slots[k]->phase){slots[k]->synchronize();if(slots[k]->phase)unfinished=true;}
   }
   for(int k=0;k<used;++k){float v=*slots[k]->hvalue;if(!std::isfinite(v)||v<0)throw std::runtime_error("Invalid parallel gDist result");dest[offset+k]=std::sqrt(v);}
  }
  } catch(...) {
   for(auto& slot:slots){cudaStreamSynchronize(slot->stream);slot->phase=0;}
   throw;
  }
 }
 double wall=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();return py::make_tuple(values,py::none(),wall);
}
