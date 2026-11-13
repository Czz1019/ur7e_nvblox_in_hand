// SPDX-License-Identifier: Apache-2.0
#include "export.hpp"
#include <nvblox/core/cuda_stream.h>
#include <nvblox/core/unified_vector.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
void ck(cudaError_t e) { if(e!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(e)); }
template<class V> __global__ void compact(const nvblox::VoxelBlock<V>*const* blocks,
 const nvblox::Index3D* ids,int count,float size,int selection,float threshold,int stride,float* out,int* written) {
 const int i=blockIdx.x*blockDim.x+threadIdx.x;
 if(i>=count*512)return;
 const int b=i/512,j=i%512,x=j/64,y=(j/8)%8,z=j%8;
 if constexpr(std::is_same_v<V,nvblox::EsdfVoxel>) {
  // Global lattice alignment keeps sampling stable across block order and negative indices.
  if((ids[b].x()*8+x)%stride || (ids[b].y()*8+y)%stride || (ids[b].z()*8+z)%stride)return;
 }
 const V& v=blocks[b]->voxels[x][y][z];
 float row[8]={};
 row[0]=(ids[b].x()*8+x+.5f)*size; row[1]=(ids[b].y()*8+y+.5f)*size;row[2]=(ids[b].z()*8+z+.5f)*size;
 int columns=8;
 if constexpr(std::is_same_v<V,nvblox::EsdfVoxel>) {
  if(!v.observed || !isfinite(v.squared_distance_vox) || v.squared_distance_vox<0)return;
  float d=sqrtf(v.squared_distance_vox)*size;
  if(selection==1 && d>threshold)return;
  if(selection==2 && !v.is_inside)return;
  row[3]=v.is_inside ? -d:d;row[4]=1;row[5]=v.is_inside;row[6]=v.is_site;columns=7;
 } else {
  if(!isfinite(v.weight) || v.weight<threshold)return;
  row[4]=v.weight;
  if constexpr(std::is_same_v<V,nvblox::TsdfVoxel>)row[3]=v.distance;
  else {row[5]=v.color.r();row[6]=v.color.g();row[7]=v.color.b();}
 }
 int k=atomicAdd(written,1);for(int c=0;c<columns;++c)out[k*columns+c]=row[c];
}
template<class V> std::vector<float> run(const nvblox::VoxelBlockLayer<V>& layer,int selection,float threshold,int columns,int stride=1) {
 ck(cudaDeviceSynchronize()); // wheel stream is private: explicit cross-stream ownership boundary.
 nvblox::CudaStreamOwning stream;
 nvblox::device_vector<const nvblox::VoxelBlock<V>*> ptrs;
 nvblox::device_vector<nvblox::Index3D> ids;
 nvblox::device_vector<float> output;
 nvblox::device_vector<int> counter(1);
 std::vector<float> result;
 const auto all=layer.getAllBlockIndices();
 for(size_t offset=0;offset<all.size();offset+=512) {
  size_t end=std::min(all.size(),offset+512);
  std::vector<const nvblox::VoxelBlock<V>*> host_ptrs;
  std::vector<nvblox::Index3D> host_ids;
  for(size_t j=offset;j<end;++j){auto b=layer.getBlockAtIndex(all[j]);if(b){host_ptrs.push_back(b.get());host_ids.push_back(all[j]);}}
  if(host_ids.empty())continue;
  ptrs.copyFromAsync(host_ptrs,stream);ids.copyFromAsync(host_ids,stream);
  output.resizeAsync(host_ids.size()*512*columns,stream);
  ck(cudaMemsetAsync(counter.data(),0,sizeof(int),stream));
  compact<<<(host_ids.size()*512+255)/256,256,0,stream>>>(ptrs.data(),ids.data(),host_ids.size(),layer.voxel_size(),selection,threshold,stride,output.data(),counter.data());
  ck(cudaGetLastError());int n=0;ck(cudaMemcpyAsync(&n,counter.data(),sizeof(int),cudaMemcpyDeviceToHost,stream));stream.synchronize();
  if(n<0 || n>int(host_ids.size()*512))throw std::runtime_error("Invalid native export count");
  size_t previous=result.size();result.resize(previous+n*columns);
  if(n){ck(cudaMemcpyAsync(result.data()+previous,output.data(),n*columns*sizeof(float),cudaMemcpyDeviceToHost,stream));stream.synchronize();}
 }
 return result;
}
}
std::vector<float> exportEsdf(const nvblox::EsdfLayer& l,int s,float b,int stride){if(s<0||s>2||!(b>0)||stride<1)throw std::invalid_argument("Invalid export selection");return run(l,s,b,7,stride);}
std::vector<float> exportTsdf(const nvblox::TsdfLayer& l,float w){return run(l,0,w,8);}
std::vector<float> exportColor(const nvblox::ColorLayer& l,float w){return run(l,0,w,8);}
