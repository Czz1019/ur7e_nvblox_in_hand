// SPDX-License-Identifier: Apache-2.0
#include <torch/library.h>
#include <nvblox_torch/py_mapper.h>
#include "export.hpp"
#include <climits>

using PyMapper = c10::intrusive_ptr<pynvblox::Mapper>;
at::Tensor tensor(std::vector<float> data, int64_t columns) {
  auto out = at::empty({int64_t(data.size())/columns, columns}, at::kFloat);
  if (!data.empty()) std::memcpy(out.data_ptr<float>(), data.data(), data.size()*sizeof(float));
  return out;
}
at::Tensor esdf(PyMapper p, int64_t selection, double band) {
  return tensor(exportEsdf(p->getNvbloxMapper(0)->esdf_layer(), selection, band), 7);
}
at::Tensor esdfStrided(PyMapper p, int64_t selection, double band, int64_t stride) {
  TORCH_CHECK(stride>=1 && stride<=INT_MAX,"ESDF stride must be a positive int");
  return tensor(exportEsdf(p->getNvbloxMapper(0)->esdf_layer(), selection, band, int(stride)), 7);
}
at::Tensor layer(PyMapper p, int64_t kind, double weight) {
  auto m=p->getNvbloxMapper(0);
  TORCH_CHECK(kind==0 || kind==1,"layer kind must be TSDF=0 or color=1");
  return tensor(kind==0 ? exportTsdf(m->tsdf_layer(),weight) : exportColor(m->color_layer(),weight),8);
}
void clearRadius(PyMapper p, double x,double y,double z,double radius) {
  TORCH_CHECK(radius>0,"radius must be positive");
  p->getNvbloxMapper(0)->clearOutsideRadius(nvblox::Vector3f(x,y,z),radius);
}
void slice(PyMapper p) { p->getNvbloxMapper(0)->updateEsdfSlice(); }
double voxelSize(PyMapper p) { return p->getNvbloxMapper(0)->tsdf_layer().voxel_size(); }
TORCH_LIBRARY(ur7e_nvblox_native, m) {
  m.def("export_esdf",&esdf); m.def("export_esdf_strided",&esdfStrided); m.def("export_layer",&layer);
  m.def("clear_radius",&clearRadius); m.def("update_esdf_slice",&slice);
  m.def("voxel_size",&voxelSize);
}
