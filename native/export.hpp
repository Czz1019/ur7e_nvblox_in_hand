#pragma once
#include <nvblox/map/layer.h>
#include <nvblox/map/common_names.h>
#include <vector>
// Rows: x,y,z,distance,observed,inside,site for ESDF;
// x,y,z,distance,weight,r,g,b for TSDF/color. No synthetic query lattice.
std::vector<float> exportEsdf(const nvblox::EsdfLayer&, int, float, int stride = 1);
std::vector<float> exportTsdf(const nvblox::TsdfLayer&, float);
std::vector<float> exportColor(const nvblox::ColorLayer&, float);
