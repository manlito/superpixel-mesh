#pragma once
#include <superpixel_mesh/mesh.hpp>

namespace superpixel_mesh {

struct MeshingReport {
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  double total_time{0};
};

} // namespace superpixel_mesh
