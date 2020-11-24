#pragma once
#include <functional>
#include <string>
#include <superpixel_mesh/report.hpp>

namespace superpixel_mesh {

struct MeshingOptions {

  // Each individual rectangle desired area
  double target_area{625};

  // Max iterations to use during optimization
  int max_iterations{30};

  // Regularization weight. The higher, the more an square of target area
  double regularization{5.0};

};

} // namespace superpixel_mesh
