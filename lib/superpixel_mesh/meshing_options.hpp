#pragma once
#include <functional>
#include <superpixel_mesh/report.hpp>

namespace superpixel_mesh {

struct MeshingOptions {

  struct SuperPixel {
    double target_area{625};
  } superpixel;

  struct Optimization {
    int max_iterations{-1};
  } optimization;
};

} // namespace superpixel_mesh
