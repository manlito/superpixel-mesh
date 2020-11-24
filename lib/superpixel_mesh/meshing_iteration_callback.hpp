#pragma once
#include <superpixel_mesh/mesh.hpp>

namespace superpixel_mesh {

struct MeshingIterationProgress {
  Mesh mesh;
  double cost{0.0};
  int iteration{0};
};

struct MeshingIterationCallback {
public:
  MeshingIterationCallback() {}
  // Convenience function to be setup callback from JavaScript via addFunction
  void SetCallback(unsigned long callback_pointer);
  bool HasCallback() const;
  void operator()(const MeshingIterationProgress &progress);
  MeshingIterationProgress const &GetIterationProgress() const;

private:
  // If set, this will be called on every iteration
  bool has_callback{false};
  unsigned long callback_pointer;
  MeshingIterationProgress iteration_progress;
};
} // namespace superpixel_mesh
