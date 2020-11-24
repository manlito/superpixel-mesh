#pragma once
#include <superpixel_mesh/image.hpp>
#include <superpixel_mesh/meshing_iteration_callback.hpp>
#include <superpixel_mesh/meshing_options.hpp>
#include <superpixel_mesh/report.hpp>

namespace superpixel_mesh {

class SuperpixelsMesh {
public:
  SuperpixelsMesh(const Image &image, const MeshingOptions &options)
      : image(image), options(options) {}
  void SetIterationCallback(MeshingIterationCallback &iteration_callback);
  void SeedSuperpixelsMesh();
  Mesh GetMesh() const;
  void SetMesh(const Mesh &mesh);
  MeshingReport OptimizeSuperpixelsMesh();

protected:
  const Image &image;
  const MeshingOptions &options;
  Mesh mesh;
  MeshingIterationCallback *iteration_callback{nullptr};
};

} // namespace superpixel_mesh
