#pragma once
#include <superpixel_mesh/image.hpp>
#include <superpixel_mesh/meshing_options.hpp>
#include <superpixel_mesh/report.hpp>

namespace superpixel_mesh {

struct MeshingResult {
  Mesh mesh;
  MeshingReport report;
};

class SuperpixelsMesh {
public:
  SuperpixelsMesh(const Image &image, const MeshingOptions &options)
      : image(image), options(options) {}
  Mesh SeedSuperpixelsMesh() const;
  MeshingResult OptimizeSuperpixelsMesh(const Mesh &mesh) const;

protected:
  const MeshingOptions &options;
  const Image &image;
};

} // namespace superpixel_mesh
