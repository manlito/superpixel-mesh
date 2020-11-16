#pragma once
#include <cstdint>
#include <tuple>
#include <vector>

namespace superpixel_mesh {

struct Vertex {
  double x{0.0};
  double y{0.0};
};

typedef uint32_t FaceIndex;
struct QuadFace {
  FaceIndex tl;
  FaceIndex tr;
  FaceIndex br;
  FaceIndex bl;
};

struct Mesh {
  std::vector<Vertex> vertices;
  std::vector<QuadFace> faces;

  Vertex const* GetVertices() const { return vertices.data(); }
  QuadFace const* GetFaces() const { return faces.data(); }
};

} // namespace superpixel_mesh
