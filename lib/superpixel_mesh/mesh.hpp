#pragma once
#include <cstdint>
#include <tuple>
#include <vector>

namespace superpixel_mesh {

struct Vertex {
  double x{0.0};
  double y{0.0};
};

typedef size_t FaceIndex;
struct QuadFace {
  FaceIndex tl;
  FaceIndex tr;
  FaceIndex br;
  FaceIndex bl;
};

struct Mesh {
  std::vector<Vertex> vertices;
  std::vector<QuadFace> faces;

  size_t GetFacesCount() const { return faces.size(); }
  size_t GetVerticesCount() const { return vertices.size(); }

  Vertex GetVertexAt(FaceIndex vertex_index) const {
    return vertices.at(vertex_index);
  }
  QuadFace GetFaceAt(FaceIndex face_index) const {
    return faces.at(face_index);
  }
};

} // namespace superpixel_mesh
