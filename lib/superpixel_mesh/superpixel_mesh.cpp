#include <ceres/autodiff_cost_function.h>
#include <ceres/cubic_interpolation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <cmath>
#include <superpixel_mesh/meshing_cost.hpp>
#include <superpixel_mesh/superpixel_mesh.hpp>
#include <unordered_set>
#include <vector>

namespace superpixel_mesh {

Mesh SuperpixelsMesh::SeedSuperpixelsMesh() const {
  Mesh mesh;

  auto bounded_coordinates = [this](const int x,
                                      const int y) -> std::pair<int, int> {
    return {std::max(0, std::min(image.Width() - 1, x)),
            std::max(0, std::min(image.Height() - 1, y))};
  };
  auto coordinate_to_encoded = [this, &bounded_coordinates](const int x,
                                                              const int y) {
    const auto [x_safe, y_safe] = bounded_coordinates(x, y);
    return y_safe * image.Width() + x_safe;
  };
  auto encoded_to_coordinate =
      [this](const int index) -> std::pair<int, int> {
    return {index % image.Width(), index / image.Width()};
  };

  std::vector<std::tuple<int, int, int, int>> faces;
  const int target_size = std::round(std::sqrt(options.superpixel.target_area));
  for (int row = 0; row < image.Height() - target_size; row += target_size) {
    for (int col = 0; col < image.Width() - target_size; col += target_size) {
      faces.push_back(
          {coordinate_to_encoded(col, row),
           coordinate_to_encoded(col + target_size, row),
           coordinate_to_encoded(col + target_size, row + target_size),
           coordinate_to_encoded(col, row + target_size)});
    }
  }
  // Convert to mesh, with shared vertices
  std::unordered_map<int, FaceIndex> encoded_to_vertex_index;
  auto get_remapped_vertex_index =
      [&encoded_to_vertex_index](const int encoded) {
        if (auto vertex_index = encoded_to_vertex_index.find(encoded);
            vertex_index != encoded_to_vertex_index.end()) {
          return vertex_index->second;
        } else {
          encoded_to_vertex_index[encoded] = encoded_to_vertex_index.size();
          return encoded_to_vertex_index.at(encoded);
        }
      };
  for (const auto &[tl, tr, br, bl] : faces) {
    mesh.faces.push_back(
        QuadFace{get_remapped_vertex_index(tl), get_remapped_vertex_index(tr),
                 get_remapped_vertex_index(br), get_remapped_vertex_index(bl)});
  }
  mesh.vertices.resize(encoded_to_vertex_index.size());
  for (const auto &[face_index, vertex_index] : encoded_to_vertex_index) {
    auto coordinates = encoded_to_coordinate(face_index);
    mesh.vertices[vertex_index] =
        Vertex{(double)coordinates.first, (double)coordinates.second};
  }

  return mesh;
}

MeshingResult SuperpixelsMesh::OptimizeSuperpixelsMesh(const Mesh &mesh) const {
  MeshingResult result;

  Grid grid(image.GetImageData().data(), 0, image.Height(), 0, image.Width());
  Interpolator interpolator(grid);

  ceres::Problem problem;

  // Parametrized vertices
  std::vector<double[2]> vertices(mesh.vertices.size());
  for (size_t vertex_index = 0; vertex_index < mesh.vertices.size();
       vertex_index++) {
    vertices[vertex_index][0] = mesh.vertices[vertex_index].x;
    vertices[vertex_index][1] = mesh.vertices[vertex_index].y;
    problem.AddParameterBlock(vertices.at(vertex_index), 2);
  }

  // Add faces
  constexpr int SIZE = 5;
  constexpr double NORMALIZING_FACTOR = 1.0 / (double)(SIZE * SIZE);
  std::vector<ceres::ResidualBlockId> residuals;
  for (const auto &[v1, v2, v3, v4] : mesh.faces) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<PixelDissimilarityCost<SIZE>,
                                        SIZE * SIZE, 2, 2, 2, 2>(
            new PixelDissimilarityCost<SIZE>(interpolator));
    if (cost_function) {
      residuals.push_back(problem.AddResidualBlock(
          cost_function, nullptr, vertices.at(v1), vertices.at(v2),
          vertices.at(v3), vertices.at(v4)));
    } else {
      throw std::runtime_error("Unable to create cost function");
    }
  }

  for (const auto &[v1, v2, v3, v4] : mesh.faces) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<AreaRegularizationCost, 4, 2, 2, 2, 2>(
            new AreaRegularizationCost(options.superpixel.target_area));
    if (cost_function) {
      problem.AddResidualBlock(
          cost_function,
          new ceres::ScaledLoss(nullptr, 1e0 * NORMALIZING_FACTOR,
                                ceres::TAKE_OWNERSHIP),
          vertices.at(v1), vertices.at(v2), vertices.at(v3), vertices.at(v4));
    } else {
      throw std::runtime_error("Unable to create cost function");
    }
  }

  // Solver time
  ceres::Solver::Options solver_options;
  solver_options.minimizer_type = ceres::TRUST_REGION;
  solver_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.max_num_iterations = 50;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  result.report.initial_cost = summary.initial_cost;
  result.report.final_cost = summary.final_cost;
  result.report.iterations = summary.iterations.size();
  result.report.total_time = summary.total_time_in_seconds;

  result.mesh = mesh;
  for (size_t vertex_index = 0; vertex_index < mesh.vertices.size();
       vertex_index++) {
    result.mesh.vertices[vertex_index].x = vertices[vertex_index][0];
    result.mesh.vertices[vertex_index].y = vertices[vertex_index][1];
  }

  return result;
}

} // namespace superpixel_mesh
