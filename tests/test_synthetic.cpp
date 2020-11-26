#include <cmath>
#include <gtest/gtest.h>
#include <superpixel_mesh/superpixel_mesh.hpp>
#include <unordered_set>

void PrintImage(const superpixel_mesh::Image &image,
                const superpixel_mesh::Mesh &mesh) {
  std::unordered_set<int> vertices_encoded;
  for (const auto vertex : mesh.vertices) {
    vertices_encoded.insert((int)vertex.y * image.Width() + (int)vertex.x);
  }
  for (int y = 0; y < image.Height(); y++) {
    for (int x = 0; x < image.Width(); x++) {
      if (vertices_encoded.count(y * image.Width() + x)) {
        std::cout << " . ";
      } else {
        std::cout << std::setfill('0') << std::setw(2)
                  << (int)image.GetImageData()[y * image.Width() + x] << " ";
      }
    }
    std::cout << "\n";
  }
}

TEST(Synthetic, Grid4x4) {

  const int cell_size = 6;
  const int subgrid_size = 16;
  superpixel_mesh::MeshingOptions meshing_options;
  meshing_options.target_area = std::pow(cell_size, 2);
  meshing_options.max_iterations = 30;
  meshing_options.regularization = 5.0;

  superpixel_mesh::ImageData image_data;

  /**
   * Create a 4x4 grid as follows:
   *
   *  ---------
   *  | A | B |
   *  ---------
   *  | C | D |
   *  ---------
   *
   * Where A, B, C and D are subgrids of subgrid_size*subgrid_size.
   * Each pixel fo A will be 0. And 33, 66 and 99 to be
   * used in B, C and D respectively
   *
   */
  const int width = subgrid_size * 2;
  image_data.resize(width * width);
  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      // Fill each subgrid
      superpixel_mesh::ImageDataType color;
      if (row == 0 && col == 0) {
        color = 0;
      } else if (row == 0 && col == 1) {
        color = 33;
      } else if (row == 1 && col == 0) {
        color = 66;
      } else {
        color = 99;
      }
      for (int y = row * subgrid_size; y < (row + 1) * subgrid_size; y++) {
        for (int x = col * subgrid_size; x < (col + 1) * subgrid_size; x++) {
          image_data[y * width + x] = color;
        }
      }
    }
  }

  superpixel_mesh::Image image(width, width, image_data);
  superpixel_mesh::SuperpixelsMesh meshing(image, meshing_options);
  meshing.SeedSuperpixelsMesh();
  auto result = meshing.OptimizeSuperpixelsMesh();
  EXPECT_GT(result.initial_cost, 5e4) << "Initial cost larger than expected";
  EXPECT_LT(result.final_cost, 25.0) << "Final cost exceeds threshold";
}
