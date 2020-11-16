#include "draw_mesh.hpp"
#include <filesystem>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <superpixel_mesh/superpixel_mesh.hpp>

int main(int argc, char *argv[]) {

  if (argc != 2) {
    std::cerr << "Usage: ./demo_opencv input_image" << std::endl;
    return EXIT_FAILURE;
  }

  std::filesystem::path input_file = argv[1];
  if (!std::filesystem::exists(input_file)) {
    std::cerr << "Input file " << input_file << " does not exist" << std::endl;
  }

  cv::Mat image_color;
  try {
    image_color = cv::imread(input_file.string(), cv::IMREAD_COLOR);
    if (image_color.empty()) {
      throw std::runtime_error("Read empty image");
    }
  } catch (const std::exception &e) {
    std::cerr << "Error reading image: " << e.what() << std::endl;
  }

  superpixel_mesh::MeshingOptions meshing_options;
  meshing_options.superpixel.target_area = 15 * 15;

  cv::Mat image_gray, image_blurred;
  cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(image_gray, image_gray, cv::Size(25, 25), 2.0);

  cv::imshow("Blurred", image_gray);

  superpixel_mesh::ImageData image_data;
  image_data.resize(image_gray.cols * image_gray.rows);
  for (int row = 0; row < image_gray.rows; row++) {
    auto const *source_ptr = image_gray.ptr(row);
    auto *target_ptr = &image_data[row * image_gray.cols];
    for (int col = 0; col < image_gray.cols;
         col++, source_ptr++, target_ptr++) {
      *target_ptr = *source_ptr;
    }
  }
  superpixel_mesh::Image image(image_gray.cols, image_gray.rows, image_data);

  superpixel_mesh::SuperpixelsMesh meshing(image, meshing_options);
  auto mesh = meshing.SeedSuperpixelsMesh();
  cv::Mat initial_mesh = draw_mesh(image_color, mesh);
  cv::imshow("InitialMesh", initial_mesh);
  cv::waitKey(0);

  auto meshing_result = meshing.OptimizeSuperpixelsMesh(mesh);
  cv::Mat optimized_mesh = draw_mesh(image_color, meshing_result.mesh);
  cv::imshow("OptimizedMesh", optimized_mesh);
  cv::waitKey();

  return EXIT_SUCCESS;
}