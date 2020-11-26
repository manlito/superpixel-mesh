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
  meshing_options.target_area = 15 * 15;

  cv::Mat image_gray;
  cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
  // Disabled to internal gaussian blur
  cv::GaussianBlur(image_gray, image_gray, cv::Size(25, 25), 2.0);
  cv::GaussianBlur(image_gray, image_gray, cv::Size(25, 25), 2.0);

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
  image.Blur();
  cv::Mat image_blurred(image.Height(), image.Width(), CV_8UC1, (void*)image.GetImageData().data());
  cv::imshow("Blurred", image_blurred);

  superpixel_mesh::SuperpixelsMesh meshing(image, meshing_options);
  meshing.SeedSuperpixelsMesh();
  auto mesh = meshing.GetMesh();
  cv::Mat initial_mesh = draw_mesh(image_color, mesh);
  cv::imshow("InitialMesh", initial_mesh);
  cv::waitKey(0);

  meshing.OptimizeSuperpixelsMesh();
  cv::Mat optimized_mesh = draw_mesh(image_color, meshing.GetMesh());
  cv::imshow("OptimizedMesh", optimized_mesh);
  cv::waitKey();

  return EXIT_SUCCESS;
}
