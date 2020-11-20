#include <cmath>
#include <superpixel_mesh/image.hpp>

namespace superpixel_mesh {

void Image::Blur(int kernel_size, double sigma) {
  if (kernel_size % 2 == 0) {
    kernel_size++;
  }
  const int padding = kernel_size / 2;

  // Init kernel
  std::vector<double> kernel(kernel_size, 0.0);

  // Fill kernel
  {
    const double s = 2.0 * sigma * sigma;
    double sum = 0.0;
    for (int x = -padding; x <= padding; x++) {
      kernel[x + padding] = std::exp(-(double)(x * x) / s);
      sum += kernel[x + padding];
    }
    const double normalization = 1.0 / sum;
    for (auto &kernel_cell : kernel)
      kernel_cell *= normalization;
  }

  // Apply horizontal blur
  for (int row = 0; row < height; row++) {
    auto *row_ptr = &image_data[row * width];
    for (int col = 0; col < width; col++) {
      double intensity = 0.0;
      for (int x = -padding; x <= padding; x++) {
        intensity += kernel[x + padding] *
                     (double)row_ptr[std::max(0, std::min(width - 1, col + x))];
      }
      row_ptr[col] = (ImageDataType)std::max(0, std::min(255, (int)intensity));
    }
  }
  // Apply vertical blur
  for (int col = 0; col < width; col++) {
    for (int row = 0; row < height; row++) {
      double intensity = 0.0;
      for (int x = -padding; x <= padding; x++) {
        intensity += kernel[x + padding] *
                     (double)image_data[std::max(0, std::min(height - 1, row + x)) * width + col];
      }
      image_data[row * width + col] = (ImageDataType)std::max(0, std::min(255, (int)intensity));
    }
  }
}

} // namespace superpixel_mesh
