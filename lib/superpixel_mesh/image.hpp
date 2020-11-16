#pragma once
#include <vector>

namespace superpixel_mesh {

typedef unsigned char ImageDataType;
typedef std::vector<ImageDataType> ImageData;

class Image {
public:
  Image(int width, int height, ImageDataType *const image_data)
      : width(width), height(height),
        image_data(ImageData(image_data, image_data + width * height)) {}
  Image(int width, int height, const ImageData &image_data)
      : width(width), height(height), image_data(image_data) {}
  int Width() const { return width; }
  int Height() const { return height; }
  ImageData const &GetImageData() const { return image_data; }

protected:
  const int width{0};
  const int height{0};
  const ImageData image_data;
};

} // namespace superpixel_mesh
