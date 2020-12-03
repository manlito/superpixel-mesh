#include "draw_mesh.hpp"
#include <opencv2/imgproc.hpp>
#include <random>

cv::Mat draw_mesh(const cv::Mat &image, const superpixel_mesh::Mesh &mesh) {
  cv::Mat color_mask(image.size(), CV_8UC3);
  color_mask.setTo(0);

  std::random_device r;
  std::default_random_engine e1(r());
  std::uniform_int_distribution<unsigned char> uniform_dist(0, 255);
  for (const auto &[tl, tr, br, bl] : mesh.faces) {
    cv::Rect rect;
    const std::vector<cv::Point> vertices = {
        cv::Point(mesh.vertices.at(tl).x, mesh.vertices.at(tl).y),
        cv::Point(mesh.vertices.at(tr).x, mesh.vertices.at(tr).y),
        cv::Point(mesh.vertices.at(br).x, mesh.vertices.at(br).y),
        cv::Point(mesh.vertices.at(bl).x, mesh.vertices.at(bl).y)};
    // cv::Scalar color(uniform_dist(e1), uniform_dist(e1), uniform_dist(e1));
    cv::Scalar color(255, 255, 255, 0);
    // cv::fillConvexPoly(color_mask, vertices, color);
    cv::polylines(color_mask, vertices, false, color, 2, cv::LINE_AA);
  }

  cv::Mat output = color_mask;
  cv::addWeighted(image, 0.6, color_mask, 0.4, 0, output);
  return output;
}
