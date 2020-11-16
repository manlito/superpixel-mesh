#pragma once
#include <superpixel_mesh/mesh.hpp>
#include <opencv2/core.hpp>

cv::Mat draw_mesh(const cv::Mat &image, const superpixel_mesh::Mesh &mesh);
