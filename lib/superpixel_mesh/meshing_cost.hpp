#pragma once
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <superpixel_mesh/image.hpp>

namespace superpixel_mesh {

typedef ceres::Grid2D<ImageDataType> Grid;
typedef ceres::BiCubicInterpolator<Grid> Interpolator;

template <typename T> class Homography {
public:
  typedef Eigen::Matrix<T, 2, 4> MatX;
  typedef Eigen::Matrix<T, 2, 4> MatB;
  typedef Eigen::Matrix<T, 3, 3> MatH;
  typedef Eigen::Matrix<T, 8, 8> Mat8;
  typedef Eigen::Matrix<T, 8, 1> Vec8;

  static void ComputeHomography(const MatX &x1, const MatX &x2, Vec8 *H) {
    Mat8 L = Mat8::Zero();
    Vec8 b = Vec8::Zero();
    /**
     *
     *  Add 2 rows for each point. Parametrization uses h9 = 1
     *
     *    0       0       0   -x1    -y1    -1    x1*y2  y1*y2    hi   =  -y2
     *   -x1     -y1     -1     0      0     0    x1*x2  y1*x2    hi+1 =   -x2
     */

    for (int i = 0; i < 4; ++i) {
      int r = i * 2;
      L(r, 3) = x1(0, i);
      L(r, 4) = x1(1, i);
      L(r, 5) = T(1.0);
      L(r, 6) = -x1(0, i) * x2(1, i);
      L(r, 7) = -x1(1, i) * x2(1, i);
      b(r) = x2(1, i);

      L(r + 1, 0) = x1(0, i);
      L(r + 1, 1) = x1(1, i);
      L(r + 1, 2) = T(1.0);
      L(r + 1, 6) = -x1(0, i) * x2(0, i);
      L(r + 1, 7) = -x1(1, i) * x2(0, i);
      b(r + 1) = x2(0, i);
    }
    // Solve Ax=B
    Eigen::PartialPivLU<Mat8> dec(L);
    (*H) = dec.solve(b);
  }
};

template <int SIZE, int PADDING = 2> struct PixelDissimilarityCost {

  PixelDissimilarityCost(const Interpolator &inteporlator)
      : interpolator(inteporlator) {}

  template <typename T>
  bool operator()(const T *const v1, const T *const v2, const T *const v3,
                  const T *const v4, T *residuals) const {

    // Compute the homography
    typename Homography<T>::MatX x1, x2;
    x1 << T(-PADDING), T(SIZE + PADDING), T(SIZE + PADDING), T(-PADDING),
        T(-PADDING), T(-PADDING), T(SIZE + PADDING), T(SIZE + PADDING);
    x2 << v1[0], v2[0], v3[0], v4[0], v1[1], v2[1], v3[1], v4[1];

    typename Homography<T>::Vec8 h;
    Homography<T>::ComputeHomography(x1, x2, &h);

    T intensities[SIZE * SIZE];
    T mean = T(0);
    int intensity_index = 0;
    for (int row = 0; row < SIZE; row++) {
      for (int col = 0; col < SIZE; col++, intensity_index++) {
        T x, y, s;
        x = T(col);
        y = T(row);
        s = T(1.0) / (h(6) * x + h(7) * y + T(1.0));
        interpolator.Evaluate(s * (h(3) * x + h(4) * y + h(5)),
                              s * (h(0) * x + h(1) * y + h(2)),
                              &intensities[intensity_index]);
        mean += intensities[intensity_index];
      }
    }
    const T normalization = T(1.0 / double(SIZE * SIZE));
    mean *= normalization;
    for (int intensity_index = 0; intensity_index < SIZE * SIZE;
         intensity_index++) {
      residuals[intensity_index] = intensities[intensity_index] - mean;
    }

    return true;
  }

  const Interpolator &interpolator;
};

struct AreaRegularizationCost {

  AreaRegularizationCost(const double target_area) : target_area(target_area) {}

  template <typename T>
  T TriangleArea(const T *const v1, const T *const v2,
                 const T *const v3) const {
    /**
     * Shoelace formula:
     * (1/2) | x1y2 + x2y3 + x3y1 - x2y1 - x3y2 - x1y3 |
     */
    const auto &x1 = v1[0];
    const auto &y1 = v1[1];
    const auto &x2 = v2[0];
    const auto &y2 = v2[1];
    const auto &x3 = v3[0];
    const auto &y3 = v3[1];

    return 0.5 * ceres::abs(x1 * y2 + x2 * y3 + x3 * y1 - x2 * y1 - x3 * y2 -
                            x1 * y3);
  }

  template <typename T>
  bool operator()(const T *const v1, const T *const v2, const T *const v3,
                  const T *const v4, T *residuals) const {
    T triangle_areas[4];
    triangle_areas[0] = TriangleArea(v4, v3, v2);
    triangle_areas[1] = TriangleArea(v3, v2, v1);
    triangle_areas[2] = TriangleArea(v2, v1, v4);
    triangle_areas[3] = TriangleArea(v1, v4, v3);

    const T target_triangle_area = T(0.5 * target_area);
    for (int triangle_index = 0; triangle_index < 4; triangle_index++)
      residuals[triangle_index] =
          target_triangle_area - triangle_areas[triangle_index];

    return true;
  }

  const double target_area;
};

} // namespace superpixel_mesh
