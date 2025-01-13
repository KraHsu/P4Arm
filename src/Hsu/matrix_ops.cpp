#include "units.h"
#include <Hsu/matrix_ops.h>
#include <algorithm>
#include <array>
#include <vector>

using namespace Eigen;

namespace Hsu {
// Function to compute Euler to Rotation Matrix
Matrix3d euler_to_rotation_matrix(double rx, double ry, double rz) {
  Matrix3d Rz;
  Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;

  Matrix3d Ry;
  Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);

  Matrix3d Rx;
  Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);

  return Rz * Ry * Rx;
}

// Compute the previous joint position
Vector3d compute_previous_joint_position(const Vector3d& end_pos, const Vector3d& euler_angles, double L) {
  Matrix3d R = euler_to_rotation_matrix(euler_angles(0), euler_angles(1), euler_angles(2));
  Vector3d direction = R.col(2);  // Equivalent to [0, 0, 1] rotated by R
  return end_pos - L * direction;
}

// Compute the next joint position
Vector3d compute_next_joint_position(const Vector3d& end_pos, const Vector3d& euler_angles, double L) {
  Matrix3d R = euler_to_rotation_matrix(euler_angles(0), euler_angles(1), euler_angles(2));
  Vector3d direction = R.col(2);  // Equivalent to [0, 0, 1] rotated by R
  return end_pos + L * direction;
}

// Generate cuboid vertices
std::vector<Vector3d> generate_cuboid_vertices(const Vector3d& a, const Vector3d& b) {
  double x_min = std::min(a(0), b(0));
  double x_max = std::max(a(0), b(0));
  double y_min = std::min(a(1), b(1));
  double y_max = std::max(a(1), b(1));
  double z_min = std::min(a(2), b(2));
  double z_max = std::max(a(2), b(2));

  return {{x_min, y_min, z_min}, {x_min, y_min, z_max}, {x_min, y_max, z_min}, {x_min, y_max, z_max},
          {x_max, y_min, z_min}, {x_max, y_min, z_max}, {x_max, y_max, z_min}, {x_max, y_max, z_max}};
}

// Check if a line segment intersects a parallelepiped
bool is_intersecting_parallelepiped(const std::vector<Vector3d>& vertices,
                                    const std::pair<Vector3d, Vector3d>& segment) {
  auto plane_normal = [](const Vector3d& v0, const Vector3d& v1, const Vector3d& v2) {
    return (v1 - v0).cross(v2 - v0);
  };

  auto is_segment_intersecting_triangle = [&](const Vector3d& p1, const Vector3d& p2, const Vector3d& v0,
                                              const Vector3d& v1, const Vector3d& v2) {
    Vector3d normal = plane_normal(v0, v1, v2);
    double d1 = normal.dot(p1 - v0);
    double d2 = normal.dot(p2 - v0);

    if (d1 * d2 > 0) return false;  // Same side

    double t = d1 / (d1 - d2);
    Vector3d intersection_point = p1 + t * (p2 - p1);

    auto vector_area = [](const Vector3d& a, const Vector3d& b) { return 0.5 * (a.cross(b)).norm(); };

    double total_area = vector_area(v1 - v0, v2 - v0);
    double area1 = vector_area(intersection_point - v0, v1 - v0);
    double area2 = vector_area(intersection_point - v1, v2 - v1);
    double area3 = vector_area(intersection_point - v2, v0 - v2);

    return std::abs<double>(area1 + area2 + area3 - total_area) < 1e-6;
  };

  std::vector<std::array<int, 3>> faces = {
      {0, 1, 2}, {0, 2, 3},  // Front
      {4, 5, 6}, {4, 6, 7},  // Back
      {0, 1, 5}, {0, 5, 4},  // Bottom
      {2, 3, 7}, {2, 7, 6},  // Top
      {1, 2, 6}, {1, 6, 5},  // Right
      {0, 3, 7}, {0, 7, 4}   // Left
  };

  for (const auto& face : faces) {
    if (is_segment_intersecting_triangle(segment.first, segment.second, vertices[face[0]], vertices[face[1]],
                                         vertices[face[2]])) {
      return true;
    }
  }

  return false;
}

Eigen::Matrix3d get_Rx(units::angle::radian_t angle) {
  Eigen::Matrix3d Rx;
  double cosA = units::math::cos(angle);
  double sinA = units::math::sin(angle);
  Rx << 1, 0, 0, 0, cosA, -sinA, 0, sinA, cosA;
  return Rx;
}

Eigen::Matrix3d get_Ry(units::angle::radian_t angle) {
  Eigen::Matrix3d Ry;
  double cosA = units::math::cos(angle);
  double sinA = units::math::sin(angle);
  Ry << cosA, 0, sinA, 0, 1, 0, -sinA, 0, cosA;
  return Ry;
}

Eigen::Matrix3d get_Rz(units::angle::radian_t angle) {
  Eigen::Matrix3d Rz;
  double cosA = units::math::cos(angle);
  double sinA = units::math::sin(angle);
  Rz << cosA, -sinA, 0, sinA, cosA, 0, 0, 0, 1;
  return Rz;
}
}  // namespace Hsu