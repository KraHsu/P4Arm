#pragma once

#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <units.h>

namespace Hsu {
Eigen::Matrix3d euler_to_rotation_matrix(double rx, double ry, double rz);
Eigen::Vector3d compute_previous_joint_position(const Eigen::Vector3d& end_pos, const Eigen::Vector3d& euler_angles,
                                                double L);
Eigen::Vector3d compute_next_joint_position(const Eigen::Vector3d& end_pos, const Eigen::Vector3d& euler_angles,
                                            double L);
std::vector<Eigen::Vector3d> generate_cuboid_vertices(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
bool is_intersecting_parallelepiped(const std::vector<Eigen::Vector3d>& vertices,
                                    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& segment);
Eigen::Matrix3d get_Rx(units::angle::radian_t angle);
Eigen::Matrix3d get_Ry(units::angle::radian_t angle);
Eigen::Matrix3d get_Rz(units::angle::radian_t angle);
}  // namespace Hsu
