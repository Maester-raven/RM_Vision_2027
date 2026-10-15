#ifndef OMNIPERCEPTION__ANGLE_SOLVER_HPP
#define OMNIPERCEPTION__ANGLE_SOLVER_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <vector>

#include "tasks/auto_aim/armor.hpp"

namespace omniperception
{

class AngleSolver
{
public:
  // 实际项目接口：
  // Armor + 相机标定参数 -> 相机坐标系下的 yaw / pitch
  static Eigen::Vector2d solve(
    const auto_aim::Armor & armor,
    const cv::Mat & camera_matrix,
    const cv::Mat & distort_coeffs);

  // 独立测试接口：
  // 不需要构造完整 Armor
  static Eigen::Vector2d solve(
    const std::vector<cv::Point2f> & points,
    auto_aim::ArmorType armor_type,
    const cv::Mat & camera_matrix,
    const cv::Mat & distort_coeffs);
};

}  // namespace omniperception

#endif  // OMNIPERCEPTION__ANGLE_SOLVER_HPP
