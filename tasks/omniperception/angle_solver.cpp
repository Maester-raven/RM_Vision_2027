#include "angle_solver.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>
#include <opencv2/calib3d.hpp>

namespace omniperception {

namespace {

// 和现有 auto_aim Solver 使用相同的装甲板尺寸
constexpr double LIGHTBAR_LENGTH = 56e-3;
constexpr double BIG_ARMOR_WIDTH = 230e-3;
constexpr double SMALL_ARMOR_WIDTH = 135e-3;

constexpr double RAD_TO_DEG =
  180.0 / 3.14159265358979323846;


// 3D 点顺序与现有 Solver 保持一致
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
};

const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
};

}  // namespace


Eigen::Vector2d AngleSolver::solve(
  const auto_aim::Armor & armor,
  const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs)
{
  return solve(
    armor.points,
    armor.type,
    camera_matrix,
    distort_coeffs);
}


Eigen::Vector2d AngleSolver::solve(
  const std::vector<cv::Point2f> & points,
  auto_aim::ArmorType armor_type,
  const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs)
{
  if (points.size() != 4) {
    throw std::runtime_error(
      "AngleSolver requires exactly 4 armor points");
  }

  const auto & object_points =
    (armor_type == auto_aim::ArmorType::big)
      ? BIG_ARMOR_POINTS
      : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec;
  cv::Vec3d tvec;

  const bool success = cv::solvePnP(
    object_points,
    points,
    camera_matrix,
    distort_coeffs,
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE);

  if (!success) {
    throw std::runtime_error(
      "AngleSolver solvePnP failed");
  }

  // OpenCV 相机坐标系：
  //
  // +x -> 图像右边
  // +y -> 图像下边
  // +z -> 相机前方

  const double x = tvec[0];
  const double y = tvec[1];
  const double z = tvec[2];

  if (z <= 0.0) {
    throw std::runtime_error(
      "AngleSolver target is behind camera");
  }

  // 保持现在 omniperception 的符号习惯：
  //
  // 左 -> yaw 为正
  // 右 -> yaw 为负
  //
  // 上 -> pitch 为负
  // 下 -> pitch 为正

  const double yaw =
    -std::atan2(x, z) * RAD_TO_DEG;

  const double horizontal_distance =
    std::sqrt(x * x + z * z);

  const double pitch =
    std::atan2(y, horizontal_distance) * RAD_TO_DEG;

  return {yaw, pitch};
}

}  // namespace omniperception
