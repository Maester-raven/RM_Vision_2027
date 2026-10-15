#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "tasks/omniperception/angle_solver.hpp"

namespace
{

constexpr double LIGHTBAR_LENGTH = 56e-3;
constexpr double SMALL_ARMOR_WIDTH = 135e-3;

const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
};

constexpr double RAD_TO_DEG =
  180.0 / 3.14159265358979323846;


// 使用 DA6415771 已验证过的标定参数
const cv::Mat CAMERA_MATRIX =
  (cv::Mat_<double>(3, 3) <<
    970.13427539, 0.0,          258.35300294,
    0.0,          968.86224255, 109.26852711,
    0.0,          0.0,          1.0);

const cv::Mat DISTORT_COEFFS =
  (cv::Mat_<double>(1, 5) <<
    0.04867245,
    -0.08070563,
    -0.0011765,
    -0.00081667,
    0.03055999);


void run_test(
  const std::string & name,
  double x,
  double y,
  double z)
{
  // 构造一个正对相机的装甲板姿态。
  //
  // object:
  // x = 法向
  // y = 水平方向
  // z = 竖直方向
  //
  // camera:
  // x = 右
  // y = 下
  // z = 前
  cv::Mat rotation_matrix =
    (cv::Mat_<double>(3, 3) <<
      0, -1,  0,
      0,  0, -1,
      1,  0,  0);

  cv::Vec3d rvec;
  cv::Rodrigues(rotation_matrix, rvec);

  cv::Vec3d tvec{x, y, z};

  std::vector<cv::Point2f> image_points;

  cv::projectPoints(
    SMALL_ARMOR_POINTS,
    rvec,
    tvec,
    CAMERA_MATRIX,
    DISTORT_COEFFS,
    image_points);

  const Eigen::Vector2d result =
    omniperception::AngleSolver::solve(
      image_points,
      auto_aim::small,
      CAMERA_MATRIX,
      DISTORT_COEFFS);

  const double expected_yaw =
    -std::atan2(x, z) * RAD_TO_DEG;

  const double expected_pitch =
    std::atan2(
      y,
      std::sqrt(x * x + z * z)) * RAD_TO_DEG;

  std::cout << "\n" << name << "\n";

  std::cout
    << "expected yaw   = "
    << expected_yaw
    << " deg\n";

  std::cout
    << "solver yaw     = "
    << result[0]
    << " deg\n";

  std::cout
    << "expected pitch = "
    << expected_pitch
    << " deg\n";

  std::cout
    << "solver pitch   = "
    << result[1]
    << " deg\n";
}

void run_tilted_test()
{
  const double x = 0.5;
  const double y = -0.3;
  const double z = 3.0;

  // 原来的姿态：装甲板正对相机
  cv::Mat base_rotation =
    (cv::Mat_<double>(3, 3) <<
      0, -1,  0,
      0,  0, -1,
      1,  0,  0);

  // 在正对相机的基础上，再人为增加一个倾斜。
  // 这里只需要制造一个“不是正对相机”的情况即可。
  cv::Vec3d extra_rotation_vector{
    0.15,
    -0.25,
    0.10
  };

  cv::Mat extra_rotation;
  cv::Rodrigues(
    extra_rotation_vector,
    extra_rotation);

  // 组合两个旋转
  cv::Mat final_rotation =
    extra_rotation * base_rotation;

  cv::Vec3d rvec;
  cv::Rodrigues(
    final_rotation,
    rvec);

  // 装甲板中心的位置
  cv::Vec3d tvec{x, y, z};

  // 根据已知 3D 姿态生成模拟图像角点
  std::vector<cv::Point2f> image_points;

  cv::projectPoints(
    SMALL_ARMOR_POINTS,
    rvec,
    tvec,
    CAMERA_MATRIX,
    DISTORT_COEFFS,
    image_points);

  // 再把这些角点交给我们自己的 AngleSolver
  const Eigen::Vector2d result =
    omniperception::AngleSolver::solve(
      image_points,
      auto_aim::small,
      CAMERA_MATRIX,
      DISTORT_COEFFS);

  // 理论 yaw / pitch
  const double expected_yaw =
    -std::atan2(x, z) * RAD_TO_DEG;

  const double expected_pitch =
    std::atan2(
      y,
      std::sqrt(x * x + z * z))
    * RAD_TO_DEG;

  std::cout << "\nTILTED + OFFSET\n";

  std::cout
    << "expected yaw   = "
    << expected_yaw
    << " deg\n";

  std::cout
    << "solver yaw     = "
    << result[0]
    << " deg\n";

  std::cout
    << "expected pitch = "
    << expected_pitch
    << " deg\n";

  std::cout
    << "solver pitch   = "
    << result[1]
    << " deg\n";

  std::cout << "image points:\n";

  for (const auto & p : image_points) {
    std::cout
      << "  ("
      << p.x
      << ", "
      << p.y
      << ")\n";
  }
}

}  // namespace


int main()
{
  run_test("CENTER",  0.0,  0.0, 3.0);

  run_test("RIGHT",   0.5,  0.0, 3.0);

  run_test("LEFT",   -0.5,  0.0, 3.0);

  run_test("DOWN",    0.0,  0.3, 3.0);

  run_test("UP",      0.0, -0.3, 3.0);

  run_tilted_test();

  return 0;
}