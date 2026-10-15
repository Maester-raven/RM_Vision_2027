#ifndef OMNIPERCEPTION__DETECTION_HPP
#define OMNIPERCEPTION__DETECTION_HPP

#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace tools
{
class Recorder;
}

namespace omniperception
{

struct OmniCameraConfig
{
  io::Camera * camera;
  std::shared_ptr<tools::Recorder> recorder;
  double mount_yaw;    // degrees
  double mount_pitch;  // degrees
  double fov_h;        // degrees
  double fov_v;        // degrees

  // Calibrated angle solver.
  // Disabled by default to preserve the original FOV behaviour.
  bool use_calibrated_angle = false;
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
};

//一个识别结果可能包含多个armor,需要排序和过滤。armors, timestamp, delta_yaw, delta_pitch
struct DetectionResult
{
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;    //rad
  double delta_pitch;  //rad

  // Assignment operator
  DetectionResult & operator=(const DetectionResult & other)
  {
    if (this != &other) {
      armors = other.armors;
      timestamp = other.timestamp;
      delta_yaw = other.delta_yaw;
      delta_pitch = other.delta_pitch;
    }
    return *this;
  }
};
}  // namespace omniperception

#endif