//
//
// Copyright 2025 University of Washington

#pragma once

#include <stdint.h>

#include <string>

namespace vc_stereo_ros2 {

enum class TriggerType { Internal, External };

/// Very basic wrapper for interacting with the V4L2 ioctls for the cameras
///
///
class V4LDevice {
 public:
  explicit V4LDevice(const std::string &device);
  ~V4LDevice();

  bool setTrigger(const TriggerType trigger_type);

 protected:
  /// Query the list of ioctl ids to find the ids we use (trigger_mode, etc)
  ///
  bool initializeV4L2Ctrls();

  std::string device_;

  int trigger_mode_v4l2_id_;
};

}  // namespace vc_stereo_ros2
