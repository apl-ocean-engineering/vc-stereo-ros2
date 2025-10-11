//
//
// Copyright 2025 University of Washington

#pragma once

#include <stdint.h>

#include <string>

namespace vc_argus_ros2 {

enum class TriggerType {
  Internal,  // Internally trigger, free-running
  External   // Externally triggerd
};

/// Rudimentary wrapper for interacting with the V4L2 ioctls for the cameras
///
class V4LDevice {
 public:
  V4LDevice() = delete;
  V4LDevice(const V4LDevice &) = default;
  explicit V4LDevice(const std::string &devnode);

  ~V4LDevice() { ; }

  /// Set the trigger type
  ///
  /// \return True if successful
  bool setTrigger(const TriggerType trigger_type);

 protected:
  /// Query the list of ioctl ids to find the ids we use (trigger_mode, etc)
  ///
  /// \return True if able to initialize all control ID, false otherwise
  bool initializeV4L2CtrlIds();

 private:
  std::string device_;

  /// Cached IDs to ioctls of interest
  int trigger_mode_v4l2_id_;
};

}  // namespace vc_argus_ros2
