//
//
// Copyright 2025 University of Washington

#pragma once

#include <stdint.h>

#include <string>

namespace argus_stereo_sync {

enum class TriggerType { Internal, External };

class V4LDevice {
 public:
  explicit V4LDevice(const std::string &device);
  ~V4LDevice();

  bool setTrigger(const TriggerType trigger_type);

 protected:
  bool initializeV4L2Ctrls();

  std::string device_;

  int trigger_mode_v4l2_id_;
};

}  // namespace argus_stereo_sync
