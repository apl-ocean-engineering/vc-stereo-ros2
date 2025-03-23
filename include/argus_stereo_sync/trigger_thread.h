//
//
// Copyright 2025 University of Washington

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace argus_stereo_sync {

class TriggerThread : public ArgusSamples::Thread {
 public:
  explicit TriggerThread()

      ~TriggerThread();

 private:
};

}  // namespace argus_stereo_sync
