//
//
// Copyright 2025 University of Washington

#pragma once

#include <condition_variable>
#include <mutex>

namespace vc_stereo_ros2 {

class MutexCondition {
 public:
  MutexCondition() {}

  void wait(void) {
    std::unique_lock<std::mutex> lk(_condMutex);
    _cond.wait(lk);
  }

  void notify_all() { _cond.notify_all(); }

  std::mutex _condMutex;
  std::condition_variable _cond;
};

}  // namespace vc_stereo_ros2
