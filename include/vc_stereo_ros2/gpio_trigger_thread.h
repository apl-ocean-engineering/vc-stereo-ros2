//
//
// Copyright 2025 University of Washington

#pragma once

#include <linux/gpio.h>

#include <memory>
#include <string>
#include <vector>

#include "nvidia_multimedia_api/Thread.h"
#include "vc_stereo_ros2/mutex_condition.h"

namespace vc_stereo_ros2 {

struct GpioConfig {
  //  GpioConfig(const std::string &dev, int p) : device(dev), pin(p) {}

  std::string device;
  int pin;
};

class GpioTriggerThread : public ArgusSamples::Thread {
 public:
  explicit GpioTriggerThread(const std::shared_ptr<MutexCondition> &cond,
                             int trigger_pulse_width_us = 2000);
  ~GpioTriggerThread() { ; }

  bool register_gpio(const GpioConfig &config);

 private:
  bool threadInitialize() override { return true; }
  bool threadExecute() override;
  bool threadShutdown() override { return true; }

  void setGpio(bool enable);

  struct gpiohandle_request handle_;
  std::shared_ptr<MutexCondition> cond_;

  int trigger_pulse_width_us_;
};

class GpioThreads : public ArgusSamples::Thread {
 public:
  explicit GpioThreads(const std::vector<GpioConfig> &configs);
  ~GpioThreads();

  // \todo Need to add code to _stop_ the timer if period is set to 0
  void setPeriodMs(uint32_t period_ms);
  void setTimer();

 private:
  bool threadInitialize() override;
  bool threadExecute() override;
  bool threadShutdown() override { return true; }

  const std::vector<GpioConfig> gpio_configs_;
  std::vector<std::shared_ptr<GpioTriggerThread> > gpio_threads_;
  std::mutex timer_mutex_;
  int timer_;
  uint32_t period_ms_;

  std::shared_ptr<MutexCondition> cond_;
};

}  // namespace vc_stereo_ros2
