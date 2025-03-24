//
//
// Copyright 2025 University of Washington

#pragma once

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Thread.h"
#include "argus_stereo_sync/mutex_condition.h"

namespace argus_stereo_sync {

struct GpioConfig {
  GpioConfig(const std::string &dev, int p) : device(dev), pin(p) {}

  std::string device;
  int pin;
};

class GpioTriggerThread : public ArgusSamples::Thread {
 public:
  explicit GpioTriggerThread(const std::shared_ptr<MutexCondition> &cond);

  ~GpioTriggerThread();

  bool configure(const GpioConfig &config);

 private:
  bool threadInitialize() override { return true; }
  bool threadExecute() override;
  bool threadShutdown() override { return true; }

  void setGpio(bool enable);

  struct gpiohandle_request handle_;
  std::shared_ptr<MutexCondition> cond_;
};

class GpioThreads : public ArgusSamples::Thread {
 public:
  explicit GpioThreads(uint32_t initial_period_ms = 0);
  ~GpioThreads();

  // \todo Need to add code to _stop_ the timer if period is 0
  void setPeriodMs(uint32_t period_ms);
  void setTimer();

 private:
  bool threadInitialize() override;
  bool threadExecute() override;
  bool threadShutdown() override { return true; }

  std::vector<std::shared_ptr<GpioTriggerThread> > gpio_threads_;
  std::mutex timer_mutex_;
  int timer_;
  uint32_t period_ms_;

  std::shared_ptr<MutexCondition> cond_;
};

}  // namespace argus_stereo_sync
