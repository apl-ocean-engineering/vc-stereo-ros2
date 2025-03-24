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

namespace argus_stereo_sync {

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

struct GpioConfig {
  GpioConfig(const std::string &dev, int p) : device(dev), pin(p) {}

  std::string device;
  int pin;
};

class GpioTriggerThread : public ArgusSamples::Thread {
 public:
  explicit GpioTriggerThread(const std::shared_ptr<MutexCondition> &cond)
      : cond_(cond) {}

  ~GpioTriggerThread() {}

  bool initialize(const GpioConfig &config) {
    struct stat st;

    if (-1 == stat(config.device.c_str(), &st)) {
      std::cerr << "Cannot identify '" << config.device << "': " << errno
                << ", " << strerror(errno) << std::endl;
      return false;
    }

    if (!S_ISCHR(st.st_mode)) {
      std::cerr << config.device << " is not a char device" << std::endl;
      return false;
    }

    int gpio_fd = open(config.device.c_str(), O_RDWR | O_NONBLOCK, 0);

    if (-1 == gpio_fd) {
      std::cerr << "Cannot open '" << config.device << "': " << errno << ", "
                << strerror(errno) << std::endl;
      return false;
    }

    handle_.lineoffsets[0] = config.pin;
    handle_.lines = 1;
    handle_.flags = GPIOHANDLE_REQUEST_OUTPUT;

    int ret = ioctl(gpio_fd, GPIO_GET_LINEHANDLE_IOCTL, &handle_);

    if (ret == -1) {
      std::cerr << "Unable to get line handle from ioctl: " << strerror(errno)
                << std::endl;
      return false;
    }

    return true;
  }

  virtual bool threadInitialize() { return true; }

  virtual bool threadExecute() {
    cond_->wait();
    setGpio(true);
    usleep(1000);
    setGpio(false);

    return true;
  }

  virtual bool threadShutdown() { return true; }

  void setGpio(bool enable) {
    struct gpiohandle_data data;
    data.values[0] = (enable ? 1 : 0);

    int ret = ioctl(handle_.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret) {
      std::cerr << "Unable to trigger gpio: " << strerror(errno) << std::endl;
    }
  }

  struct gpiohandle_request handle_;
  std::shared_ptr<MutexCondition> cond_;
};

class GpioThreads : public ArgusSamples::Thread {
 public:
  explicit GpioThreads(uint32_t initial_period_ms = 0)
      : cond_(std::make_shared<MutexCondition>()) {}

  ~GpioThreads() {}

  // \todo Need to add code to _stop_ the timer if period is 0
  void setPeriodMs(uint32_t period_ms) {
    std::lock_guard<std::mutex> guard(timer_mutex_);
    std::cerr << "Configuring TriggerLoop with " << period_ms << " ms delay"
              << std::endl;

    struct itimerspec itval;

    int sec = period_ms / 1000;
    int ns = (period_ms % 1000) * 1000000;
    itval.it_interval.tv_sec = sec;
    itval.it_interval.tv_nsec = ns;
    itval.it_value.tv_sec = sec;
    itval.it_value.tv_nsec = ns;
    timerfd_settime(timer_, 0, &itval, NULL);
  }

 private:
  virtual bool threadInitialize() {
    std::array<GpioConfig, 2> gpio_devices = {GpioConfig("/dev/gpiochip", 49),
                                              GpioConfig("/dev/gpiochip", 138)};

    for (const auto gpio_device : gpio_devices) {
      auto gpio_trig = std::make_shared<GpioTriggerThread>(cond_);

      if (!gpio_trig->initialize(gpio_device)) {
        std::cerr << "Unable to configure GPIO for " << gpio_device.device
                  << " line " << gpio_device.pin << std::endl;
        continue;
      }

      gpio_threads_.push_back(gpio_trig);
    }

    return true;
  }

  virtual bool threadExecute() {
    /* Wait */
    fd_set rfds;
    int retval;

    /* Watch timefd file descriptor */
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    FD_SET(timer_, &rfds);

    {
      std::lock_guard<std::mutex> guard(timer_mutex_);
      retval = select(timer_ + 1, &rfds, NULL, NULL, NULL);
    }

    if (retval > 0) {
      cond_->notify_all();
    }

    // Essential to read from the timer before select()ing again?
    char buf[9];
    read(timer_, buf, 8);

    return true;
  }

  virtual bool threadShutdown() { return true; }

  std::vector<std::shared_ptr<GpioTriggerThread> > gpio_threads_;
  std::mutex timer_mutex_;
  int timer_;

  std::shared_ptr<MutexCondition> cond_;
};

}  // namespace argus_stereo_sync
