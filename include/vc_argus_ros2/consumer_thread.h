//
//
// Copyright 2025 University of Washington

#pragma once

// clang-format off
// rclcpp must be included before anything that might include X11.h
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vc_argus_ros2/camera_publisher.h"
// clang-format on

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <cudaEGL.h>

#include <memory>

#include "nvidia_multimedia_api/EGLGlobal.h"
#include "nvidia_multimedia_api/Thread.h"

namespace vc_argus_ros2 {

class ConsumerThread : public ArgusSamples::Thread {
 public:
  explicit ConsumerThread(
      rclcpp::Logger logger, const std::shared_ptr<rclcpp::Clock> &clock,
      const Argus::Size2D<uint32_t> &stream_size,
      ArgusSamples::EGLDisplayHolder *holder,
      Argus::IEGLOutputStream *output_stream,
      const std::shared_ptr<vc_argus_ros2::CameraPublisher> &camera_pub);

  ~ConsumerThread();

  void setGamma(double g) { gamma_ = g; }

 private:
  virtual bool threadInitialize();
  virtual bool threadExecute();
  virtual bool threadShutdown();

  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Clock> clock_;
  Argus::Size2D<uint32_t> stream_size_;

  ArgusSamples::EGLDisplayHolder *display_;
  Argus::IEGLOutputStream *output_stream_;
  CUeglStreamConnection cu_stream_;
  CUcontext cuda_context_;

  double gamma_;

  std::shared_ptr<vc_argus_ros2::CameraPublisher> camera_pub_;
};

}  // namespace vc_argus_ros2
