//
//
// Copyright 2025 University of Washington

#pragma once

// clang-format off
// rclcpp must be included before anything that might include X11.h
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vc_stereo_ros2/camera_publisher.h"
// clang-format on

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <cudaEGL.h>

#include <memory>

#include "Thread.h"

namespace vc_stereo_ros2 {

class StereoConsumer : public ArgusSamples::Thread {
 public:
  explicit StereoConsumer(
      rclcpp::Logger logger, Argus::IEGLOutputStream *leftStream,
      Argus::IEGLOutputStream *rightStream,
      const std::shared_ptr<vc_stereo_ros2::CameraPublisher> &left_pub,
      const std::shared_ptr<vc_stereo_ros2::CameraPublisher> &right_pub);

  ~StereoConsumer();

 private:
  virtual bool threadInitialize();
  virtual bool threadExecute();
  virtual bool threadShutdown();

  rclcpp::Logger logger_;
  Argus::IEGLOutputStream *m_leftStream;
  Argus::IEGLOutputStream *m_rightStream;
  CUeglStreamConnection m_cuStreamLeft;
  CUeglStreamConnection m_cuStreamRight;
  CUcontext m_cudaContext;

  std::shared_ptr<vc_stereo_ros2::CameraPublisher> left_pub_;
  std::shared_ptr<vc_stereo_ros2::CameraPublisher> right_pub_;
};

}  // namespace vc_stereo_ros2
