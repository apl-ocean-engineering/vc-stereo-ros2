//
//
// Copyright 2025 University of Washington

#pragma once

// clang-format off
// rclcpp must be included before anything that might include X11.h
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "argus_stereo_sync/camera_publisher.h"
// clang-format on

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <cudaEGL.h>

#include <memory>

#include "Thread.h"

namespace argus_stereo_sync {

class StereoConsumer : public ArgusSamples::Thread {
 public:
  explicit StereoConsumer(
      Argus::IEGLOutputStream *leftStream, Argus::IEGLOutputStream *rightStream,
      const std::shared_ptr<argus_stereo_sync::CameraPublisher> &left_pub,
      const std::shared_ptr<argus_stereo_sync::CameraPublisher> &right_pub);

  ~StereoConsumer();

 private:
  virtual bool threadInitialize();
  virtual bool threadExecute();
  virtual bool threadShutdown();

  Argus::IEGLOutputStream *m_leftStream;
  Argus::IEGLOutputStream *m_rightStream;
  CUeglStreamConnection m_cuStreamLeft;
  CUeglStreamConnection m_cuStreamRight;
  CUcontext m_cudaContext;

  std::shared_ptr<argus_stereo_sync::CameraPublisher> left_pub_;
  std::shared_ptr<argus_stereo_sync::CameraPublisher> right_pub_;
};

}  // namespace argus_stereo_sync
