//
//
// Copyright 2025 University of Washington

#pragma once

// clang-format off
// rclcpp.hpp must be included before anything that might call X11.h
#include <rclcpp/rclcpp.hpp>
// clang-format on

#include <Argus/Argus.h>
#include <cudaEGL.h>

#include <memory>
#include <sensor_msgs/msg/image.hpp>

#include "nvidia_multimedia_api/EGLGlobal.h"
#include "vc_argus_ros2/camera_publisher.h"

namespace vc_argus_ros2 {

class CudaFrameAcquire {
 public:
  CudaFrameAcquire(
      CUeglStreamConnection& connection,  // NOLINT [runtime/references]
      rclcpp::Logger logger, ArgusSamples::EGLDisplayHolder* display,
      Argus::IEGLOutputStream* egl_stream,
      const std::shared_ptr<vc_argus_ros2::CameraPublisher>& pub,
      const Argus::Size2D<uint32_t>& stream_size);

  ~CudaFrameAcquire();

  bool publish(const rclcpp::Time& now);

 private:
  CUeglStreamConnection& m_connection;
  CUgraphicsResource m_resource;
  CUeglFrame m_frame;
  CUstream m_stream;

  std::shared_ptr<vc_argus_ros2::CameraPublisher> pub_;

  int exposure_ns_;
  int analog_gain_;
  int isp_gain_;

  uint8_t* oBuffer_;
};

}  // namespace vc_argus_ros2
