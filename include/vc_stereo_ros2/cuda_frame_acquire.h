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

#include <sensor_msgs/msg/image.hpp>

#include "vc_stereo_ros2/camera_publisher.h"

namespace vc_stereo_ros2 {

class CudaFrameAcquire {
 public:
  CudaFrameAcquire(CUeglStreamConnection& connection,
                   const std::shared_ptr<vc_stereo_ros2::CameraPublisher>& pub,
                   const Argus::Size2D<uint32_t>& stream_size);

  ~CudaFrameAcquire();

  bool publish(const rclcpp::Time& now);

 private:
  CUeglStreamConnection& m_connection;
  CUgraphicsResource m_resource;
  CUeglFrame m_frame;
  CUstream m_stream;

  std::shared_ptr<vc_stereo_ros2::CameraPublisher> pub_;

  uint8_t* oBuffer_;
};

}  // namespace vc_stereo_ros2
