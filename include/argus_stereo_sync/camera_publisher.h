//
//
// Copyright 2025 University of Washington

#pragma once

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace argus_stereo_sync {

class CameraPublisher {
 public:
  CameraPublisher(
      std::shared_ptr<image_transport::ImageTransport> image_transport,
      const std::string &name)
      : publisher_(image_transport->advertiseCamera(name + "/image_raw", 1)) {}

  ~CameraPublisher() {}

  void setCameraInfo(const sensor_msgs::msg::CameraInfo &info) {
    camera_info_ = info;
  }

  void publish(const sensor_msgs::msg::Image &image) {
    publisher_.publish(image, camera_info_);
  }

 private:
  image_transport::CameraPublisher publisher_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace argus_stereo_sync
