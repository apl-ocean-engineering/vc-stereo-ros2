//
//
// Copyright 2025 University of Washington

#pragma once

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "imaging_msgs/msg/imaging_metadata.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace vc_stereo_ros2 {

class CameraPublisher {
 public:
  CameraPublisher(
      const std::string &name,
      image_transport::CameraPublisher camera_pub,
      const rclcpp::Publisher<imaging_msgs::msg::ImagingMetadata>::SharedPtr
          &metadata_pub)
      : publisher_(camera_pub),
        metadata_pub_(metadata_pub) {}

  ~CameraPublisher() {}

  void setCameraInfo(const sensor_msgs::msg::CameraInfo &info) {
    camera_info_ = info;
  }

  void publish(const sensor_msgs::msg::Image &image) {
    camera_info_.header.stamp = image.header.stamp;
    camera_info_.header.frame_id = image.header.frame_id;
    publisher_.publish(image, camera_info_);
  }

  void publish_metadata(const imaging_msgs::msg::ImagingMetadata &metadata) {
    metadata_pub_->publish(metadata);
  }

 private:
  image_transport::CameraPublisher publisher_;
  const rclcpp::Publisher<imaging_msgs::msg::ImagingMetadata>::SharedPtr
      metadata_pub_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace vc_stereo_ros2
