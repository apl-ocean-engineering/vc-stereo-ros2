//
//
// Copyright 2025 University of Washington

#pragma once

#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "imaging_msgs/msg/imaging_metadata.hpp"

namespace vc_argus_ros2 {

// Extends the idea of image_transport::CameraPublisher,
// publishing an image, a cached camera_info, and imaging_metadata
// with a single call
class CameraPublisher {
 public:
  CameraPublisher(
      const std::string &name, image_transport::CameraPublisher camera_pub,
      const rclcpp::Publisher<imaging_msgs::msg::ImagingMetadata>::SharedPtr
          &metadata_pub)
      : publisher_(camera_pub), metadata_pub_(metadata_pub) {}

  ~CameraPublisher() {}

  void setCameraInfo(const sensor_msgs::msg::CameraInfo &info) {
    camera_info_ = info;
  }

  void publish(const sensor_msgs::msg::Image &image,
               const imaging_msgs::msg::ImagingMetadata &metadata) {
    camera_info_.header.stamp = image.header.stamp;
    camera_info_.header.frame_id = image.header.frame_id;
    publisher_.publish(image, camera_info_);

    metadata_pub_->publish(metadata);
  }

 private:
  image_transport::CameraPublisher publisher_;
  const rclcpp::Publisher<imaging_msgs::msg::ImagingMetadata>::SharedPtr
      metadata_pub_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace vc_argus_ros2
