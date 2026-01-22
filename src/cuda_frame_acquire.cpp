
//
//
// Copyright 2025 University of Washington

#include "vc_argus_ros2/cuda_frame_acquire.h"

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/MetadataContainer.h>

#include <cstdio>
#include <memory>

#include "nvidia_multimedia_api/CUDAHelper.h"
#include "nvidia_multimedia_api/Error.h"
#include "sensor_msgs/fill_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vc_argus_ros2/convert.h"

namespace vc_argus_ros2 {

CudaFrameAcquire::CudaFrameAcquire(
    CUeglStreamConnection& connection, rclcpp::Logger logger,
    ArgusSamples::EGLDisplayHolder* display,
    Argus::IEGLOutputStream* egl_stream,
    const std::shared_ptr<vc_argus_ros2::CameraPublisher>& pub,
    const Argus::Size2D<uint32_t>& stream_size)
    : m_connection(connection),
      m_stream(NULL),
      m_resource(0),
      pub_(pub),
      exposure_ns_(0),
      analog_gain_(0),
      isp_gain_(0),
      oBuffer_(new uint8_t[3 * stream_size.width() * stream_size.height()]) {
  CUresult cuResult = cuEGLStreamConsumerAcquireFrame(
      &m_connection, &m_resource, &m_stream, -1);

  if (cuResult == CUDA_SUCCESS) {
    cuResult = cuGraphicsResourceGetMappedEglFrame(&m_frame, m_resource, 0, 0);
    if (cuResult != CUDA_SUCCESS) {
      printf("Unable to get mapped frame (%s)",
             ArgusSamples::getCudaErrorString(cuResult));
    }
  } else {
    printf("Unable to acquire frame (%s)",
           ArgusSamples::getCudaErrorString(cuResult));
  }

  Argus::Status status;
  Argus::UniqueObj<EGLStream::MetadataContainer> metadataContainer(
      EGLStream::MetadataContainer::create(
          display->get(), egl_stream->getEGLStream(),
          EGLStream::MetadataContainer::CONSUMER, &status));
  EGLStream::IArgusCaptureMetadata* iArgusCaptureMetadata =
      Argus::interface_cast<EGLStream::IArgusCaptureMetadata>(
          metadataContainer);
  if (iArgusCaptureMetadata) {
    auto leftMetadata = iArgusCaptureMetadata->getMetadata();
    auto ILeftMetadata =
        Argus::interface_cast<Argus::ICaptureMetadata>(leftMetadata);

    if (!ILeftMetadata) {
      RCLCPP_WARN(logger, "Cannot get metadata");
    } else {
      exposure_ns_ = ILeftMetadata->getSensorExposureTime();
      analog_gain_ = ILeftMetadata->getSensorAnalogGain();
      isp_gain_ = ILeftMetadata->getIspDigitalGain();
    }
  } else {
    RCLCPP_WARN_STREAM(logger, "Could not query metadata (" << status << ")");
  }
}

CudaFrameAcquire::~CudaFrameAcquire() {
  if (m_resource) {
    cuEGLStreamConsumerReleaseFrame(&m_connection, m_resource, &m_stream);
  }

  if (oBuffer_) delete[] oBuffer_;
}

bool CudaFrameAcquire::publish(const rclcpp::Time& now, double gamma) {
  CUDA_RESOURCE_DESC cudaResourceDesc;
  memset(&cudaResourceDesc, 0, sizeof(cudaResourceDesc));
  cudaResourceDesc.resType = CU_RESOURCE_TYPE_ARRAY;

  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[0];
  CUsurfObject cudaSurfObj1 = 0;
  CUresult cuResult = cuSurfObjectCreate(&cudaSurfObj1, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object 1 (%s)",
                    ArgusSamples::getCudaErrorString(cuResult));
  }

  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[1];
  CUsurfObject cudaSurfObj2 = 0;
  cuResult = cuSurfObjectCreate(&cudaSurfObj2, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object 2 (%s)",
                    ArgusSamples::getCudaErrorString(cuResult));
  }

  float delta = convertSurfObject(cudaSurfObj1, cudaSurfObj2, m_frame.width,
                                  m_frame.height, gamma, oBuffer_);
  cuSurfObjectDestroy(cudaSurfObj1);
  cuSurfObjectDestroy(cudaSurfObj2);

  sensor_msgs::msg::Image output;
  output.header.stamp = now;
  sensor_msgs::fillImage(output, sensor_msgs::image_encodings::BGR8,
                         m_frame.height, m_frame.width, 3 * m_frame.width,
                         reinterpret_cast<void*>(oBuffer_));

  imaging_msgs::msg::ImagingMetadata meta_msg;
  meta_msg.header.stamp = output.header.stamp;
  meta_msg.exposure_us = exposure_ns_ / 1e3;
  meta_msg.gain = analog_gain_;
  meta_msg.digital_gain = isp_gain_;

  pub_->publish(output, meta_msg);

  return true;
}

}  // namespace vc_argus_ros2
