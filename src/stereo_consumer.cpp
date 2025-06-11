//
//
// Copyright 2025 University of Washington

// clang-format off
// Must come first, do not let this be sorted with the other headers
#include "vc_stereo_ros2/stereo_consumer.h"
// clang-format on

#include <Argus/Argus.h>
#include <EGLStream/ArgusCaptureMetadata.h>

#include <memory>

#include "nvidia_multimedia_api/CUDAHelper.h"
#include "nvidia_multimedia_api/EGLGlobal.h"
#include "nvidia_multimedia_api/Error.h"
#include "vc_stereo_ros2/cuda_frame_acquire.h"

#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

namespace vc_stereo_ros2 {

using ArgusSamples::getCudaErrorString;

StereoConsumer::StereoConsumer(
    rclcpp::Logger logger, const std::shared_ptr<rclcpp::Clock> &clock,
    const Argus::Size2D<uint32_t> &stream_size,
    ArgusSamples::EGLDisplayHolder *holder, Argus::IEGLOutputStream *leftStream,
    Argus::IEGLOutputStream *rightStream,
    const std::shared_ptr<vc_stereo_ros2::CameraPublisher> &left_pub,
    const std::shared_ptr<vc_stereo_ros2::CameraPublisher> &right_pub)
    : logger_(logger),
      clock_(clock),
      stream_size_(stream_size),
      display_(holder),
      m_leftStream(leftStream),
      m_rightStream(rightStream),
      m_cuStreamLeft(nullptr),
      m_cuStreamRight(nullptr),
      m_cudaContext(0),
      left_pub_(left_pub),
      right_pub_(right_pub) {}

StereoConsumer::~StereoConsumer() {}

bool StereoConsumer::threadInitialize() {
  PROPAGATE_ERROR(ArgusSamples::initCUDA(&m_cudaContext));

  RCLCPP_INFO(logger_, "Connecting CUDA consumer to left stream");
  CUresult cuResult =
      cuEGLStreamConsumerConnect(&m_cuStreamLeft, m_leftStream->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)",
                    getCudaErrorString(cuResult));
  }

  RCLCPP_INFO(logger_, "Connecting CUDA consumer to right stream");
  cuResult = cuEGLStreamConsumerConnect(&m_cuStreamRight,
                                        m_rightStream->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)",
                    getCudaErrorString(cuResult));
  }
  return true;
}

bool StereoConsumer::threadExecute() {
  RCLCPP_INFO(logger_, "Waiting for Argus producer to connect to left stream.");
  m_leftStream->waitUntilConnected();

  RCLCPP_INFO(logger_,
              "Waiting for Argus producer to connect to right stream.");
  m_rightStream->waitUntilConnected();

  RCLCPP_INFO(logger_, "Streams connected, processing frames.");
  while (true) {
    EGLint streamState = EGL_STREAM_STATE_CONNECTING_KHR;

    if (!ArgusSamples::eglQueryStreamKHR(m_leftStream->getEGLDisplay(),
                                         m_leftStream->getEGLStream(),
                                         EGL_STREAM_STATE_KHR, &streamState) ||
        (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      RCLCPP_WARN(logger_, "left : EGL_STREAM_STATE_DISCONNECTED_KHR received");
      break;
    }

    if (!ArgusSamples::eglQueryStreamKHR(m_rightStream->getEGLDisplay(),
                                         m_rightStream->getEGLStream(),
                                         EGL_STREAM_STATE_KHR, &streamState) ||
        (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      RCLCPP_WARN(logger_,
                  "right : EGL_STREAM_STATE_DISCONNECTED_KHR received");
      break;
    }

    CudaFrameAcquire left_acq(m_cuStreamLeft, logger_, display_, m_leftStream,
                              left_pub_, stream_size_);
    CudaFrameAcquire right_acq(m_cuStreamRight, logger_, display_,
                               m_rightStream, right_pub_, stream_size_);

    const auto t = clock_->now();
    PROPAGATE_ERROR(left_acq.publish(t) && right_acq.publish(t));
  }

  RCLCPP_INFO(logger_, "No more frames. Cleaning up.");
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool StereoConsumer::threadShutdown() {
  CUresult cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamLeft);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)",
                    getCudaErrorString(cuResult));
  }

  cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamRight);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)",
                    ArgusSamples::getCudaErrorString(cuResult));
  }

  PROPAGATE_ERROR(ArgusSamples::cleanupCUDA(&m_cudaContext));
  RCLCPP_DEBUG(logger_, "Done.");
  return true;
}

}  // namespace vc_stereo_ros2
