//
//
// Copyright 2025 University of Washington

// clang-format off
// Must come first, do not let this be sorted with the other headers
#include "vc_stereo_ros2/consumer_thread.h"
// clang-format on

#include <Argus/Argus.h>
#include <EGLStream/ArgusCaptureMetadata.h>

#include <memory>

#include "nvidia_multimedia_api/CUDAHelper.h"
#include "nvidia_multimedia_api/EGLGlobal.h"
#include "nvidia_multimedia_api/Error.h"
#include "vc_stereo_ros2/cuda_frame_acquire.h"

namespace vc_stereo_ros2 {

using ArgusSamples::getCudaErrorString;

ConsumerThread::ConsumerThread(
    rclcpp::Logger logger, const std::shared_ptr<rclcpp::Clock> &clock,
    const Argus::Size2D<uint32_t> &stream_size,
    ArgusSamples::EGLDisplayHolder *holder,
    Argus::IEGLOutputStream *output_stream,
    const std::shared_ptr<vc_stereo_ros2::CameraPublisher> &camera_pub)
    : logger_(logger),
      clock_(clock),
      stream_size_(stream_size),
      display_(holder),
      output_stream_(output_stream),
      cu_stream_(nullptr),
      cuda_context_(0),
      camera_pub_(camera_pub) {}

ConsumerThread::~ConsumerThread() {}

bool ConsumerThread::threadInitialize() {
  PROPAGATE_ERROR(ArgusSamples::initCUDA(&cuda_context_));

  RCLCPP_INFO(logger_, "Connecting CUDA consumer to stream");
  CUresult cuResult =
      cuEGLStreamConsumerConnect(&cu_stream_, output_stream_->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)",
                    getCudaErrorString(cuResult));
  }

  return true;
}

bool ConsumerThread::threadExecute() {
  RCLCPP_INFO(logger_, "Waiting for Argus producer to connect to left stream.");
  output_stream_->waitUntilConnected();

  RCLCPP_INFO(logger_, "Streams connected, processing frames.");
  while (true) {
    EGLint streamState = EGL_STREAM_STATE_CONNECTING_KHR;

    if (!ArgusSamples::eglQueryStreamKHR(output_stream_->getEGLDisplay(),
                                         output_stream_->getEGLStream(),
                                         EGL_STREAM_STATE_KHR, &streamState) ||
        (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      RCLCPP_WARN(logger_, " : EGL_STREAM_STATE_DISCONNECTED_KHR received");
      break;
    }

    CudaFrameAcquire frame_acq(cu_stream_, logger_, display_, output_stream_,
                               camera_pub_, stream_size_);

    const auto t = clock_->now();
    PROPAGATE_ERROR(frame_acq.publish(t));
  }

  RCLCPP_INFO(logger_, "No more frames. Cleaning up.");
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool ConsumerThread::threadShutdown() {
  CUresult cuResult = cuEGLStreamConsumerDisconnect(&cu_stream_);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)",
                    getCudaErrorString(cuResult));
  }

  PROPAGATE_ERROR(ArgusSamples::cleanupCUDA(&cuda_context_));
  RCLCPP_DEBUG(logger_, "Done.");
  return true;
}

}  // namespace vc_stereo_ros2
