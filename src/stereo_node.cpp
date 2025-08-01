//
//
// Copyright 2025 University of Washington

// clang-format off
// rclcpp must be included before anything that might include X11.h
#include "rclcpp/rclcpp.hpp"
// clang-format on

#include <Argus/Argus.h>

#include <memory>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <imaging_msgs/msg/imaging_metadata.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "nvidia_multimedia_api/EGLGlobal.h"
#include "nvidia_multimedia_api/Error.h"

#include "vc_stereo_ros2/camera_publisher.h"
#include "vc_stereo_ros2/gpio_trigger_thread.h"
#include "vc_stereo_ros2/stereo_consumer.h"
#include "vc_stereo_ros2/stereo_parameters.hpp"
#include "vc_stereo_ros2/v4l_device.h"

namespace vc_stereo_ros2 {

using Argus::CameraDevice;
using Argus::CameraProvider;
using Argus::CaptureSession;
using Argus::IAutoControlSettings;
using Argus::ICameraProvider;
using Argus::ICaptureSession;
using Argus::IDenoiseSettings;
using Argus::IEdgeEnhanceSettings;
using Argus::IEGLOutputStream;
using Argus::IEGLOutputStreamSettings;
using Argus::IOutputStreamSettings;
using Argus::IRequest;
using Argus::ISourceSettings;
using Argus::OutputStream;
using Argus::OutputStreamSettings;
using Argus::Range;
using Argus::Request;
using Argus::UniqueObj;

// Image size is currently fixed, because I'm lazy
static const Argus::Size2D<uint32_t> STREAM_SIZE(1440, 1080);

class SyncedStereoNode : public rclcpp::Node {
 public:
  SyncedStereoNode(const rclcpp::NodeOptions & options)
      : Node("synced_stereo", options),
        g_display(true),
        left_info_manager_(this, "left"),
        right_info_manager_(this, "right"),
        camera_provider_(CameraProvider::create()),
        video0_("/dev/video0"),
        video1_("/dev/video1") {
    param_listener_ =
        std::make_shared<ParamListener>(get_node_parameters_interface());

        execute();
  }

  virtual ~SyncedStereoNode() {
    RCLCPP_INFO(get_logger(), "Starting destructor");

    auto icapturesession =
        Argus::interface_cast<ICaptureSession>(capture_session_);
    if (icapturesession) {
      icapturesession->stopRepeat();
      icapturesession->waitForIdle();
    }

    RCLCPP_INFO(get_logger(), "Captures complete, disconnecting producer.");
    istream_left_->disconnect();
    istream_right_->disconnect();

    if (stereo_consumer_) {
      PROPAGATE_ERROR_CONTINUE(stereo_consumer_->shutdown());
      stereo_consumer_.reset();
    }

    if (camera_provider_) {
      camera_provider_.reset();
    }
    PROPAGATE_ERROR_CONTINUE(g_display.cleanup());

    RCLCPP_INFO(get_logger(), "Done -- exiting.");
  }

  bool execute() {
    
    auto params = param_listener_->get_params();

    int framerate = params.framerate;
    RCLCPP_INFO_STREAM(get_logger(),
                       "Setting frame rate to " << framerate << " fps");

    if (params.trigger_mode == "external") {
      RCLCPP_INFO(get_logger(), "Configuring cameras for _external_ trigger");

      video0_.setTrigger(TriggerType::External);
      video1_.setTrigger(TriggerType::External);

      gpio_threads_.setPeriodMs(1000 / framerate);

      RCLCPP_INFO(get_logger(), " ... configured");

    } else {
      RCLCPP_INFO(get_logger(), "Configuring cameras for _internal_ trigger");

      video0_.setTrigger(TriggerType::Internal);
      video1_.setTrigger(TriggerType::Internal);
    }

    left_camera_pub_ = std::make_shared<vc_stereo_ros2::CameraPublisher>(
        "left",
        image_transport::create_camera_publisher(this, "left/image_raw"),
        this->create_publisher<imaging_msgs::msg::ImagingMetadata>(
            "left/imaging_metadata", 1));
    right_camera_pub_ = std::make_shared<vc_stereo_ros2::CameraPublisher>(
        "right",
        image_transport::create_camera_publisher(this, "right/image_raw"),
        this->create_publisher<imaging_msgs::msg::ImagingMetadata>(
            "right/imaging_metadata", 1));

    // Set up camera info for both cameras
    if (params.left_camera_info.size() > 0) {
      if (!left_info_manager_.loadCameraInfo(params.left_camera_info)) {
        RCLCPP_FATAL_STREAM(get_logger(),
                            "Unable to load camera LEFT info from \""
                                << params.left_camera_info << "\"");
        return false;
      }
      left_camera_pub_->setCameraInfo(left_info_manager_.getCameraInfo());
    }

    if (params.right_camera_info.size() > 0) {
      if (!right_info_manager_.loadCameraInfo(params.right_camera_info)) {
        RCLCPP_FATAL_STREAM(get_logger(),
                            "Unable to load RIGHT camera info from \""
                                << params.right_camera_info << "\"");
        return false;
      }
      right_camera_pub_->setCameraInfo(right_info_manager_.getCameraInfo());
    }

    PROPAGATE_ERROR(g_display.initialize());

    ICameraProvider *iCameraProvider =
        Argus::interface_cast<ICameraProvider>(camera_provider_);
    if (!iCameraProvider) {
      ORIGINATE_ERROR("Failed to get ICameraProvider interface");
    }
    RCLCPP_INFO(get_logger(), "Argus Version: %s",
                iCameraProvider->getVersion().c_str());

    iCameraProvider->getCameraDevices(&camera_devices_);
    RCLCPP_INFO(get_logger(), "FOUND %lu CAMERAS", camera_devices_.size());
    if (camera_devices_.size() < 2) {
      ORIGINATE_ERROR("Must have at least 2 sensors available");
    }

    std::vector<CameraDevice *> lrCameras;
    lrCameras.push_back(camera_devices_[0]);
    lrCameras.push_back(camera_devices_[1]);

    capture_session_.reset(iCameraProvider->createCaptureSession(lrCameras));
    auto icapturesession =
        Argus::interface_cast<ICaptureSession>(capture_session_);
    if (!icapturesession) {
      ORIGINATE_ERROR("Failed to get capture session interface");
    }

    UniqueObj<OutputStreamSettings> streamSettings(
        icapturesession->createOutputStreamSettings(Argus::STREAM_TYPE_EGL));

    IOutputStreamSettings *iStreamSettings =
        Argus::interface_cast<IOutputStreamSettings>(streamSettings);

    {
      IEGLOutputStreamSettings *iEGLStreamSettings =
          Argus::interface_cast<IEGLOutputStreamSettings>(streamSettings);
      if (!iStreamSettings || !iEGLStreamSettings) {
        ORIGINATE_ERROR("Failed to create OutputStreamSettings");
      }
      iEGLStreamSettings->setPixelFormat(Argus::PIXEL_FMT_YCbCr_420_888);
      iEGLStreamSettings->setResolution(STREAM_SIZE);
      iEGLStreamSettings->setEGLDisplay(g_display.get());
      iEGLStreamSettings->setMode(Argus::EGL_STREAM_MODE_MAILBOX);
      iEGLStreamSettings->setMetadataEnable(true);
    }

    RCLCPP_INFO(get_logger(), "Creating left stream.");
    iStreamSettings->setCameraDevice(lrCameras[0]);
    stream_left_.reset(
        icapturesession->createOutputStream(streamSettings.get()));
    istream_left_ = Argus::interface_cast<IEGLOutputStream>(stream_left_);
    if (!istream_left_) {
      ORIGINATE_ERROR("Failed to create left stream");
    }

    RCLCPP_INFO(get_logger(), "Creating right stream.");
    iStreamSettings->setCameraDevice(lrCameras[1]);
    stream_right_.reset(
        icapturesession->createOutputStream(streamSettings.get()));
    istream_right_ = Argus::interface_cast<IEGLOutputStream>(stream_right_);
    if (!istream_right_) {
      ORIGINATE_ERROR("Failed to create right stream");
    }

    request.reset(icapturesession->createRequest());

    IRequest *irequest = Argus::interface_cast<IRequest>(request);
    if (!irequest) {
      ORIGINATE_ERROR("Failed to create Request");
    }

    irequest->enableOutputStream(stream_left_.get());
    irequest->enableOutputStream(stream_right_.get());

    {
      ISourceSettings *iSourceSettings =
          Argus::interface_cast<ISourceSettings>(request);
      if (!iSourceSettings) {
        ORIGINATE_ERROR("Failed to get source settings request interface");
      }
      iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / framerate));

      const Argus::Range<float> gain_range(1, 48);
      iSourceSettings->setGainRange(gain_range);

      const uint64_t exp_ns = params.max_exposure_ms * 1e6;
      RCLCPP_INFO_STREAM(get_logger(),
                         "Setting max exposure to " << exp_ns << " ns");
      const Argus::Range<uint64_t> exposure_time_range(44000, exp_ns);
      iSourceSettings->setExposureTimeRange(exposure_time_range);
    }

    {
      // Set Deonoise settings
      //
      // Valid values: DENOISE_MODE_OFF, EDGE_ENHANCE_MODE_FAST,
      // EDGE_ENHANCE_MODE_HIGH_QUALITY also ->setDenoiseStrength(x) for 0 <= x
      // <=
      // 1
      IDenoiseSettings *denoiseSettings =
          Argus::interface_cast<IDenoiseSettings>(request);
      denoiseSettings->setDenoiseMode(Argus::DENOISE_MODE_OFF);
    }

    {
      // Set Edge Enhancement settings
      //
      // Valid values: EDGE_ENHANCE_MODE_OFF, EDGE_ENHANCE_MODE_FAST,
      // EDGE_ENHANCE_MODE_HIGH_QUALITY also ->setEdgeEnhanceStrength(x) for 0
      // <= x <= 1
      IEdgeEnhanceSettings *edgeEnhanceSettings =
          Argus::interface_cast<IEdgeEnhanceSettings>(request);
      edgeEnhanceSettings->setEdgeEnhanceMode(Argus::EDGE_ENHANCE_MODE_OFF);
    }

    {
      // Set auto-* settings
      // Intentionally include most of the settings, just so we know they're
      // availables even if we're using the default
      IAutoControlSettings *iAutoControlSettings =
          Argus::interface_cast<IAutoControlSettings>(
              irequest->getAutoControlSettings());

      const Argus::Range<float> ISP_DIGITAL_GAIN_RANGE(1, 1);
      iAutoControlSettings->setIspDigitalGainRange(ISP_DIGITAL_GAIN_RANGE);

      iAutoControlSettings->setAeAntibandingMode(
          Argus::AE_ANTIBANDING_MODE_OFF);

      // Auto-exposure settings
      iAutoControlSettings->setAeMode(Argus::AE_MODE_ON);
      iAutoControlSettings->setAeLock(false);

      // Auto-whitebalance settings
      iAutoControlSettings->setAwbMode(Argus::AWB_MODE_AUTO);
      iAutoControlSettings->setAwbLock(params.awb_lock);

      // Should use ColorSaturation instead?
      iAutoControlSettings->setColorSaturationEnable(true);
      iAutoControlSettings->setColorSaturationBias(params.saturation);

      iAutoControlSettings->setExposureCompensation(
          params.exposure_compensation);
    }

    stereo_consumer_ = std::make_shared<StereoConsumer>(
        this->get_logger(), this->get_clock(), STREAM_SIZE, &g_display,
        istream_left_, istream_right_, left_camera_pub_, right_camera_pub_);

    gpio_threads_.initialize();
    gpio_threads_.waitRunning();

    PROPAGATE_ERROR(stereo_consumer_->initialize());
    PROPAGATE_ERROR(stereo_consumer_->waitRunning());

    RCLCPP_INFO(get_logger(), "Starting repeat capture requests.");
    if (icapturesession->repeat(request.get()) != Argus::STATUS_OK) {
      ORIGINATE_ERROR("Failed to start repeat capture request for preview");
    }

    // param_listener_ is also setting up a callback
    // are they conflicting?  why can't I get a callback from
    // it when there's a paramater change?
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &SyncedStereoNode::parametersCallback, this, std::placeholders::_1));

    return true;
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    param_listener_->update(parameters);
    auto params = param_listener_->get_params();

    // Pause streaming

    auto icapturesession =
        Argus::interface_cast<ICaptureSession>(capture_session_);
    if (!icapturesession) {
      result.successful = false;
      result.reason = "Could not get ICaptureSession";
      return result;
    }

    icapturesession->stopRepeat();

    // \todo{}
    //   Only set parameters if they've changed (?)

    {
      ISourceSettings *iSourceSettings =
          Argus::interface_cast<ISourceSettings>(request);
      if (iSourceSettings) {
        const uint64_t exp_ns = params.max_exposure_ms * 1e6;
        RCLCPP_INFO_STREAM(get_logger(),
                           "Setting max exposure to " << exp_ns << " ns");
        const Argus::Range<uint64_t> exposure_time_range(44000, exp_ns);
        iSourceSettings->setExposureTimeRange(exposure_time_range);
      } else {
        result.successful = false;
        result.reason = "Failed to get source settings request interface";
      }
    }

    {
      IRequest *irequest = Argus::interface_cast<IRequest>(request);
      if (irequest) {
        // Set auto-* settings
        // Intentionally include most of the settings, just so we know they're
        // availables even if we're using the default
        IAutoControlSettings *iAutoControlSettings =
            Argus::interface_cast<IAutoControlSettings>(
                irequest->getAutoControlSettings());

        if (iAutoControlSettings) {
          iAutoControlSettings->setAwbLock(params.awb_lock);

          iAutoControlSettings->setColorSaturationBias(params.saturation);

          iAutoControlSettings->setExposureCompensation(
              params.exposure_compensation);
        } else {
          result.successful = false;
          result.reason = "Failed to get IAutoControlSettings";
        }
      } else {
        result.successful = false;
        result.reason = "Failed to get IRequest";
      }
    }
    // Restart streaming in all cases
    if (icapturesession->repeat(request.get()) != Argus::STATUS_OK) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start repeat capture request for preview");
    }

    return result;
  }

 protected:
  camera_info_manager::CameraInfoManager left_info_manager_,
      right_info_manager_;

  ArgusSamples::EGLDisplayHolder g_display;
  UniqueObj<CaptureSession> capture_session_;
  std::vector<CameraDevice *> camera_devices_;

  UniqueObj<CameraProvider> camera_provider_;
  UniqueObj<Request> request;
  std::shared_ptr<StereoConsumer> stereo_consumer_;

  UniqueObj<OutputStream> stream_left_, stream_right_;
  IEGLOutputStream *istream_right_, *istream_left_;

  V4LDevice video0_, video1_;

  std::shared_ptr<vc_stereo_ros2::CameraPublisher> left_camera_pub_,
      right_camera_pub_;

  GpioThreads gpio_threads_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  std::shared_ptr<vc_stereo_ros2::ParamListener> param_listener_;
  vc_stereo_ros2::Params params_;
};

}  // namespace vc_stereo_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vc_stereo_ros2::SyncedStereoNode)