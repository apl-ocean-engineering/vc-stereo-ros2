//
//
// Copyright 2025 University of Washington

// clang-format off
// rclcpp must be included before anything that might include X11.h
#include "rclcpp/rclcpp.hpp"
// clang-format on

#include <Argus/Argus.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <imaging_msgs/msg/imaging_metadata.hpp>
#include <memory>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "nvidia_multimedia_api/ArgusHelpers.h"
#include "nvidia_multimedia_api/EGLGlobal.h"
#include "vc_argus_ros2/argus_parameters.hpp"
#include "vc_argus_ros2/camera_publisher.h"
#include "vc_argus_ros2/consumer_thread.h"
#include "vc_argus_ros2/gpio_trigger_thread.h"
#include "vc_argus_ros2/v4l_device.h"

namespace vc_argus_ros2 {

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

class ArgusCameraNode : public rclcpp::Node {
 public:
  explicit ArgusCameraNode(const rclcpp::NodeOptions &options)
      : Node("argus_camera", options),
        display_holder_(true),
        left_info_manager_(this, "left"),
        right_info_manager_(this, "right"),
        camera_provider_(CameraProvider::create()) {
    v4l_devices_ = {V4LDevice("/dev/video0"), V4LDevice("/dev/video1")};
    execute();
  }

  virtual ~ArgusCameraNode() {
    RCLCPP_INFO(get_logger(), "Starting destructor");

    auto icapturesession =
        Argus::interface_cast<ICaptureSession>(capture_session_);
    if (icapturesession) {
      icapturesession->stopRepeat();
      icapturesession->waitForIdle();
    }

    RCLCPP_INFO(get_logger(), "Captures complete, disconnecting producer.");
    for (auto &stream : streams_) {
      auto istream = Argus::interface_cast<IEGLOutputStream>(stream);
      istream->disconnect();
    }

    for (auto consumer : consumers_) {
      if (consumer) {
        if (!consumer->shutdown()) {
          RCLCPP_ERROR(get_logger(), "Unable to shut down consumer.");
        }
        consumer.reset();
      }
    }

    if (camera_provider_) {
      camera_provider_.reset();
    }

    if (!display_holder_.cleanup()) {
      RCLCPP_ERROR(get_logger(), "Unable to cleanup EGL display_holder");
    }
  }

  bool execute() {
    // In this version of the code, this is known apriori for the Nano dev board
    gpio_threads_ = std::make_shared<GpioThreads>(std::vector<GpioConfig>(
        {{"/dev/gpiochip0", 49}, {"/dev/gpiochip0", 138}}));

    param_listener_ =
        std::make_shared<ParamListener>(get_node_parameters_interface());

    auto params = param_listener_->get_params();

    if (!display_holder_.initialize()) {
      RCLCPP_FATAL(get_logger(), "Unable to initialized EGL Display Holder");
      return false;
    }

    ICameraProvider *iCameraProvider =
        Argus::interface_cast<ICameraProvider>(camera_provider_);
    if (!iCameraProvider) {
      RCLCPP_FATAL(get_logger(), "Failed to get ICameraProvider interface");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Argus Version: %s",
                iCameraProvider->getVersion().c_str());

    std::vector<CameraDevice *> camera_devices;
    iCameraProvider->getCameraDevices(&camera_devices);
    RCLCPP_INFO(get_logger(), "FOUND %lu CAMERAS", camera_devices.size());
    for (const auto &cam : camera_devices) {
      ArgusSamples::ArgusHelpers::printCameraDeviceInfo(cam, "  ");
    }

    // !! These are the core configurables
    std::vector<int> cameras_to_use = {0, 1};
    std::vector<std::string> camera_names = {"left", "right"};

    if (camera_devices.size() < cameras_to_use.size()) {
      RCLCPP_FATAL(get_logger(), "Must have at least %lu sensors available",
                   cameras_to_use.size());
      return false;
    } else if (cameras_to_use.size() == 0) {
      RCLCPP_FATAL(get_logger(), "No cameras specified");
      return false;
    }

    std::vector<CameraDevice *> lrCameras;
    for (auto i : cameras_to_use) {
      if ((i >= 0) && (i < camera_devices.size()) && camera_devices[i]) {
        lrCameras.push_back(camera_devices[i]);
      }
    }

    if (lrCameras.size() != cameras_to_use.size()) {
      RCLCPP_FATAL(get_logger(), "Couldn't find all of the cameras");
      return false;
    }

    // TODO:  Allow selecting for mode
    const uint32_t which_mode = 0;
    auto sensor_mode = ArgusSamples::ArgusHelpers::getSensorMode(
        lrCameras.front(), which_mode);
    if (!sensor_mode) {
      RCLCPP_FATAL(get_logger(), "Couldn't get sensor mode");
      return false;
    }
    auto imode = Argus::interface_cast<Argus::ISensorMode>(sensor_mode);
    if (!imode) {
      RCLCPP_FATAL(get_logger(), "Unable to cast sensor mode");
      return false;
    }

    stream_size_ = imode->getResolution();
    RCLCPP_INFO(get_logger(), "Using resolution %d x %d", stream_size_[0],
                stream_size_[1]);

    //== Set up publishers

    const auto framerate = params.framerate;
    RCLCPP_INFO_STREAM(get_logger(),
                       "Setting frame rate to " << framerate << " fps");
    setTriggering(params.trigger_mode, params.framerate);

    for (const auto &name : camera_names) {
      camera_pubs_.push_back(std::make_shared<vc_argus_ros2::CameraPublisher>(
          name,
          image_transport::create_camera_publisher(this, name + "/image_raw"),
          this->create_publisher<imaging_msgs::msg::ImagingMetadata>(
              name + "/imaging_metadata", 1)));
    }

    // Set up camera info for both cameras
    if (params.left_camera_info.size() > 0) {
      if (!left_info_manager_.loadCameraInfo(params.left_camera_info)) {
        RCLCPP_FATAL_STREAM(get_logger(),
                            "Unable to load camera LEFT info from \""
                                << params.left_camera_info << "\"");
        return false;
      }

      // TODO this is one of the places where the system breaks down
      camera_pubs_[0]->setCameraInfo(left_info_manager_.getCameraInfo());
    }

    if (params.right_camera_info.size() > 0) {
      if (!right_info_manager_.loadCameraInfo(params.right_camera_info)) {
        RCLCPP_FATAL_STREAM(get_logger(),
                            "Unable to load RIGHT camera info from \""
                                << params.right_camera_info << "\"");
        return false;
      }
      camera_pubs_[1]->setCameraInfo(right_info_manager_.getCameraInfo());
    }

    //== Set up cameras
    capture_session_.reset(iCameraProvider->createCaptureSession(lrCameras));
    auto icapturesession =
        Argus::interface_cast<ICaptureSession>(capture_session_);
    if (!icapturesession) {
      RCLCPP_FATAL(get_logger(), "Failed to get capture session interface");
      return false;
    }

    UniqueObj<OutputStreamSettings> streamSettings(
        icapturesession->createOutputStreamSettings(Argus::STREAM_TYPE_EGL));

    IOutputStreamSettings *iStreamSettings =
        Argus::interface_cast<IOutputStreamSettings>(streamSettings);

    {
      IEGLOutputStreamSettings *iEGLStreamSettings =
          Argus::interface_cast<IEGLOutputStreamSettings>(streamSettings);
      if (!iStreamSettings || !iEGLStreamSettings) {
        RCLCPP_FATAL(get_logger(), "Failed to create OutputStreamSettings");
        return false;
      }
      iEGLStreamSettings->setPixelFormat(Argus::PIXEL_FMT_YCbCr_420_888);
      iEGLStreamSettings->setResolution(stream_size_);
      iEGLStreamSettings->setEGLDisplay(display_holder_.get());
      iEGLStreamSettings->setMode(Argus::EGL_STREAM_MODE_MAILBOX);
      iEGLStreamSettings->setMetadataEnable(true);
    }

    // TODO This has bad smells...
    streams_.resize(lrCameras.size());
    for (int i = 0; i < lrCameras.size(); i++) {
      iStreamSettings->setCameraDevice(lrCameras[i]);
      auto stream = icapturesession->createOutputStream(streamSettings.get());
      if (!stream) {
        RCLCPP_FATAL(get_logger(),
                     "Unable to create output stream for camera %d", i);
        return false;
      }
      streams_[i].reset(stream);
    }

    request_.reset(icapturesession->createRequest());
    IRequest *irequest = Argus::interface_cast<IRequest>(request_);
    if (!irequest) {
      RCLCPP_FATAL(get_logger(), "Failed to create Request");
      return false;
    }

    for (auto &stream : streams_) {
      irequest->enableOutputStream(stream.get());
    }

    {
      ISourceSettings *iSourceSettings =
          Argus::interface_cast<ISourceSettings>(request_);
      if (!iSourceSettings) {
        RCLCPP_FATAL(get_logger(),
                     "Failed to get source settings request_ interface");
        return false;
      }
      iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / framerate));

      // const Argus::Range<float> gain_range(1, 48);
      iSourceSettings->setGainRange(imode->getAnalogGainRange());

      const uint64_t exp_ns = params.max_exposure_ms * 1e6;
      RCLCPP_INFO_STREAM(get_logger(),
                         "Setting max allowed exposure to " << exp_ns << " ns");
      const Argus::Range<uint64_t> exposure_time_range(
          imode->getExposureTimeRange().min(), exp_ns);
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
          Argus::interface_cast<IDenoiseSettings>(request_);
      denoiseSettings->setDenoiseMode(Argus::DENOISE_MODE_OFF);
    }

    {
      // Set Edge Enhancement settings
      //
      // Valid values: EDGE_ENHANCE_MODE_OFF, EDGE_ENHANCE_MODE_FAST,
      // EDGE_ENHANCE_MODE_HIGH_QUALITY also ->setEdgeEnhanceStrength(x) for 0
      // <= x <= 1
      IEdgeEnhanceSettings *edgeEnhanceSettings =
          Argus::interface_cast<IEdgeEnhanceSettings>(request_);
      edgeEnhanceSettings->setEdgeEnhanceMode(Argus::EDGE_ENHANCE_MODE_OFF);
    }

    {
      // Set auto-* settings
      // Intentionally include most of the settings, just so we know they're
      // available even if we're using the default
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

    consumers_.push_back(std::make_shared<ConsumerThread>(
        this->get_logger(), this->get_clock(), stream_size_, &display_holder_,
        Argus::interface_cast<IEGLOutputStream>(streams_[0]), camera_pubs_[0]));

    consumers_.push_back(std::make_shared<ConsumerThread>(
        this->get_logger(), this->get_clock(), stream_size_, &display_holder_,
        Argus::interface_cast<IEGLOutputStream>(streams_[1]), camera_pubs_[1]));

    gpio_threads_->initialize();
    gpio_threads_->waitRunning();

    for (auto consumer : consumers_) {
      if (!consumer->initialize()) {
        RCLCPP_FATAL(get_logger(), "Unable to initialize consumer");
      }

      if (!consumer->waitRunning()) {
        RCLCPP_FATAL(get_logger(), "Unable to start consumers");
      }
    }

    RCLCPP_INFO(get_logger(), "Starting repeat capture request_s.");
    if (icapturesession->repeat(request_.get()) != Argus::STATUS_OK) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to start repeat capture request_ for preview");
    }

    param_listener_->setUserCallback(std::bind(
        &ArgusCameraNode::parametersCallback, this, std::placeholders::_1));

    return true;
  }

  void parametersCallback(const vc_argus_ros2::Params &params) {
    // Pause streaming
    auto icapturesession =
        Argus::interface_cast<ICaptureSession>(capture_session_);
    if (!icapturesession) {
      RCLCPP_WARN(get_logger(), "Unable to get capturesession");
      return;
    }

    icapturesession->stopRepeat();

    // \todo{}
    //   Only set parameters if they've changed (?)

    {
      ISourceSettings *iSourceSettings =
          Argus::interface_cast<ISourceSettings>(request_);
      if (iSourceSettings) {
        const uint64_t exp_ns = params.max_exposure_ms * 1e6;
        RCLCPP_INFO_STREAM(get_logger(),
                           "Setting max exposure to " << exp_ns << " ns");
        const Argus::Range<uint64_t> exposure_time_range(44000, exp_ns);
        iSourceSettings->setExposureTimeRange(exposure_time_range);
      }
    }

    {
      IRequest *irequest = Argus::interface_cast<IRequest>(request_);
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
        }
      }
    }

    // Restart streaming in all cases
    if (icapturesession->repeat(request_.get()) != Argus::STATUS_OK) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start repeat capture request_ for preview");
    }
  }

 protected:
  void setTriggering(const std::string &trigger_mode, int framerate) {
    if (trigger_mode == "external") {
      RCLCPP_INFO(get_logger(), "Configuring cameras for _external_ trigger");

      for (auto &v4l_device : v4l_devices_) {
        v4l_device.setTrigger(TriggerType::External);
      }

      gpio_threads_->setPeriodMs(1000 / framerate);

    } else {
      RCLCPP_INFO(get_logger(), "Configuring cameras for _internal_ trigger");

      for (auto &v4l_device : v4l_devices_) {
        v4l_device.setTrigger(TriggerType::Internal);
      }
    }
  }

  Argus::Size2D<uint32_t> stream_size_;

  camera_info_manager::CameraInfoManager left_info_manager_,
      right_info_manager_;

  ArgusSamples::EGLDisplayHolder display_holder_;
  UniqueObj<CaptureSession> capture_session_;

  UniqueObj<CameraProvider> camera_provider_;
  UniqueObj<Request> request_;

  std::vector<std::shared_ptr<ConsumerThread> > consumers_;
  std::vector<UniqueObj<OutputStream> > streams_;

  std::vector<V4LDevice> v4l_devices_;

  std::vector<std::shared_ptr<vc_argus_ros2::CameraPublisher> > camera_pubs_;

  std::shared_ptr<GpioThreads> gpio_threads_;

  std::shared_ptr<vc_argus_ros2::ParamListener> param_listener_;
};

}  // namespace vc_argus_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vc_argus_ros2::ArgusCameraNode)
