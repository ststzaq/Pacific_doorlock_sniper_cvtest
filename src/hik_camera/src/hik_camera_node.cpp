#include "GxIAPI.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode with Daheng SDK backend!");

    GX_STATUS status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
      throw std::runtime_error("GXInitLib failed");
    }
    sdk_initialized_ = true;

    uint32_t device_count = 0;
    status = GXUpdateDeviceList(&device_count, 1000);
    while (device_count == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No Daheng camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", status);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      status = GXUpdateDeviceList(&device_count, 1000);
    }

    GX_OPEN_PARAM open_param{};
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    int camera_index = this->declare_parameter("camera_index", 1);
    if (camera_index < 1) {
      RCLCPP_WARN(this->get_logger(), "Invalid camera_index=%d, fallback to 1", camera_index);
      camera_index = 1;
    }
    camera_index_content_ = std::to_string(camera_index);
    open_param.pszContent = camera_index_content_.data();

    status = GXOpenDevice(&open_param, &camera_handle_);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
      throw std::runtime_error("GXOpenDevice failed");
    }

    int64_t img_width_max = 0;
    int64_t img_height_max = 0;
    int64_t payload_size = 0;
    GXGetInt(camera_handle_, GX_INT_WIDTH, &img_width_max);
    GXGetInt(camera_handle_, GX_INT_HEIGHT, &img_height_max);
    GXGetInt(camera_handle_, GX_INT_PAYLOAD_SIZE, &payload_size);

    RCLCPP_INFO(
      this->get_logger(), "Image size: ( %ld x %ld ), payload: %ld", img_width_max, img_height_max,
      payload_size);

    image_msg_.data.resize(static_cast<size_t>(img_width_max * img_height_max * 3));
    raw_frame_buffer_.resize(static_cast<size_t>(payload_size));

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    status = GXStreamOn(camera_handle_);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
      throw std::runtime_error("GXStreamOn failed");
    }

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_6mm_MV-CS016-10UC.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "bgr8";

      while (rclcpp::ok() && running_) {
        GX_FRAME_DATA frame_data{};
        frame_data.pImgBuf = raw_frame_buffer_.data();
        frame_data.nImgSize = static_cast<int64_t>(raw_frame_buffer_.size());

        GX_STATUS frame_status = GXGetImage(camera_handle_, &frame_data, 1000);
        if (frame_status == GX_STATUS_SUCCESS && frame_data.nStatus == GX_FRAME_STATUS_SUCCESS) {
          image_msg_.header.stamp = this->now();
          image_msg_.width = static_cast<uint32_t>(frame_data.nWidth);
          image_msg_.height = static_cast<uint32_t>(frame_data.nHeight);
          image_msg_.step = image_msg_.width * 3;

          if (!convertFrameToBgr(frame_data)) {
            RCLCPP_ERROR(this->get_logger(), "Unsupported or invalid pixel format from Daheng camera!");
            fail_count_++;
            continue;
          }

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          fail_count_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get frame failed! status: [%x], frame status: [%x]", frame_status, frame_data.nStatus);
          GXStreamOff(camera_handle_);
          GXStreamOn(camera_handle_);
          fail_count_++;
        }

        if (fail_count_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      GXStreamOff(camera_handle_);
      GXCloseDevice(camera_handle_);
      camera_handle_ = nullptr;
    }
    if (sdk_initialized_) {
      GXCloseLib();
      sdk_initialized_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void cameraFailSuggestion(GX_STATUS error)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "\x1b[1;31m状态码异常[%x]，相机可能没有正确启动。请务必检查相机是否被其他进程或者软件占用（例如大恒 GalaxyView）。"
      "关闭这些软件后再重试一次。\x1b[0m",
      error);
  }

  bool convertFrameToBgr(const GX_FRAME_DATA & frame_data)
  {
    const auto width = static_cast<int>(frame_data.nWidth);
    const auto height = static_cast<int>(frame_data.nHeight);
    if (width <= 0 || height <= 0 || frame_data.pImgBuf == nullptr) {
      return false;
    }

    image_msg_.data.resize(static_cast<size_t>(width * height * 3));
    cv::Mat bgr_mat(height, width, CV_8UC3, image_msg_.data.data());

    size_t required_input_bytes = 0;
    switch (frame_data.nPixelFormat) {
      case GX_PIXEL_FORMAT_BGR8:
      case GX_PIXEL_FORMAT_RGB8:
        required_input_bytes = static_cast<size_t>(width * height * 3);
        break;
      case GX_PIXEL_FORMAT_MONO8:
      case GX_PIXEL_FORMAT_BAYER_RG8:
      case GX_PIXEL_FORMAT_BAYER_GR8:
      case GX_PIXEL_FORMAT_BAYER_GB8:
      case GX_PIXEL_FORMAT_BAYER_BG8:
        required_input_bytes = static_cast<size_t>(width * height);
        break;
      default:
        break;
    }

    if (required_input_bytes > 0 && static_cast<size_t>(frame_data.nImgSize) < required_input_bytes) {
      RCLCPP_WARN(
        this->get_logger(), "Frame buffer too small: got %lld, need %zu", frame_data.nImgSize,
        required_input_bytes);
      return false;
    }

    switch (frame_data.nPixelFormat) {
      case GX_PIXEL_FORMAT_BGR8:
        std::memcpy(image_msg_.data.data(), frame_data.pImgBuf, required_input_bytes);
        return true;
      case GX_PIXEL_FORMAT_RGB8: {
        cv::Mat rgb_mat(height, width, CV_8UC3, frame_data.pImgBuf);
        cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);
        return true;
      }
      case GX_PIXEL_FORMAT_MONO8: {
        cv::Mat mono_mat(height, width, CV_8UC1, frame_data.pImgBuf);
        cv::cvtColor(mono_mat, bgr_mat, cv::COLOR_GRAY2BGR);
        return true;
      }
      case GX_PIXEL_FORMAT_BAYER_RG8: {
        cv::Mat bayer_mat(height, width, CV_8UC1, frame_data.pImgBuf);
        cv::cvtColor(bayer_mat, bgr_mat, cv::COLOR_BayerRG2BGR);
        return true;
      }
      case GX_PIXEL_FORMAT_BAYER_GR8: {
        cv::Mat bayer_mat(height, width, CV_8UC1, frame_data.pImgBuf);
        cv::cvtColor(bayer_mat, bgr_mat, cv::COLOR_BayerGR2BGR);
        return true;
      }
      case GX_PIXEL_FORMAT_BAYER_GB8: {
        cv::Mat bayer_mat(height, width, CV_8UC1, frame_data.pImgBuf);
        cv::cvtColor(bayer_mat, bgr_mat, cv::COLOR_BayerGB2BGR);
        return true;
      }
      case GX_PIXEL_FORMAT_BAYER_BG8: {
        cv::Mat bayer_mat(height, width, CV_8UC1, frame_data.pImgBuf);
        cv::cvtColor(bayer_mat, bgr_mat, cv::COLOR_BayerBG2BGR);
        return true;
      }
      default:
        RCLCPP_WARN(
          this->get_logger(),
          "Unsupported pixel format: 0x%llx. Supported: BGR8, RGB8, MONO8, BAYER_{RG,GR,GB,BG}8.",
          frame_data.nPixelFormat);
        return false;
    }
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;

    // ========== Exposure Time ==========
    GX_FLOAT_RANGE exposure_range{};
    GX_STATUS status = GXGetFloatRange(camera_handle_, GX_FLOAT_EXPOSURE_TIME, &exposure_range);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
    }

    double exposure_current = 1000.0;
    GXGetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, &exposure_current);

    param_desc.description = "Exposure time in microseconds";
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = exposure_range.dMin;
    param_desc.floating_point_range[0].to_value = exposure_range.dMax;
    param_desc.floating_point_range[0].step = 0.0;

    RCLCPP_INFO(
      this->get_logger(), "Exposure min/max (current): %.3f/%.1f (%.3f)", exposure_range.dMin,
      exposure_range.dMax, exposure_current);

    double exposure_time = this->declare_parameter("exposure_time", exposure_current, param_desc);
    GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_time);

    GXSetEnum(camera_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    GXSetFloat(camera_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, 250.0);

    // ========== Gain ==========
    GX_FLOAT_RANGE gain_range{};
    status = GXGetFloatRange(camera_handle_, GX_FLOAT_GAIN, &gain_range);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
    }

    double gain_current = 0.0;
    GXGetFloat(camera_handle_, GX_FLOAT_GAIN, &gain_current);

    param_desc.description = "Gain";
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].from_value = gain_range.dMin;
    param_desc.floating_point_range[0].to_value = gain_range.dMax;
    // Some Galaxy models report non-positive dInc for continuous gain controls; use step=0 to mark
    // this ROS parameter as continuously adjustable instead of rejecting valid runtime updates.
    param_desc.floating_point_range[0].step = gain_range.dInc > 0.0 ? gain_range.dInc : 0.0;

    double gain = this->declare_parameter("gain", gain_current, param_desc);
    GXSetFloat(camera_handle_, GX_FLOAT_GAIN, gain);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::string error_reason;

    for (const auto & param : parameters) {
      bool param_success = true;

      if (param.get_name() == "exposure_time") {
        GX_STATUS status = GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, param.as_double());
        if (GX_STATUS_SUCCESS != status) {
          param_success = false;
          error_reason = "Failed to set exposure time";
        }
      } else if (param.get_name() == "gain") {
        GX_STATUS status = GXSetFloat(camera_handle_, GX_FLOAT_GAIN, param.as_double());
        if (GX_STATUS_SUCCESS != status) {
          param_success = false;
          error_reason = "Failed to set gain";
        }
      } else {
        param_success = false;
        error_reason = "Unknown parameter: " + param.get_name();
      }

      if (!param_success) {
        result.successful = false;
        result.reason = error_reason;
      }
    }

    return result;
  }

  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;

  GX_DEV_HANDLE camera_handle_ = nullptr;
  bool sdk_initialized_ = false;
  std::vector<uint8_t> raw_frame_buffer_;
  std::string camera_index_content_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::atomic<bool> running_{true};
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
