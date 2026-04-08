#include "GxIAPI.h"
//#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
//换成大恒后的新库
#include <atomic>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>
//我一个接口都不想动，所以依旧叫hik_camera，节点叫hik_camera_node
namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting DaHengCameraNode!");

    /*
    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);
    */
    GX_STATUS status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) 
    {
      cameraFailSuggestion(status);
      throw std::runtime_error("GXInitLib failed, status=" + std::to_string(status));
    }
    sdk_initialized_ = true;

    /*
    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      */
    uint32_t device_count = 0;
    status = GXUpdateDeviceList(&device_count, 1000);
    while (device_count == 0 && rclcpp::ok()) 
    {
      RCLCPP_ERROR(this->get_logger(), "No Daheng camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", status);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      //nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      status = GXUpdateDeviceList(&device_count, 1000);
    }


    /*
    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    int status = MV_CC_OpenDevice(camera_handle_);
    if(status != MV_OK ){ cameraFailSuggestion(status);}
    */
    GX_OPEN_PARAM open_param {} ;
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    int camera_index = this->declare_parameter("camera_index", 1); //索引从1开始
    if (camera_index < 1) {
      RCLCPP_WARN(this->get_logger(), "Invalid camera_index=%d, clamp to 1", camera_index);
      camera_index = 1;
    }
    std::snprintf(camera_index_content_.data(), camera_index_content_.size(), "%d", camera_index);
    open_param.pszContent = camera_index_content_.data();

    status = GXOpenDevice(&open_param, &camera_handle_);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
      throw std::runtime_error("GXOpenDevice failed");
    }


    // Get camera infomation
    // MV_CC_GetImageInfo(camera_handle_, &img_info_);
    // image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
    
    // MVCC_ENUMVALUE stEnumValue = { 0 };
    // MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &stEnumValue);
    // RCLCPP_INFO(this->get_logger(), "Camera support %d pixel format(s)", stEnumValue.nSupportedNum);
    //大恒的SDK获取像素格式的方式不一样了
    int64_t img_width_max = 0;
    int64_t img_height_max = 0;
    int64_t payload_size = 0;
    GXGetInt(camera_handle_, GX_INT_WIDTH, &img_width_max);
    GXGetInt(camera_handle_, GX_INT_HEIGHT, &img_height_max);
    GXGetInt(camera_handle_, GX_INT_PAYLOAD_SIZE, &payload_size);

    // MVCC_INTVALUE_EX stIntValue = { 0 };
    // MV_CC_GetIntValueEx(camera_handle_, "Width", &stIntValue);
    // int img_width_max = stIntValue.nCurValue;
    // MV_CC_GetIntValueEx(camera_handle_, "Height", &stIntValue);
    // int img_height_max = stIntValue.nCurValue;
    // RCLCPP_INFO(this->get_logger(), "Image size: ( %d x %d )", img_width_max, img_height_max);

    RCLCPP_INFO(
      this->get_logger(), "Image size: ( %ld x %ld ), payload: %ld", img_width_max, img_height_max,
      payload_size);

    //image_msg_.data.resize(img_width_max * img_height_max * 3);
    image_msg_.data.resize(static_cast<size_t>(img_width_max * img_height_max * 3));
    raw_frame_buffer_.resize(static_cast<size_t>(payload_size));

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    //MV_CC_StartGrabbing(camera_handle_);
    status = GXStreamOn(camera_handle_);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
      throw std::runtime_error("GXStreamOn failed");
    }


    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");

    // camera_info_manager_ =
    //   std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
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

    capture_thread_ = std::thread
    {[this]() -> void 
      {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      // image_msg_.encoding = "rgb8";
      image_msg_.encoding = "bgr8";

      //static int frame_count = 0;

      // while (rclcpp::ok()) {
      //   nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      //   if (MV_OK == nRet) {
      //     image_msg_.header.stamp = this->now();
      //     image_msg_.width = out_frame.stFrameInfo.nWidth;
      //     image_msg_.height = out_frame.stFrameInfo.nHeight;
      //     image_msg_.step = image_msg_.width * 3;

      //     // OpenCV bayer converter is much faster than HIK SDK
      //     cv::Mat bayer_mat(
      //       out_frame.stFrameInfo.nHeight,
      //       out_frame.stFrameInfo.nWidth,
      //       CV_8UC1,
      //       out_frame.pBufAddr
      //     );
          
      //     cv::Mat rgb_mat(
      //       out_frame.stFrameInfo.nHeight,
      //       out_frame.stFrameInfo.nWidth,
      //       CV_8UC3,
      //       image_msg_.data.data()
      //     );
          
      //     cv::cvtColor(bayer_mat, rgb_mat, cv::COLOR_BayerRG2RGB);
      //     // cv::cvtColor(bayer_mat, rgb_mat, cv::COLOR_BayerRG2BGR);
          
      //     if (rgb_mat.empty()) {
      //       RCLCPP_ERROR(this->get_logger(), "OpenCV cvtColor failed!");
      //       MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
      //       continue;
      //     }

      //     frame_count++;
      //     if (frame_count % 2000 == 0) {
      //       int center_x = image_msg_.width / 2;
      //       int center_y = image_msg_.height / 2;
      //       int pixel_index = center_y * image_msg_.step + center_x * 3;
            
      //       if (pixel_index + 2 < image_msg_.data.size()) {
      //         uint8_t r = image_msg_.data[pixel_index];
      //         uint8_t g = image_msg_.data[pixel_index + 1];
      //         uint8_t b = image_msg_.data[pixel_index + 2];
              
      //         RCLCPP_INFO(this->get_logger(), 
      //           "\x1b[1;32mFrame %d - Pixel at (%d, %d): R=%d, G=%d, B=%d\x1b[0m",
      //           frame_count, center_x, center_y, r, g, b);
      //       } else {
      //         RCLCPP_WARN(this->get_logger(),
      //           "Frame %d - Pixel index out of bounds: %d (buffer size: %zu)",
      //           frame_count, pixel_index, image_msg_.data.size());
      //       }
      //     }

      //     camera_info_msg_.header = image_msg_.header;
      //     camera_pub_.publish(image_msg_, camera_info_msg_);

      //     MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
      //     fail_conut_ = 0;
      //   } else {
      //     RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
      //     MV_CC_StopGrabbing(camera_handle_);
      //     MV_CC_StartGrabbing(camera_handle_);
      //     fail_conut_++;
      //   }

      //   if (fail_conut_ > 5) {
      //     RCLCPP_FATAL(this->get_logger(), "Camera failed!");
      //     rclcpp::shutdown();
      //   }
      // }
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
          if (frame_status == GX_STATUS_SUCCESS) {
            RCLCPP_WARN(
              this->get_logger(), "Get frame failed! status: [%x], frame status: [%x]",
              frame_status, frame_data.nStatus);
          } else {
            RCLCPP_WARN(this->get_logger(), "Get frame failed! status: [%x]", frame_status);
          }
          GXStreamOff(camera_handle_);
          GXStreamOn(camera_handle_);
          fail_count_++;
        }

        if (fail_count_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
  
    }
   };
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      // MV_CC_StopGrabbing(camera_handle_);
      // MV_CC_CloseDevice(camera_handle_);
      // MV_CC_DestroyHandle(&camera_handle_);
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
  void cameraFailSuggestion(GX_status error) //原来是int error标志位，忘了哪来的了
  {
    RCLCPP_WARN(
      this->get_logger(),
      "\x1b[1;31m状态码异常[%x]，相机可能没有正确启动。请务必检查相机是否被其他进程或者软件占用（例如HIK MVS）。"
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

    const size_t expected_bgr_bytes = static_cast<size_t>(width * height * 3);
    if (image_msg_.data.size() != expected_bgr_bytes) {
      image_msg_.data.resize(expected_bgr_bytes);
    }
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




  void declareParameters()  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    //MVCC_FLOATVALUE f_value;
    

    // ========== 曝光时间 ==========
    // int status = MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    // if(status != MV_OK ){ cameraFailSuggestion(status);}
    GX_FLOAT_RANGE exposure_range{};
    GX_STATUS status = GXGetFloatRange(camera_handle_, GX_FLOAT_EXPOSURE_TIME, &exposure_range);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
    }

    double exposure_current = 1000.0;
    GXGetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, &exposure_current);

    param_desc.description = "Exposure time in microseconds";
    param_desc.floating_point_range.resize(1);  // 声明为浮点范围
    param_desc.floating_point_range[0].from_value = exposure_range.dMin;
    param_desc.floating_point_range[0].to_value = exposure_range.dMax;
    param_desc.floating_point_range[0].step = 0.0;  // step=0 表示连续可调

    RCLCPP_INFO(this->get_logger(),
     "Exposure min/max (current): %.3f/%.1f (%.3f)", exposure_range.dMin, exposure_range.dMax, exposure_current);

    double exposure_time = this->declare_parameter("exposure_time", 1000.0, param_desc);
    // MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(exposure_time));
    // MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", 250.000f);

    // MV_CC_SetBoolValue(camera_handle_, "ColorTransformationEnable", true);
    // MV_CC_SetFloatValue(camera_handle_, "Gamma", 7.5f);
    // MV_CC_SetBoolValue(camera_handle_, "CCMEnable", false);
  
    GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_time);
    GXSetEnum(camera_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON); 
    GXSetValue(camera_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, 250.0);


    // ========== Gain ==========
    //MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    GX_FLOAT_RANGE gain_range{};
    status = GXGetFloatRange(camera_handle_, GX_FLOAT_GAIN, &gain_range);
    if (status != GX_STATUS_SUCCESS) {
      cameraFailSuggestion(status);
    }

    double gain_current = 0.0;
    GXGetFloat(camera_handle_, GX_FLOAT_GAIN, &gain_current);
    
    param_desc.description = "Gain";
    param_desc.floating_point_range.resize(1);
    // param_desc.floating_point_range[0].from_value = exposure_range.dMin;
    // param_desc.floating_point_range[0].to_value = exposure_range.dMax;
    // param_desc.floating_point_range[0].step = 0.1;  // 假设增益步进0.1

    // double gain = this->declare_parameter("gain", static_cast<double>(f_value.fCurValue), param_desc);
    // MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain));

    param.desc.floating_point_range[0].from_value = gain_range.dMin;
    param.desc.floating_point_range[0].to_value = gain_range.dMax;

    param.desc.floating_point_range[0].step = gain_range.dInc > 0.0 ? gain_range.dInc : 0.0;
    // 如果步进无效（bIncIsValid为false），则设置为0表示连续可调
    double gain = this->declare_parameter("gain", gain_current, param_desc);
    GXSetFloat(camera_handle_, GX_FLOAT_GAIN, gain);
  }


  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
    {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;  // 默认成功
    std::string error_reason;

    for (const auto & param : parameters) {
      bool param_success = true;  // 单个参数的成功标志

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

      // 只要有一个参数失败，整体就失败
      if (!param_success) {
        result.successful = false;
        result.reason = error_reason;
        // 可以选择 break; 立即返回，或继续验证其他参数
      }
    }

    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  // int nRet = MV_OK;
  // void * camera_handle_;
  // MV_IMAGE_BASIC_INFO img_info_;

  // MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  GX_DEV_HANDLE camera_handle_ = nullptr;
  bool sdk_initialized_ = false;
  std::vector<uint8_t> raw_frame_buffer_;
  std::array<char, 16> camera_index_content_{};

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_conut_ = 0;
  std::atomic<bool> running_{true};
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)