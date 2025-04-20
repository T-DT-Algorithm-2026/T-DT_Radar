// hik
#include "MvErrorDefine.h"

// camera
#include "baseCamera.h"
#include "hikCamera.h"

// opencv
#include <opencv2/core/utility.hpp>

// roborts_utils
#include <roborts_utils/base_msg.h>
#include <roborts_utils/base_param.h>
#include <roborts_utils/base_toolkit.h>

// STL
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace tdt_vision {

HikvisionCam::HikvisionCam(std::string config_name,
                           rclcpp::Node::SharedPtr camera_node)
    : Camera(camera_node) {
  RCLCPP_INFO(camera_node_->get_logger(), "hikcamera_node init");

  setCameraParam(readCameraParam(config_name));

  camera_info_pub_ =
      camera_node->create_publisher<sensor_msgs::msg::CameraInfo>(
          "camera_info", rclcpp::SensorDataQoS());
}

bool HikvisionCam::readCameraParam(std::string config_name) {
  identity_.config_name = config_name;
  identity_.type = TDT_CAMERA_CAMTYPE_HIKVISION;

  //相机名字：方便区分
  LoadParam::ReadParam(identity_.config_name, "name", identity_.name);

  //相机guid
  LoadParam::ReadParam(identity_.config_name, "guid", identity_.guid);

  int type = 0;
  LoadParam::ReadParam(identity_.config_name, "type", type);
  identity_.type = (TDT_CAMERA_CAMTYPE)type;

  LoadParam::ReadParam(identity_.config_name, "width", format_.width);

  LoadParam::ReadParam(identity_.config_name, "height", format_.height);

  LoadParam::ReadParam(identity_.config_name, "fps", format_.fps);

  LoadParam::ReadParam(identity_.config_name, "pixel_format",
                       format_.pixel_format);

  LoadParam::ReadParam(identity_.config_name, "armor_exposure",
                       setting_.exposure);

  LoadParam::ReadParam(identity_.config_name, "gain", setting_.gain);
  LoadParam::ReadParam(identity_.config_name, "balance_val",
                       setting_.balance_val);
  LoadParam::ReadParam(identity_.config_name, "balance_red",
                       setting_.balance_red);
  LoadParam::ReadParam(identity_.config_name, "balance_green",
                       setting_.balance_green);
  LoadParam::ReadParam(identity_.config_name, "balance_blue",
                       setting_.balance_blue);
  LoadParam::ReadParam(identity_.config_name, "brightness",
                       setting_.brightness);
  LoadParam::ReadParam(identity_.config_name, "saturation",
                       setting_.saturation);
  LoadParam::ReadParam(identity_.config_name, "contrast", setting_.contrast);
  LoadParam::ReadParam(identity_.config_name, "gamma", setting_.gamma);
  LoadParam::ReadParam(identity_.config_name, "sharpness", setting_.sharpness);
  LoadParam::ReadParam(identity_.config_name, "black_level",
                       setting_.black_level);
  LoadParam::ReadParam(identity_.config_name, "hue", setting_.hue);

  LoadParam::ReadParam(identity_.config_name, "trigger_mode",
                       setting_.trigger_mode);

  LoadParam::ReadParam(identity_.config_name, "get_timeStamp",
                       setting_.get_timeStamp);

  LoadParam::ReadParam(identity_.config_name, "trigger_source",
                       setting_.trigger_source);
  LoadParam::ReadParam(identity_.config_name, "trigger_activation",
                       setting_.trigger_activation);
  LoadParam::ReadParam(identity_.config_name, "trigger_delay",
                       setting_.trigger_delay);
  LoadParam::ReadParam(identity_.config_name, "trigger_selector",
                       setting_.trigger_selector);
  LoadParam::ReadParam(identity_.config_name, "trigger_cache",
                       setting_.trigger_cache);
  LoadParam::ReadParam(identity_.config_name, "image_node_num",
                       setting_.image_node_num);

  LoadParam::ReadParam(identity_.config_name, "reverse_x", setting_.reverse_x);
  LoadParam::ReadParam(identity_.config_name, "reverse_y", setting_.reverse_y);

  LoadParam::ReadParam(identity_.config_name, "camera_matrix",
                       setting_.camera_matrix);
  LoadParam::ReadParam(identity_.config_name, "dist_coeff",
                       setting_.dist_coeff);

  return true;
}

void HikvisionCam::publishCameraInfo() {
  sensor_msgs::msg::CameraInfo camera_info_msg;

  camera_info_msg.header.frame_id = "camera_frame";
  camera_info_msg.header.stamp = camera_node_->now();
  camera_info_msg.height = format_.height;  // switched back width and height
  camera_info_msg.width = format_.width;

  // camera_matrix
  camera_info_msg.k[0] = setting_.camera_matrix.at<double>(0, 0);  // fx
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = setting_.camera_matrix.at<double>(0, 2);  // cx
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = setting_.camera_matrix.at<double>(1, 1);  // fy
  camera_info_msg.k[5] = setting_.camera_matrix.at<double>(1, 2);  // cy
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;

  // dist_coeff
  camera_info_msg.d.resize(5);
  for (int i = 0; i < 5; i++) {
    camera_info_msg.d[i] = setting_.dist_coeff.at<double>(i);
  }

  // rectification_matrix
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;

  // projection_matrix
  camera_info_msg.p[0] = setting_.camera_matrix.at<double>(0, 0);  // fx
  camera_info_msg.p[1] = 0.0;
  camera_info_msg.p[2] = setting_.camera_matrix.at<double>(0, 2);  // cx
  camera_info_msg.p[3] = 0.0;  // no shift for rectified image
  camera_info_msg.p[4] = 0.0;
  camera_info_msg.p[5] = setting_.camera_matrix.at<double>(1, 1);  // fy
  camera_info_msg.p[6] = setting_.camera_matrix.at<double>(1, 2);  // cy
  camera_info_msg.p[7] = 0.0;
  camera_info_msg.p[8] = 0.0;
  camera_info_msg.p[9] = 0.0;
  camera_info_msg.p[10] = 1.0;
  camera_info_msg.p[11] = 0.0;

  // publish
  camera_info_pub_->publish(camera_info_msg);
}

void HikvisionCam::setCameraParam(bool ifdone) {
  if (!ifdone) {
    TDT_FATAL("Load %s ERRor!", identity_.config_name.c_str());
    return;
  } else {
    if (!InitHandle()) {
      TDT_FATAL("Failed to Init hikvision Handle. (incorrect guid %s)",
                identity_.guid.c_str());
    }

    if (!OpenCamera())  //开启相机
    {
      TDT_FATAL("Failed to Open hikvision camera. (incorrect guid %s)",
                identity_.guid.c_str());
    }

    Set(TDT_CAMERA_FORMAT::TDT_CAMERA_FORMAT_PIXEL, format_.pixel_format);
    Set(TDT_CAMERA_FORMAT::TDT_CAMERA_FORMAT_WIDTH, format_.width);
    Set(TDT_CAMERA_FORMAT::TDT_CAMERA_FORMAT_HEIGHT, format_.height);
    Set(TDT_CAMERA_FORMAT::TDT_CAMERA_FORMAT_FPS, format_.fps);

    Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
    Set(TDT_CAMERA_SETTING_GAIN, setting_.gain);
    Set(TDT_CAMERA_SETTING_BRIGHTNESS, setting_.brightness);
    Set(TDT_CAMERA_SETTING_BALANCE_RED, setting_.balance_red);
    Set(TDT_CAMERA_SETTING_BALANCE_GREEN, setting_.balance_green);
    Set(TDT_CAMERA_SETTING_BALANCE_BLUE, setting_.balance_blue);
    Set(TDT_CAMERA_SETTING_HUE, setting_.hue);
    Set(TDT_CAMERA_SETTING_SATURATION, setting_.saturation);
    Set(TDT_CAMERA_SETTING_GAMMA, setting_.gamma);
    Set(TDT_CAMERA_SETTING_SHARPNESS, setting_.sharpness);
    Set(TDT_CAMERA_SETTING_TRIGER_MODE, setting_.trigger_mode);
    Set(TDT_CAMERA_SETTING_TRIGER_SOURCE, setting_.trigger_source);
    Set(TDT_CAMERA_SETTING_TRIGER_ACTIVATION, setting_.trigger_activation);
    Set(TDT_CAMERA_SETTING_TRIGER_DELAY, setting_.trigger_delay);
    Set(TDT_CAMERA_SETTING_TRIGGER_SELECTOR, setting_.trigger_selector);
    Set(TDT_CAMERA_SETTING_TRIGGER_CACHE, setting_.trigger_cache);
    Set(TDT_CAMERA_SETTING_IMAGE_NODE_NUM, setting_.image_node_num);
    Set(TDT_CAMERA_SETTING_BLACK_LEVEL, setting_.black_level);
    Set(TDT_CAMERA_SETTING_REVERSE_X, setting_.reverse_x);
    Set(TDT_CAMERA_SETTING_REVERSE_Y, setting_.reverse_y);

    StartGrabbing();
    // std::thread t(&HikvisionCam::TakeFrame, this);
    // t.detach();
  }
}

bool HikvisionCam::InitHandle() {
  int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList);
  if (MV_OK != nRet) {
    TDT_ERROR("MV_CC_EnumDevices fail [%d]\n", nRet);
    return false;
  }

  if (m_stDevList.nDeviceNum == 0) {
    TDT_ERROR("no camera found!\n");
    return false;
  }

  TDT_INFO("Hikcamera number: [%d] \n", m_stDevList.nDeviceNum);

  bool pass_guid = false;
  for (size_t i = 0; i < m_stDevList.nDeviceNum; i++) {
    if (identity_.guid ==
        std::string((char*)m_stDevList.pDeviceInfo[i]
                        ->SpecialInfo.stUsb3VInfo.chDeviceGUID)) {
      nRet = MV_CC_CreateHandle(&handle_, m_stDevList.pDeviceInfo[i]);
      if (MV_OK != nRet) {
        TDT_INFO("MV_CC_CreateHandle fali with guid [%s]\n",
                 identity_.guid.c_str());
        continue;
      } else {
        TDT_INFO("MV_CC_CreateHandle success with guid [%s]\n",
                 identity_.guid.c_str());
        pass_guid = true;
        break;
      }
    }
  }

  if (!pass_guid) {
    TDT_ERROR("No camera found with guid: [%s]\n", identity_.guid.c_str());

    if (m_stDevList.nDeviceNum == 1) {
      identity_.guid = std::string((char*)m_stDevList.pDeviceInfo[0]
                                       ->SpecialInfo.stUsb3VInfo.chDeviceGUID);
      // TODO:应该保存相机的guid

      nRet = MV_CC_CreateHandle(&handle_, m_stDevList.pDeviceInfo[0]);

      if (MV_OK != nRet) {
        TDT_ERROR("MV_CC_CreateHandle fail with guid: [%s]\n",
                  identity_.guid.c_str());
        return false;
      }

      TDT_WARNING(
          "Only one camera found with wrong guid offered,the right one "
          "is[%s]\n",
          identity_.guid.c_str());
      return true;
    } else {
      TDT_FATAL("[%d] HikCameras should run with their specific guid\n",
                m_stDevList.nDeviceNum);
      return false;
    }
  }
  return true;
}

void HikvisionCam::pause(bool ifPause) {}

bool HikvisionCam::OpenCamera() {
  // 注册数据回调函数，采集图像数据在回调函数中获取
  uint32_t nAccessMode = MV_ACCESS_Exclusive;
  unsigned short nSwitchoverKey = 0;
  // 连接设备
  int n_ret_ = MV_CC_OpenDevice(handle_, nAccessMode,
                                nSwitchoverKey);  // 通过获得的句柄打开相机
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_OpenDevice fail [%d]\n", n_ret_);
    return false;
  }
  TDT_INFO("MV_CC_OpenDevice SUCESS [%d]\n", n_ret_);
  return true;
}

bool HikvisionCam::CloseCamera() {
  int n_ret_ = MV_CC_CloseDevice(handle_);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_CloseDevice fail [%d]\n", n_ret_);
    return false;
  }
  return true;
}

template <class T>
void HikvisionCam::Set(TDT_CAMERA_SETTING tdt_camera_setting, const T& value) {
  bool ret = true;
  switch (tdt_camera_setting) {
    case TDT_CAMERA_SETTING_EXPOSURE:
      if (value < 0) {
        ret &= SetExposureAuto(true);
      } else {
        ret &= SetExposureAuto(false);
        ret &= SetExposure(value);
      }
      if (ret) {
        setting_.exposure = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (exposure : %d)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_GAIN:
      if (value < 0) {
        ret &= SetGainAuto(true);
      } else {
        ret &= SetGainAuto(false);
        ret &= SetGain(value);
      }
      if (ret) {
        setting_.gain = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (gain : %d)", value);
      }
      break;

    case TDT_CAMERA_SETTING_BRIGHTNESS:
      if (value < 0) {
        ret &= SetBrightness(value);
      }
      if (ret) {
        setting_.brightness = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (brightness : %d)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_BALANCE_RED:
      if (value < 0) {
        ret &= SetWhitebalanceAuto(true);
      } else if (setting_.balance_green >= 0 && setting_.balance_blue >= 0) {
        ret &= SetWhitebalanceAuto(false);
        ret &= SetWhitebalance(value, setting_.balance_green,
                               setting_.balance_blue);
      }
      if (ret) {
        setting_.balance_red = value;
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera setting. (balance_red : "
            "%d)",
            value);
      }
      break;

    case TDT_CAMERA_SETTING_BALANCE_GREEN:
      if (value < 0) {
        ret &= SetWhitebalanceAuto(true);
      } else if (setting_.balance_red >= 0 && setting_.balance_blue >= 0) {
        ret &= SetWhitebalanceAuto(false);
        ret &=
            SetWhitebalance(setting_.balance_red, value, setting_.balance_blue);
      }
      if (ret) {
        setting_.balance_green = value;
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera setting. (balance_green : "
            "%d)",
            value);
      }
      break;

    case TDT_CAMERA_SETTING_BALANCE_BLUE:
      if (value < 0) {
        ret &= SetWhitebalanceAuto(true);
      } else if (setting_.balance_green >= 0 && setting_.balance_red >= 0) {
        ret &= SetWhitebalanceAuto(false);
        ret &= SetWhitebalance(setting_.balance_red, setting_.balance_green,
                               value);
      }
      if (ret) {
        setting_.balance_blue = value;
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera setting. (balance_blue : "
            "%d)",
            value);
      }
      break;

    case TDT_CAMERA_SETTING_HUE:
      if (value < 0) {
        ret &= SetHueDisable(true);
      } else {
        ret &= SetHueDisable(false);
        ret &= SetHue(value);
      }
      if (ret) {
        setting_.hue = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (hue : %d)", value);
      }
      break;

    case TDT_CAMERA_SETTING_SATURATION:
      if (value < 0) {
        ret &= SetSaturationDisable(true);
      } else {
        ret &= SetSaturationDisable(false);
        ret &= SetSaturation(value);
      }
      if (ret) {
        setting_.saturation = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (saturation : %d)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_GAMMA:
      if (value < 0) {
        ret &= SetGammaDisable(true);
      } else {
        ret &= SetGammaDisable(true);
        ret &= SetGammaDisable(false);
        if (value < 1) {
          std::cout << "in" << std::endl;
          ret &= SetGamma(value, MV_GAMMA_SELECTOR_USER);
        } else {
          ret &= SetGamma();
        }
      }
      if (ret) {
        setting_.gamma = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (gamma : %d)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_SHARPNESS:
      if (value < 0) {
        ret &= SetSharpnessDisable(true);
      } else {
        ret &= SetSharpnessDisable(false);
        ret &= SetSharpness(value);
      }
      if (ret) {
        setting_.sharpness = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (sharpness : %d)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_BLACK_LEVEL:
      if (value < 0) {
        ret &= SetBlacklevelDisable(true);
      } else {
        ret &= SetBlacklevelDisable(false);
        ret &= SetBlacklevel(value);
      }
      if (ret) {
        setting_.black_level = value;
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera setting. (black_level : "
            "%d)",
            value);
      }
      break;

    case TDT_CAMERA_SETTING_TRIGER_MODE:
      ret &= SetTriggerMode(value);
      break;

    case TDT_CAMERA_SETTING_TRIGER_SOURCE:
      ret &= SetTriggerSource(value);
      break;

    case TDT_CAMERA_SETTING_TRIGER_ACTIVATION:
      ret &= SetTriggerActivation(value);
      break;

    case TDT_CAMERA_SETTING_TRIGGER_SELECTOR:
      ret &= SetTriggerSelector(value);
      break;

    case TDT_CAMERA_SETTING_TRIGGER_CACHE:
      ret &= SetTriggerCache(value);
      break;

    case TDT_CAMERA_SETTING_IMAGE_NODE_NUM:
      ret &= SetImageNodeNum(value);
      break;

    case TDT_CAMERA_SETTING_REVERSE_X:
      if (value == 0.0f) {
        ret &= SetReverse_X(false);
      } else {
        ret &= SetReverse_X(value);
      }
      if (ret) {
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (reverse_x : %f)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_REVERSE_Y:
      if (value == 0.0f) {
        ret &= SetReverse_Y(false);
      } else {
        ret &= SetReverse_Y(value);
      }
      if (ret) {
      } else {
        TDT_ERROR("Failed to set hikvision camera setting. (reverse_y : %f)",
                  value);
      }
      break;

    case TDT_CAMERA_SETTING_TRIGER_DELAY:
      ret &= SetTriggerDelay(value);
      break;
    default:
      TDT_ERROR(
          "Failed to set hikvision camera setting. (TDT_CAMERA_SETTING : "
          "%d)",
          tdt_camera_setting);
      break;
  }
}

template <class T>
void HikvisionCam::Set(TDT_CAMERA_FORMAT tdt_camera_format, const T& value) {
  bool ret = true;
  //         CloseGrabbing();
  switch (tdt_camera_format) {
    case TDT_CAMERA_FORMAT_PIXEL:
      if (value == 0) {
        ret &= SetPixelformat(PixelType_Gvsp_RGB8_Packed);
      } else if (value == 1) {
        ret &= SetPixelformat(PixelType_Gvsp_BayerRG8);
      } else {
        TDT_ERROR(
            "Failed to set hikvision camera format. (incorrect "
            "pixel_format %d)",
            value);
      }
      if (ret) {
        format_.pixel_format = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (pixel_format %d)",
                  value);
      }
      break;
    case TDT_CAMERA_FORMAT_WIDTH:
      ret &= SetResolution(value, format_.height);
      if (ret) {
        format_.width = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (width %d)", value);
      }
      break;
    case TDT_CAMERA_FORMAT_HEIGHT:
      ret &= SetResolution(format_.width, value);
      if (ret) {
        format_.height = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (height %d)", value);
      }
      break;
    case TDT_CAMERA_FORMAT_FPS:
      if (value < 0) {
        ret &= SetFpsDisable(true);
      } else {
        ret &= SetFps(value);
      }
      if (ret) {
        format_.fps = value;
      } else {
        TDT_ERROR("Failed to set hikvision camera format. (fps %d)", value);
      }
      break;

    default:
      TDT_ERROR(
          "Failed to set hikvision camera format. (TDT_CAMERA_SETTING : "
          "%d)",
          tdt_camera_format);
      break;
  }
}

void HikvisionCam::SetLut(float value, int mode) {
  std::cout << "value= " << value << std::endl;

  lut_.create(1, 256, CV_8UC1);
  cv::Mat_<uchar> table = lut_;
  switch (mode) {
    case 0:
      for (int i = 0; i < 256; i++) {
        double f;
        f = (i + 0.5F) / 256;
        f = pow(f, value);
        table(0, i) = (f * 256 - 0.5F);
      }
      break;
    case 1:
      for (int i = 0; i < value; i++) {
        table(0, i) = 255 * i / value;
      }
      for (int i = value; i < 256; i++) {
        table(0, i) = 255;
      }
      break;
  }
}

bool HikvisionCam::StartGrabbing() {
  // 开始采集图像
  int n_ret_ = MV_CC_StartGrabbing(handle_);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_StartGrabbing fail! n_ret_ [%x]\n", n_ret_);
    MV_CC_StopGrabbing(handle_);
    MV_CC_DestroyHandle(handle_);
    return false;
  }
  // 获取一帧数据的大小
  MVCC_INTVALUE stIntvalue = {0};
  n_ret_ = MV_CC_GetIntValue(handle_, "PayloadSize", &stIntvalue);
  if (n_ret_ != MV_OK) {
    TDT_ERROR("MV_CC_GetPayloadSize failed! n_ret_ [%x]\n", n_ret_);
    return false;
  }

  n_data_size_ =
      stIntvalue.nCurValue + 2048;  // 一帧数据大小+预留字节(用于SDK内部处理)
  // 抓取一帧图片
  p_data_ = (unsigned char*)malloc(n_data_size_);

  memset(&st_img_info_, 0, sizeof(MV_FRAME_OUT_INFO_EX));

  return true;
}

void HikvisionCam::GetImage(sensor_msgs::msg::Image& image) {
  double start_tick = cv::getTickCount();

  if (setting_.trigger_mode) {
    if (!GetMat_Triger(image, setting_.trigger_source)) {
      TDT_ERROR("Failed to get image from hikvision cam");
    }
  } else {
    if (!GetMat(image, setting_.get_timeStamp)) {
      TDT_ERROR("Failed to get image from hikvision cam");
    }
  }

  // double cost_time =
  //     (cv::getTickCount() - start_tick) / cv::getTickFrequency() * 1000;
  // TDT_INFO("HikvisionCam::GetImage cost time %f ms", cost_time);

  // if (cost_time > 1) {
  //   TDT_WARNING("HikvisionCam::GetImage cost time %f ms", cost_time);
  // }
}

// 取帧时间过长或者取帧错误 如果内存炸掉，malloc申请出错 像素转换错误 return
// false
bool HikvisionCam::GetMat_Triger(sensor_msgs::msg::Image& image_msg,
                                 int trigger_source) {
  // std::cout<<"in trigger mode"<<std::endl;
  int n_ret_ = 0;
  if (first_run_) {
    if (trigger_source == MV_TRIGGER_SOURCE_SOFTWARE) sendTrigger();
    first_run_ = 0;
  }
  int RestartTimes = 1;
OPEN_LOOP:

  // n_ret_ = MV_CC_GetImageBuffer(handle_, &img_out_, 2000);
  n_ret_ = MV_CC_GetOneFrameTimeout(handle_, p_data_, n_data_size_,
                                    &st_img_info_, 1000);
  if (n_ret_ != MV_OK) {  // 如果取一帧超过时间，或者取帧错误，都不对
    TDT_WARNING("MV_CC_GetOneFrameTimeout failed! n_ret_ [%x]\n",
                n_ret_);  // 如果卡帧就报错（现在估计是温度高的问题）
    if (trigger_source == MV_TRIGGER_SOURCE_SOFTWARE) sendTrigger();

    if (RestartTimes < 5) {
      TDT_ERROR("the %d times !   雷达新相机有问题！ \n",
                RestartTimes);
      rclcpp::shutdown();
      exit(0);      
      RestartCamera();
      if (trigger_source == MV_TRIGGER_SOURCE_SOFTWARE) sendTrigger();
      ++RestartTimes;
      goto OPEN_LOOP;  // 5为重启相机的次数，如果重启多次解决不了问题，就重启程序
    } else if (RestartTimes >= 5) {
      TDT_FATAL("Restart ERROR, restart this executable\n");
    }
    return false;
  }
  // st_img_info_ = img_out_.stFrameInfo;
  // memcpy(p_data_, img_out_.pBufAddr, st_img_info_.nFrameLen);
  // MV_CC_FreeImageBuffer(handle_, &img_out_);
  
  double TriggerTime = triggerTime_;
  if (trigger_source == MV_TRIGGER_SOURCE_SOFTWARE) sendTrigger();

  int tdt_mode = 0;
  // int tdt_mode = tdttoolkit::BaseBlackboard::Get<int>("vision_mode");

  if (tdt_mode == 0)
    LoadParam::ReadParam(identity_.config_name, "armor_exposure", expose_time_);
  else if (tdt_mode == 1 || tdt_mode == 2)
    LoadParam::ReadParam(identity_.config_name, "buff_exposure", expose_time_);
  else {
    MVCC_FLOATVALUE exposeTime = {-1, -1, -1};
    n_ret_ = MV_CC_GetFloatValue(handle_, "ExposureTime", &exposeTime);
    if (MV_OK != n_ret_) {
      TDT_ERROR("failed in GetExposedTime[%x]\n", n_ret_);
    }
    expose_time_ = exposeTime.fCurValue;
  }
  // TDT_INFO("expose time = %f", expose_time_);

  MV_CC_PIXEL_CONVERT_PARAM StParam = {0};
  memset(&StParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
  // 源数据
  StParam.pSrcData = p_data_;                    // 原始图像数据
  StParam.nSrcDataLen = st_img_info_.nFrameLen;  // 原始图像数据长度
  StParam.enSrcPixelType = st_img_info_.enPixelType;  // 原始图像数据的像素格式
  StParam.nWidth = st_img_info_.nWidth;               // 图像宽
  StParam.nHeight = st_img_info_.nHeight;             // 图像高
  // 目标数据pImage
  StParam.enDstPixelType =
      PixelType_Gvsp_BGR8_Packed;  // 需要保存的像素格式类型，转换成BGR格式
  StParam.nDstBufferSize =
      st_img_info_.nWidth * st_img_info_.nHeight * 4 + 2048;  // 存储节点的大小
  // unsigned char *p_image = (unsigned char *)malloc(
  //     st_img_info_.nWidth * st_img_info_.nHeight * 4 + 2048);
  // if (p_image == NULL) {
  //   return false;
  // }
  // StParam.pDstBuffer = p_image;  // 输出数据缓冲区，存放转换之后的数据
  image_msg.data.resize(st_img_info_.nWidth * st_img_info_.nHeight * 3);
  StParam.pDstBuffer = image_msg.data.data();
  n_ret_ = MV_CC_ConvertPixelType(handle_, &StParam);  // 转换像素
  if (n_ret_ != MV_OK) {
    TDT_ERROR("MV_CC_ConvertPixelType failed! n_ret_ [%x]\n", n_ret_);
    return false;
  }
  image_msg.height = st_img_info_.nHeight;
  image_msg.width = st_img_info_.nWidth;
  image_msg.step = st_img_info_.nWidth * 3;
  image_msg.encoding = "bgr8";
  image_msg.header.stamp =
      tdttoolkit::Time::GetRosTimeByTime(TriggerTime + expose_time_ / 2);

  cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3,
                image_msg.data.data());
  // if (open_lut_) {
  LUT(frame, lut_, frame);
  // } else {
  //   frame.copyTo(frame);
  // }
  // ret_time = TriggerTime + expose_time_ / 2;
  // free(p_image);
  // p_image = NULL;
  return true;
}

bool HikvisionCam::GetMat(sensor_msgs::msg::Image& image_msg,
                          bool get_timeStamp) {
  /*
if (first_run_) {
first_run_ = 0;
if (get_timeStamp) {
SetTriggerMode(MV_TRIGGER_MODE_ON);
SetTriggerSource(MV_TRIGGER_SOURCE_SOFTWARE);
SetTriggerSelector(12);
SetTriggerDelay(0);
sendTrigger();
int n_ret_ = 0;
n_ret_ = MV_CC_GetOneFrameTimeout(handle_, p_data_, n_data_size_,
              &st_img_info_, 1000);
if (n_ret_ != MV_OK) {  // 如果取一帧超过时间，或者取帧错误，都不对
TDT_ERROR("MV_CC_GetOneFrameTimeout failed! n_ret_ [%x]\n", n_ret_);
first_run_ = 1;
}
MV_CC_PIXEL_CONVERT_PARAM StParam = {0};
memset(&StParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
int64_t timeStamp =
(st_img_info_.nDevTimeStampLow +
(int64_t)(st_img_info_.nDevTimeStampHigh) * 4294967296);

MVCC_FLOATVALUE exposeTime = {-1, -1, -1};
n_ret_ = MV_CC_GetFloatValue(handle_, "ExposureTime", &exposeTime);
if (MV_OK != n_ret_) {
TDT_ERROR("failed in GetExposedTime[%x]\n", n_ret_);
}
expose_time_ = exposeTime.fCurValue;

time_diff = triggerTime_ - timeStamp / 1e2;
TDT_INFO("time diff = %f", time_diff);
SetTriggerMode(MV_TRIGGER_MODE_OFF);
}
}
OPEN_LOOP:

int RestartTimes = 1;
double startTime = cv::getTickCount() / cv::getTickFrequency();
int n_ret_ = 0;
n_ret_ = MV_CC_GetImageBuffer(handle_, &img_out_, 2000);
if (n_ret_ != MV_OK) {  // 如果取一帧超过时间，或者取帧错误，都不对
TDT_ERROR("MV_CC_GetImageBuffer failed! n_ret_ [%x]\n",
n_ret_);  // 如果卡帧就报错（现在估计是温度高的问题）
RestartCamera();
TDT_WARNING("the %d times !   Get Frame Error , will restart the camera \n",
RestartTimes);
++RestartTimes;
if (RestartTimes < 5)
goto OPEN_LOOP;  // 5为重启相机的次数，如果重启多次解决不了问题，就重启程序
else {
TDT_ERROR("Restart ERROR, restart this executable\n");
exit(-1);
}
return false;
}
// TDT_INFO("Camera Wait Time = %fms",
//          (cv::getTickCount() / cv::getTickFrequency() - startTime) *
//          1000);
st_img_info_ = img_out_.stFrameInfo;
memcpy(p_data_, img_out_.pBufAddr, st_img_info_.nFrameLen);
MV_CC_FreeImageBuffer(handle_, &img_out_);

MV_CC_PIXEL_CONVERT_PARAM StParam = {0};
memset(&StParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
int64_t timeStamp = (st_img_info_.nDevTimeStampLow +
(int64_t)(st_img_info_.nDevTimeStampHigh) * 4294967296);
// ret_time = timeStamp / 1e2 + time_diff;  // 单位us

// 源数据
StParam.pSrcData = p_data_;                    // 原始图像数据
StParam.nSrcDataLen = st_img_info_.nFrameLen;  // 原始图像数据长度
StParam.enSrcPixelType = st_img_info_.enPixelType;  // 原始图像数据的像素格式
StParam.nWidth = st_img_info_.nWidth;               // 图像宽
StParam.nHeight = st_img_info_.nHeight;             // 图像高
// 目标数据pImage
StParam.enDstPixelType =
PixelType_Gvsp_BGR8_Packed;  // 需要保存的像素格式类型，转换成BGR格式
StParam.nDstBufferSize =
st_img_info_.nWidth * st_img_info_.nHeight * 4 + 2048;  // 存储节点的大小
// unsigned char *p_image = (unsigned char *)malloc(
//     st_img_info_.nWidth * st_img_info_.nHeight * 4 + 2048);
// if (p_image == NULL) {
//   return false;
// }
// StParam.pDstBuffer = p_image;  // 输出数据缓冲区，存放转换之后的数据
image_msg.data.resize(st_img_info_.nWidth * st_img_info_.nHeight * 3);
StParam.pDstBuffer = image_msg.data.data();
n_ret_ = MV_CC_ConvertPixelType(handle_, &StParam);  // 转换像素
if (n_ret_ != MV_OK) {
TDT_ERROR("MV_CC_ConvertPixelType failed! n_ret_ [%x]\n", n_ret_);
return false;
}
image_msg.height = st_img_info_.nHeight;
image_msg.width = st_img_info_.nWidth;
image_msg.step = st_img_info_.nWidth * 3;
// image_msg.encoding = "bgr8";
// double ret_time = timeStamp / 1e2 + time_diff;
// image_msg.header.stamp = ret_time;
// image_msg.header.stamp=this->getTimeNow();

cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3,
image_msg.data.data());

// if (open_lut_) {
LUT(frame, lut_, frame);
// } else {
//   frame.copyTo(frame);
// }
// ret_time = TriggerTime + expose_time_ / 2;
// free(p_image);
// p_image = NULL;
return true;
*/

  /* 旧取帧法 */
  /*
  n_ret_ = MV_CC_GetImageForBGR(handle_, p_data_, n_data_size_, &st_img_info_,
  1000); struct timeval get_mat_time; gettimeofday(&get_mat_time,NULL); cv::Mat
  frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3,
  p_data_);//读入图片到frame
  img=frame;//赋给Camera_Output，注意img和frame共享一个矩阵
  if (save_res_ == 1){
      OutputVideo << img;
  }
  return true;
  */
OPEN_LOOP:
  // double startTime = cv::getTickCount() / cv::getTickFrequency()* 1000000;

  int RestartTimes = 1;
  int n_ret_ = MV_CC_GetOneFrameTimeout(handle_, p_data_, n_data_size_,
                                        &st_img_info_, 1000);
  if (n_ret_ != MV_OK) {  // 如果取一帧超过时间，或者取帧错误，都不对
    TDT_ERROR("MV_CC_GetOneFrameTimeout failed! n_ret_ [%x]\n",
              n_ret_);  // 如果卡帧就报错（现在估计是温度高的问题）
    RestartCamera();
    TDT_WARNING("the %d times !   Get Frame Error , will restart the camera \n",
                RestartTimes);
    ++RestartTimes;
    if (RestartTimes < 5)
      goto OPEN_LOOP;  // 5为重启相机的次数，如果重启多次解决不了问题，就重启程序
    else {
      TDT_ERROR("Restart ERROR, restart this executable\n");
      exit(-1);
    }
    return false;
  }
  // double recieveTime = cv::getTickCount() / cv::getTickFrequency()* 1000000;
  // std::cout << " triggerTime=" << (triggerTime - startTime)  << "
  // recieveTime=" << (recieveTime - triggerTime) << std::endl;
  MV_CC_PIXEL_CONVERT_PARAM StParam = {0};
  memset(&StParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
  int64_t timeStamp = (st_img_info_.nDevTimeStampLow +
                       (int64_t)(st_img_info_.nDevTimeStampHigh) * 4294967296);
  // ret_time          = timeStamp / 1e2; // 单位us

  // TDT_INFO("Camera test %ld", st_img_info_.nHostTimeStamp);
  // TDT_INFO("Camera Time is %ld", timeStamp / 100000);
  // 源数据
  StParam.pSrcData = p_data_;                    // 原始图像数据
  StParam.nSrcDataLen = st_img_info_.nFrameLen;  // 原始图像数据长度
  StParam.enSrcPixelType = st_img_info_.enPixelType;  // 原始图像数据的像素格式
  StParam.nWidth = st_img_info_.nWidth;               // 图像宽
  StParam.nHeight = st_img_info_.nHeight;             // 图像高
  // 目标数据pImage
  StParam.enDstPixelType =
      PixelType_Gvsp_BGR8_Packed;  // 需要保存的像素格式类型，转换成BGR格式
  StParam.nDstBufferSize =
      st_img_info_.nWidth * st_img_info_.nHeight * 3;  // 存储节点的大小
  image_msg.data.resize(st_img_info_.nWidth * st_img_info_.nHeight * 3);
  StParam.pDstBuffer = image_msg.data.data();
  n_ret_ = MV_CC_ConvertPixelType(handle_, &StParam);  // 转换像素
  if (n_ret_ != MV_OK) {
    TDT_ERROR("MV_CC_ConvertPixelType failed! n_ret_ [%x]\n", n_ret_);
    return false;
  }
  image_msg.height = st_img_info_.nHeight;
  image_msg.width = st_img_info_.nWidth;
  image_msg.step = st_img_info_.nWidth * 3;
  image_msg.encoding = "bgr8";
  image_msg.header.stamp =
      tdttoolkit::Time::GetRosTimeByTime(triggerTime_ + expose_time_ / 2);
  cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3,
                image_msg.data.data());

  // if (open_lut_) {
  LUT(frame, lut_, frame);
  // } else {
  //     frame.copyTo(img);
  // // }
  // cv::imshow("hik", frame);
  // cv::waitKey(1);
  // ret_time = TriggerTime + expose_time_ / 2;
  // free(p_image);
  // p_image = NULL;

  return true;
}

bool HikvisionCam::RestartCamera() {
  bool ret = true;
  TDT_INFO("===RESTART HIKVISION CAMERA===");
  ret &= CloseGrabbing();
  ret &= CloseCamera();
  int n_ret_ = 0;
  n_ret_ = MV_CC_DestroyHandle(handle_);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_DestroyHandle_ fail! n_ret_ [%x]\n", n_ret_);
  }

  ret &= InitHandle();
  ret &= OpenCamera();
  ret &= StartGrabbing();
  if (ret) {
    return true;
  } else {
    return false;
  }
}

bool HikvisionCam::SetExposureAuto(bool if_auto) {
  int n_ret_ = 0;
  if (if_auto) {
    n_ret_ = MV_CC_SetEnumValue(handle_, "ExposureAuto",
                                MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetExposureAuto fail! n_ret_ [%x]\n", n_ret_);
      MVCC_ENUMVALUE CurrentValue = {0};
      n_ret_ = MV_CC_GetEnumValue(handle_, "ExposureAuto", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
      return false;
    }
    expose_time_ = -1;
  } else {
    n_ret_ =
        MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetExposureAuto fail! n_ret_ [%x]\n", n_ret_);
      MVCC_ENUMVALUE CurrentValue = {0};
      n_ret_ = MV_CC_GetEnumValue(handle_, "ExposureAuto", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetExposure(float value) {
  int n_ret_ = 0;
  n_ret_ = MV_CC_SetFloatValue(handle_, "ExposureTime", value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetExposureTime fail! n_ret_ [%x]\n", n_ret_);
    MVCC_FLOATVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetFloatValue(handle_, "ExposureTime", &CurrentValue);
    TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
    return false;
  }
  expose_time_ = value;
  return true;
}

bool HikvisionCam::DoExposure() {
  int n_ret_ = 0;

  n_ret_ =
      MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_ONCE);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
    MVCC_ENUMVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetEnumValue(handle_, "ExposureAuto", &CurrentValue);
    TDT_ERROR("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetGainAuto(bool if_auto) {
  int n_ret_ = 0;

  if (if_auto) {
    n_ret_ = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetGainAuto fail! n_ret_ [%x]\n", n_ret_);
      MVCC_ENUMVALUE CurrentValue = {0};
      n_ret_ = MV_CC_GetEnumValue(handle_, "GainAuto", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetGainAuto fail! n_ret_ [%x]\n", n_ret_);
      MVCC_ENUMVALUE CurrentValue = {0};
      n_ret_ = MV_CC_GetEnumValue(handle_, "GainAuto", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetGain(float value) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetFloatValue(handle_, "Gain", value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetGain fail! n_ret_ [%x]\n", n_ret_);
    MVCC_FLOATVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetFloatValue(handle_, "Gain", &CurrentValue);
    TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetBrightness(uint32_t value) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetIntValue(handle_, "Brightness", value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBrightness fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "Brightness", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetWhitebalanceAuto(bool if_auto) {
  int n_ret_ = 0;

  if (if_auto) {
    n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto",
                                MV_BALANCEWHITE_AUTO_CONTINUOUS);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
      MVCC_ENUMVALUE CurrentValue = {0};
      n_ret_ = MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto",
                                MV_BALANCEWHITE_AUTO_OFF);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
      MVCC_ENUMVALUE CurrentValue = {0};
      n_ret_ = MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetWhitebalance(uint32_t r, uint32_t g, uint32_t b) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", 0);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
    MVCC_ENUMVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetEnumValue(handle_, "BalanceRatioSelector", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  n_ret_ = MV_CC_SetIntValue(handle_, "BalanceRatio", r);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBalanceRatioRed fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "BalanceRatio", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }

  n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", 1);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
    MVCC_ENUMVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetEnumValue(handle_, "BalanceRatioSelector", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  n_ret_ = MV_CC_SetIntValue(handle_, "BalanceRatio", g);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBalanceRatioGreen fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "BalanceRatio", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }

  n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", 2);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
    MVCC_ENUMVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetEnumValue(handle_, "BalanceRatioSelector", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  n_ret_ = MV_CC_SetIntValue(handle_, "BalanceRatio", b);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBalanceRatioBlue fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "BalanceRatio", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::DoWhitebalance() {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto",
                              MV_BALANCEWHITE_AUTO_ONCE);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_DoBalanceWhite fail! n_ret_ [%x]\n", n_ret_);
    MVCC_ENUMVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetHueDisable(bool if_disable) {
  int n_ret_ = 0;

  if (if_disable) {
    n_ret_ = MV_CC_SetBoolValue(handle_, "HueEnable", false);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetHueEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "HueEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetBoolValue(handle_, "HueEnable", true);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetHueEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "HueEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetHue(uint32_t value) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetIntValue(handle_, "Hue", (int)value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetHue fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "Hue", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetSaturationDisable(bool if_disable) {
  int n_ret_ = 0;

  if (if_disable) {
    n_ret_ = MV_CC_SetBoolValue(handle_, "SaturationEnable", false);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetSaturationEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "SaturationEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetBoolValue(handle_, "SaturationEnable", true);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetSaturationEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "SaturationEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetSaturation(uint32_t value) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetIntValue(handle_, "Saturation", value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetSaturation fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "Saturation", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetGammaDisable(bool if_disable) {
  int n_ret_ = 0;

  MVCC_ENUMVALUE CurrentValue = {0};
  n_ret_ = MV_CC_GetEnumValue(handle_, "PixelFormat", &CurrentValue);

  if (if_disable) {
    if (CurrentValue.nCurValue == PixelType_Gvsp_BayerRG8) {
      SetLut(-1, 0);
    } else {
      n_ret_ = MV_CC_SetBoolValue(handle_, "GammaEnable", false);
      if (MV_OK != n_ret_) {
        TDT_ERROR("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
        bool CurrentValue;
        n_ret_ = MV_CC_GetBoolValue(handle_, "GammaEnable", &CurrentValue);
        TDT_INFO("current value: %d\n", CurrentValue);
        return false;
      }
    }
  } else {
    if (CurrentValue.nCurValue != PixelType_Gvsp_BayerRG8) {
      n_ret_ = MV_CC_SetBoolValue(handle_, "GammaEnable", true);
      if (MV_OK != n_ret_) {
        TDT_ERROR("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
        bool CurrentValue;
        n_ret_ = MV_CC_GetBoolValue(handle_, "GammaEnable", &CurrentValue);
        TDT_INFO("current value: %d\n", CurrentValue);
        return false;
      }
    }
  }
  return true;
}

bool HikvisionCam::SetGamma(float value, unsigned char selector) {
  int n_ret_ = 0;

  MVCC_ENUMVALUE CurrentValue = {0};
  n_ret_ = MV_CC_GetEnumValue(handle_, "PixelFormat", &CurrentValue);
  SetLut(value, 0);
  // if (CurrentValue.nCurValue == PixelType_Gvsp_BayerRG8) {
  //   SetLut(value, 0);
  // }

  // else {
  //   if (selector == MV_GAMMA_SELECTOR_SRGB) {
  //     n_ret_ =
  //         MV_CC_SetEnumValue(handle_, "GammaSelector",
  //         MV_GAMMA_SELECTOR_SRGB);
  //     if (MV_OK != n_ret_) {
  //       TDT_ERROR("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
  //       MVCC_ENUMVALUE CurrentValue = {0};
  //       n_ret_ = MV_CC_GetEnumValue(handle_, "GammaSelector", &CurrentValue);
  //       TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
  //       return false;
  //     }
  //   } else {
  //     n_ret_ =
  //         MV_CC_SetEnumValue(handle_, "GammaSelector",
  //         MV_GAMMA_SELECTOR_USER);
  //     if (MV_OK != n_ret_) {
  //       TDT_ERROR("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
  //       MVCC_ENUMVALUE CurrentValue = {0};
  //       n_ret_ = MV_CC_GetEnumValue(handle_, "GammaSelector", &CurrentValue);
  //       TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
  //       return false;
  //     }
  //     n_ret_ = MV_CC_SetFloatValue(handle_, "Gamma", value);
  //     if (MV_OK != n_ret_) {
  //       TDT_ERROR("MV_CC_SetGamma fail! n_ret_ [%x]\n", n_ret_);
  //       MVCC_FLOATVALUE CurrentValue = {0};
  //       n_ret_ = MV_CC_GetFloatValue(handle_, "Gamma", &CurrentValue);
  //       TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
  //       return false;
  //     }
  //   }
  // }
  return true;
}

bool HikvisionCam::SetSharpnessDisable(bool if_disable) {
  int n_ret_ = 0;

  if (if_disable) {
    n_ret_ = MV_CC_SetBoolValue(handle_, "SharpnessEnable", false);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetSharpnessEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "SharpnessEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetBoolValue(handle_, "SharpnessEnable", true);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetSharpnessEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "SharpnessEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetSharpness(uint32_t value) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetIntValue(handle_, "Sharpness", value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetSharpness fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "Sharpness", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetBlacklevelDisable(bool if_disable) {
  int n_ret_ = 0;

  if (if_disable) {
    n_ret_ = MV_CC_SetBoolValue(handle_, "BlackLevelEnable", false);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetBlackLevelEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "BlackLevelEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetBoolValue(handle_, "BlackLevelEnable", true);
    if (MV_OK != n_ret_) {
      TDT_ERROR("MV_CC_SetBlackLevelEnable fail! n_ret_ [%x]\n", n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "BlackLevelEnable", &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetBlacklevel(uint32_t value) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetIntValue(handle_, "BlackLevel", value);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetBlackLevel fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "BlackLevel", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetResolution(uint32_t width, uint32_t height) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetIntValue(handle_, "Width", width);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetWidth fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "Width", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }

  n_ret_ = MV_CC_SetIntValue(handle_, "Height", height);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetHeight fail! n_ret_ [%x]\n", n_ret_);
    MVCC_INTVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "Height", &CurrentValue);
    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetFpsDisable(bool if_disable) {
  int n_ret_ = 0;

  if (if_disable) {
    n_ret_ = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);
    if (MV_OK != n_ret_) {
      TDT_ERROR(
          "MV_CC_SetAcquisitionFrameRateEnable fail! n_ret_ "
          "[%x]\n",
          n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable",
                                  &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  } else {
    n_ret_ = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    if (MV_OK != n_ret_) {
      TDT_ERROR(
          "MV_CC_SetAcquisitionFrameRateEnable fail! n_ret_ "
          "[%x]\n",
          n_ret_);
      bool CurrentValue;
      n_ret_ = MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable",
                                  &CurrentValue);
      TDT_INFO("current value: %d\n", CurrentValue);
      return false;
    }
  }
  return true;
}

bool HikvisionCam::SetFps(float fps) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetAcquisitionFrameRate fail! n_ret_ [%x]\n", n_ret_);
    MVCC_FLOATVALUE CurrentValue = {0};
    n_ret_ =
        MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &CurrentValue);
    TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
    return false;
  }
  return true;
}

bool HikvisionCam::SetTriggerMode(uint32_t isTrigger) {
  int n_ret_ = 0;
  n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerMode",
                              (_MV_CAM_TRIGGER_MODE_)isTrigger);

  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetTriggerSource(uint32_t triggerSource) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerSource",
                              (_MV_CAM_TRIGGER_SOURCE_)triggerSource);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerSource fail! nRet [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetTriggerActivation(uint32_t triggerActivation) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerActivation", triggerActivation);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerActivation fail! nRet [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetTriggerDelay(float delay) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetFloatValue(handle_, "TriggerDelay", delay);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerDelay fail! nRet [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetTriggerSelector(uint32_t triggerSelector) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerSelector", triggerSelector);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerSelector fail! nRet [%x]\n", n_ret_);
    return false;
  };
  return true;
}

bool HikvisionCam::SetTriggerCache(uint32_t triggerCache) {
  int n_ret_ = 0;

  n_ret_ =
      MV_CC_SetBoolValue(handle_, "TriggerCacheEnable", (bool)triggerCache);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerCache fail! nRet [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetImageNodeNum(uint32_t nodeNum) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetImageNodeNum(handle_, nodeNum);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetTriggerCache fail! nRet [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetPixelformat(uint32_t pixelformat) {
  int n_ret_ = 0;

  n_ret_ = MV_CC_SetEnumValue(handle_, "PixelFormat", pixelformat);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_SetPixelFormat fail! n_ret_ [%x]\n", n_ret_);
    return false;
  }
  return true;
}

// void HikvisionCam::TakeFrame() {
// while (true) {
// static uint64_t last_tick = cv::getTickCount();

// TDT_INFO("相机帧数FPS %lf",
//          cv::getTickFrequency() / (cv::getTickCount() - last_tick));

// last_tick = cv::getTickCount();
// std::unique_lock<std::mutex> locker_(stop_mtx_);
// if (Stop) {
//   std::cout << "stop" << std::endl;
//   break;
// }
// if (setting_.trigger_mode) {
//   if (!GetMat_Triger(takeImg_, takeTime_, setting_.trigger_source)) {
//     TDT_ERROR("Failed to get image from hikvision cam");
//   }
// } else {
//   if (!GetMat(takeImg_, takeTime_, setting_.get_timeStamp)) {
//     TDT_ERROR("Failed to get image from hikvision cam");
//   }
// }
// locker_.unlock();

// std::unique_lock<std::mutex> locker(img_mtx_);
// swap(takeImg_, swapImg_);
// swapTime_ = takeTime_;
// locker.unlock();
// condVar_.notify_all();
//   }
// }

bool HikvisionCam::CloseGrabbing() {
  int n_ret_ = MV_CC_StopGrabbing(handle_);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_StopGrabbing fail! n_ret_ [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::sendTrigger() {
  int n_ret_ = MV_CC_SetCommandValue(handle_, "TriggerSoftware");
  if (MV_OK != n_ret_) {
    TDT_ERROR("failed in TriggerSoftware[%x]\n", n_ret_);
    return false;
  }
  triggerTime_ = tdttoolkit::Time::GetTimeNow();
  return true;
}

bool HikvisionCam::SetReverse_X(bool x) {
  // auto A = std::make_shared<MV_CC_FLIP_IMAGE_PARAM>();
  // A->enFlipType = MV_FLIP_HORIZONTAL;
  int n_ret_ = MV_CC_SetBoolValue(handle_, "ReverseX", x);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_FlipImage fail! n_ret_ [%x]\n", n_ret_);
    return false;
  }
  return true;
}

bool HikvisionCam::SetReverse_Y(bool y) {
  // auto A = MV_CC_FLIP_IMAGE_PARAM();
  // A->enFlipType = MV_FLIP_HORIZONTAL;
  int n_ret_ = MV_CC_SetBoolValue(handle_, "ReverseY", y);
  if (MV_OK != n_ret_) {
    TDT_ERROR("MV_CC_FlipImage fail! n_ret_ [%x]\n", n_ret_);
    return false;
  }
  return true;
}

}  // namespace tdt_vision