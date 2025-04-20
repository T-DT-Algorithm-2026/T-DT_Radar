// 海康相机类
#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

#include "baseCamera.h"

// STL
#include <cstdint>

namespace tdt_vision {

/**
 * 相机设置
 */
enum TDT_CAMERA_SETTING {
  TDT_CAMERA_SETTING_EXPOSURE = 0,
  TDT_CAMERA_SETTING_GAIN = 1,
  TDT_CAMERA_SETTING_BRIGHTNESS = 2,
  TDT_CAMERA_SETTING_BALANCE_VAL = 3,
  TDT_CAMERA_SETTING_BALANCE_RED = 4,
  TDT_CAMERA_SETTING_BALANCE_GREEN = 5,
  TDT_CAMERA_SETTING_BALANCE_BLUE = 6,
  TDT_CAMERA_SETTING_HUE = 7,
  TDT_CAMERA_SETTING_SATURATION = 8,
  TDT_CAMERA_SETTING_CONTRAST = 9,
  TDT_CAMERA_SETTING_GAMMA = 10,
  TDT_CAMERA_SETTING_SHARPNESS = 11,
  TDT_CAMERA_SETTING_BLACK_LEVEL = 12,
  TDT_CAMERA_SETTING_TRIGER_MODE = 13,
  TDT_CAMERA_SETTING_TRIGER_SOURCE = 14,
  TDT_CAMERA_SETTING_TRIGER_ACTIVATION = 15,
  TDT_CAMERA_SETTING_TRIGER_DELAY = 16,
  TDT_CAMERA_SETTING_TRIGGER_SELECTOR = 17,
  TDT_CAMERA_SETTING_TRIGGER_CACHE = 18,
  TDT_CAMERA_SETTING_IMAGE_NODE_NUM = 19,
  TDT_CAMERA_SETTING_REVERSE_X = 20,
  TDT_CAMERA_SETTING_REVERSE_Y = 21
};

class HikvisionCam final : public Camera {
 public:
  HikvisionCam(std::string config_name, rclcpp::Node::SharedPtr camera_node);

  HikvisionCam() = default;

  /*
  @brief:发布ros消息
  */
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  void publishCameraInfo() override;

  /*
  @brief:读取相机参数
  */
  bool readCameraParam(std::string config_name);

  /*
  @brief:设置相机参数
  */
  void setCameraParam(bool ifdone = false);

  /*
  @brief:根据guid初始化句柄操作,只有一个相机插入时可忽略guid。
  多个相机强制使用guid初始化句柄，否则TDT_FATAL .
  TODO:多相机的冗余配置没有完成
  */
  bool InitHandle();

  /*
  @brief: 打开相机
  */
  bool OpenCamera() override;

  /*
  @brief：关闭相机
  */
  bool CloseCamera() override;

  /*
  @brief:暂定取帧，不占用资源
  @param: ifPause: true 暂停取帧，false 恢复取帧
  */
  void pause(bool ifPause = false);

  /*
   */
  void SetLut(float val, int mode);

  /*
  @brief:开始采集图像
  */
  bool StartGrabbing();

  /*
  @brief:获取图像
  */
  void GetImage(sensor_msgs::msg::Image& image_) override;

  /*
  @brief:重启相机
  */
  bool RestartCamera();

  /*
  @brief:设计相机触发模式
  */
  void setCameraTrigger(bool ifTrigger);

  bool GetMat_Triger(
      sensor_msgs::msg::Image& image_msg,
      int trigger_source =
          MV_TRIGGER_SOURCE_SOFTWARE);  // 触发取帧获取Opencv Mat图像

  bool GetMat(sensor_msgs::msg::Image& image_msg,
              bool get_timeStamp = false);  // 非触发取帧获取Opencv Mat图像

  template <class T>
  void Set(TDT_CAMERA_SETTING tdt_camera_setting, const T& value);

  template <class T>
  void Set(TDT_CAMERA_FORMAT tdt_camera_format, const T& value);

  bool SetExposureAuto(bool if_auto);
  bool SetExposure(float val);  // 设置曝光
  bool DoExposure();            // 单帧自动曝光
  bool SetGainAuto(bool if_auto);
  bool SetGain(float val);           // 设置增益
  bool SetBrightness(uint32_t val);  // 设置亮度

  bool SetWhitebalanceAuto(bool if_auto);
  bool SetWhitebalance(uint32_t r, uint32_t g, uint32_t b);  // 设置白平衡
  bool DoWhitebalance();  // 单帧自动白平衡
  bool SetHueDisable(bool if_disable);
  bool SetHue(uint32_t val);  // 设置色调
  bool SetSaturationDisable(bool if_disable);
  bool SetSaturation(uint32_t val);  // 设置饱和度

  bool SetGammaDisable(bool if_disable);
  bool SetGamma(float val = 1. / 2.2,
                unsigned char selector = MV_GAMMA_SELECTOR_SRGB);  // 设置伽玛
  bool SetSharpnessDisable(bool if_disable);
  bool SetSharpness(uint32_t val);  // 设置锐度
  bool SetBlacklevelDisable(bool if_disable);
  bool SetBlacklevel(uint32_t val);  // 设置黑位

  bool SetPixelformat(uint32_t pixelformat = PixelType_Gvsp_RGB8_Packed);
  bool SetResolution(uint32_t width, uint32_t height);
  bool SetFpsDisable(bool if_disable);
  bool SetFps(float fps);

  bool SetTriggerMode(uint32_t isTrigger);
  bool SetTriggerSource(uint32_t triggerSource);
  bool SetTriggerActivation(uint32_t triggerActivation);
  bool SetTriggerDelay(float triggerDelay);
  bool SetTriggerSelector(uint32_t triggerSelector);
  bool SetTriggerCache(uint32_t triggerCache);
  bool SetImageNodeNum(uint32_t imageNodeNum);
  bool sendTrigger();
  bool SetReverse_X(bool x);
  bool SetReverse_Y(bool y);
  /*
  @brief:获取相机guid
  */
  inline std::string getGuid() { return identity_.guid; };

  // void TakeFrame(sensor_msgs::msg::Image &image_msg);

  bool CloseGrabbing();

 private:
  struct CameraSetting {
    float exposure;
    float gain;
    int brightness;
    int balance_val;
    int balance_red;
    int balance_green;
    int balance_blue;
    int hue;
    int saturation;
    int contrast;
    float gamma;
    int sharpness;
    int black_level;
    int trigger_mode;
    bool get_timeStamp;
    int trigger_source;
    int trigger_activation;
    float trigger_delay;
    int trigger_selector;
    bool trigger_cache;
    int image_node_num;
    bool reverse_x;
    bool reverse_y;
    cv::Mat camera_matrix;
    cv::Mat dist_coeff;
  } setting_;

  MV_CC_DEVICE_INFO_LIST m_stDevList;  //设备列表信息结构体

  void* handle_ = nullptr;  //设备句柄

  cv::Mat lut_;

  uint32_t n_data_size_ = 0;

  unsigned char* p_data_ = NULL;

  MV_FRAME_OUT_INFO_EX st_img_info_ = {0};

  // MV_FRAME_OUT img_out_;

  float expose_time_ = 1000;

  bool open_lut_ = true;

  double time_diff = 0;  // 程序时间 - 相机时间

  uint32_t seq_ = 0;
};

}  // namespace tdt_vision

#endif