#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

namespace tdt_radar {

const cv::Size BOARD_SIZE = cv::Size(11, 8);         // 内部角点数量
const float SQUARE_SIZE = 0.025f;                    // 标定板方格边长 (单位：米)
const cv::Vec3d AXIS_OFFSET(0.0, -0.0429, 0.15175);  // 机械结构参数

class CalibrateFlyOpenCV {
public:
    // 构造函数：传入相机内参配置文件路径
    explicit CalibrateFlyOpenCV(const std::string& config_path);

    // 执行标定主流程：传入保存标定数据的文件夹路径
    void run(const std::string& data_folder);

private:
    // 核心处理函数
    void load_and_process_data(const std::string& folder_path);
    void save_sample(const cv::Mat& img, std::vector<cv::Point2f>& corners);
    void run_calibration();
    cv::Mat getGimbalPose(double yaw_deg, double pitch_deg);

    // 成员变量
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    float yaw;
    float pitch;

    std::vector<cv::Mat> R_base2gripper; // 云台姿态容器
    std::vector<cv::Mat> t_base2gripper; // 云台平移容器
    std::vector<cv::Mat> R_target2cam;   // 相机姿态容器
    std::vector<cv::Mat> t_target2cam;   // 相机平移容器
};

} // namespace tdt_radar