#include "calibrate_fly_opencv.h"

namespace tdt_radar {

CalibrateFlyOpenCV::CalibrateFlyOpenCV(const std::string& config_path)
{
    cv::namedWindow("Calibration Preview", cv::WINDOW_NORMAL);

    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) 
    {
        std::cerr << "Failed to open YAML config: " << config_path << std::endl;
        exit(1);
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["dist_coeff"] >> distCoeffs;
    fs.release();
    
    std::cout << "Loaded camera parameters:\n"
              << "Camera Matrix:\n" << cameraMatrix 
              << "\nDistortion Coefficients:\n" << distCoeffs << std::endl;
}

void CalibrateFlyOpenCV::run(const std::string& data_folder)
{
    load_and_process_data(data_folder);
    run_calibration();
}

void CalibrateFlyOpenCV::load_and_process_data(const std::string& folder_path)
{
    // 使用 glob 获取文件夹下所有的 .png 文件路径，无视乱序或跳跃
    std::vector<cv::String> image_files;
    cv::glob(folder_path + "/*.png", image_files, false);
    
    if (image_files.empty()) {
        std::cerr << "错误：在文件夹 " << folder_path << " 中没有找到任何 .png 文件！" << std::endl;
        return;
    }

    int valid_count = 0;

    for (const auto& img_filename : image_files) 
    {
        std::string base_filename = img_filename.substr(0, img_filename.find_last_of('.'));
        std::string txt_filename = base_filename + ".txt";

        cv::Mat img = cv::imread(img_filename);
        if (img.empty()) continue; 

        // 读取对应的 txt 文件
        std::ifstream in_file(txt_filename);
        if (in_file.is_open()) {
            std::string key;
            in_file >> key >> yaw;
            in_file >> key >> pitch;
            in_file.close();
        } else {
            std::cerr << "警告: 找到图片 " << img_filename << " 但缺失对应的 txt 文件，跳过此组。" << std::endl;
            continue; 
        }

        std::string short_name = img_filename.substr(img_filename.find_last_of('/') + 1);
        std::cout << "\n处理文件: " << short_name << " - Yaw: " << yaw << "°, Pitch: " << pitch << "°" << std::endl;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        
        cv::Mat T_g = getGimbalPose(yaw, pitch);
        
        bool found = cv::findChessboardCorners(gray, BOARD_SIZE, corners);

        if (found) 
        {
            // 保存基座到云台的相对位姿 (深拷贝)
            R_base2gripper.push_back(T_g(cv::Rect(0, 0, 3, 3)).clone());
            t_base2gripper.push_back(T_g(cv::Rect(3, 0, 1, 3)).clone());
            
            // 计算相机到标定板的相对位姿
            save_sample(img, corners);
            
            valid_count++;
            std::cout << "采样成功！当前有效样本数: " << valid_count << std::endl;
            
            cv::Mat debug_img = img.clone();
            cv::drawChessboardCorners(debug_img, BOARD_SIZE, corners, found);
            cv::imshow("Calibration Preview", debug_img);
            cv::waitKey(50); // 暂停显示一会儿过程
        } 
        else 
        {
            std::cerr << "警告: 图片 " << short_name << " 未找到标定板！" << std::endl;
        }
    }
    
    std::cout << "\n遍历结束，共处理了 " << valid_count << " 组有效数据。" << std::endl;
}

void CalibrateFlyOpenCV::save_sample(const cv::Mat& img, std::vector<cv::Point2f>& corners)
{
    // A. 亚像素精细化
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::find4QuadCornerSubpix(gray, corners, cv::Size(5, 5));

    std::vector<cv::Point2f> pnp_points;
    pnp_points.push_back(corners[2]);   //(0,2)
    pnp_points.push_back(corners[4]);   //(0,4)
    pnp_points.push_back(corners[6]);   //(0,6)
    pnp_points.push_back(corners[79]);  //(10,2)
    pnp_points.push_back(corners[81]);  //(10,4)
    pnp_points.push_back(corners[83]);  //(10,6)
    
    std::vector<cv::Point3f> objPoints;
    objPoints.push_back(cv::Point3f(0.05, 0, 0));
    objPoints.push_back(cv::Point3f(0.1, 0, 0));
    objPoints.push_back(cv::Point3f(0.15, 0, 0));
    objPoints.push_back(cv::Point3f(0.05, 0.175, 0));
    objPoints.push_back(cv::Point3f(0.1, 0.175, 0));
    objPoints.push_back(cv::Point3f(0.15, 0.175, 0));
    std::cout << "用于 PnP 的图像点:\n" << pnp_points[2] << std::endl;

    // B. PnP 求解标定板相对于相机的位姿 (B 矩阵)
    cv::Mat rvec, tvec, R_cam;
    cv::solvePnP(objPoints, pnp_points, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Rodrigues(rvec, R_cam);
    std::cout << "标定板相对于相机的平移向量 t (米):\n" << tvec << std::endl;

    // 存入容器待计算 (深拷贝)
    R_target2cam.push_back(R_cam.clone());
    t_target2cam.push_back(tvec.clone());
}

void CalibrateFlyOpenCV::run_calibration()
{
    if (R_base2gripper.size() < 3) 
    {
        std::cerr << "错误：采样数据不足（当前 " << R_base2gripper.size() << " 组），至少需要3组！" << std::endl;
        return;
    }
    
    cv::Mat R_cam2pitch, t_cam2pitch;
    cv::Mat R_base2board, t_base2board;
    std::cout << "正在执行手眼标定计算..." << std::endl;

    // 执行 OpenCV 内置机器人手眼标定核心函数
    cv::calibrateRobotWorldHandEye(
            R_target2cam, t_target2cam,       // 相机看标定板的绝对位姿 (PnP结果)
            R_base2gripper, t_base2gripper,   // 基座看云台末端的绝对位姿 (正向运动学结果)
            R_base2board, t_base2board,       // 输出Y：基座到标定板的变换
            R_cam2pitch, t_cam2pitch,         // 输出X：Pitch轴到相机的变换 (即相机外参)
            cv::CALIB_ROBOT_WORLD_HAND_EYE_LI
        );

    // 结果输出
    std::cout << "\n==========================================" << std::endl;
    std::cout << "标定完成！(Camera to Pitch Axis)" << std::endl;
    std::cout << "旋转矩阵 R:\n" << R_cam2pitch << std::endl;
    std::cout << "平移向量 t (米):\n" << t_cam2pitch << std::endl;
    std::cout << "偏移距离: " << cv::norm(t_cam2pitch) << " m" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "\n[参考] 标定板相对云台基座的绝对位置:\n"
              << "X: " << t_base2board.at<double>(0) << " m, \n"
              << "Y: " << t_base2board.at<double>(1) << " m, \n"
              << "Z: " << t_base2board.at<double>(2) << " m" << std::endl;
    std::cout << "==========================================" << std::endl;
}

cv::Mat CalibrateFlyOpenCV::getGimbalPose(double yaw_deg, double pitch_deg)
{
    double y = yaw_deg * CV_PI / 180.0;
    double p = pitch_deg * CV_PI / 180.0;

    // Yaw 旋转 (绕 Z 轴)
    cv::Mat R_yaw = (cv::Mat_<double>(3, 3) << cos(y), -sin(y), 0, sin(y),cos(y), 0, 0, 0, 1);
    // Pitch 旋转 (绕 y 轴)
    cv::Mat R_pitch = (cv::Mat_<double>(3, 3) << cos(p), 0, sin(p), 0, 1, 0, -sin(p), 0, cos(p));

    // 构造变换矩阵 T = T_yaw * T_offset * T_pitch
    cv::Mat T_yaw = cv::Mat::eye(4, 4, CV_64F);
    R_yaw.copyTo(T_yaw(cv::Rect(0, 0, 3, 3)));

    cv::Mat T_off = cv::Mat::eye(4, 4, CV_64F);
    T_off.at<double>(0, 3) = AXIS_OFFSET[0];
    T_off.at<double>(1, 3) = AXIS_OFFSET[1];
    T_off.at<double>(2, 3) = AXIS_OFFSET[2];

    cv::Mat T_pitch = cv::Mat::eye(4, 4, CV_64F);
    R_pitch.copyTo(T_pitch(cv::Rect(0, 0, 3, 3)));

    // 最终得到当前云台状态下的机械变换链
    cv::Mat T_final = T_yaw * T_off * T_pitch;
    return T_final;
}

}  // namespace tdt_radar

// ================= 主函数入口 =================
int main(int argc, char** argv) 
{
    // 默认路径，可通过命令行参数修改
    std::string config_path = "./config/camera_param_fly.yaml";
    std::string data_folder = "/home/robot/calib_img";

    if (argc > 1) {
        config_path = argv[1];
    }
    if (argc > 2) {
        data_folder = argv[2];
    }

    std::cout << "====== OpenCV RobotWorld Hand-Eye Calibration ======" << std::endl;
    std::cout << "使用相机配置文件: " << config_path << std::endl;
    std::cout << "读取标定数据文件夹: " << data_folder << std::endl;

    tdt_radar::CalibrateFlyOpenCV calibrator(config_path);
    calibrator.run(data_folder);

    return 0;
}