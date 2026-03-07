#include "calibrate_fly.h"

namespace tdt_radar {

struct RelativeMotionError {
    double a_r[3], a_t[3]; // 云台相对运动 (Angle-Axis 和 平移)
    double b_r[3], b_t[3]; // 相机相对运动 (Angle-Axis 和 平移)

    // 构造函数：传入第 i 帧和第 j 帧的分别的旋转(3x3)和平移(3x1)
    RelativeMotionError(const cv::Mat& R_Ai, const cv::Mat& t_Ai, 
                      const cv::Mat& R_Bi, const cv::Mat& t_Bi) {
        
        cv::Mat rA, rB;
        cv::Rodrigues(R_Ai, rA);
        cv::Rodrigues(R_Bi, rB);

        for(int k = 0; k < 3; ++k) {
            a_r[k] = rA.at<double>(k); 
            a_t[k] = t_Ai.at<double>(k);
            b_r[k] = rB.at<double>(k); 
            b_t[k] = t_Bi.at<double>(k);
        }
    }
    // Ceres 自动微分计算
    template <typename T>
    bool operator()(const T* const cam2pitch_rot,   // 待优化外参：Pitch -> Cam 的旋转 (3)
                    const T* const cam2pitch_trans, // 待优化外参：Pitch -> Cam 的平移 (3)
                    const T* const target_rot,      // 待优化全局变量：Base -> Target 旋转 (3)
                    const T* const target_trans,    // 待优化全局变量：Base -> Target 平移 (3)
                    T* residuals) const {
        // 构造三个不共线的空间基准点
        T basis_pts[3][3] = {
            {T(0), T(0), T(0)},
            {T(1), T(0), T(0)},
            {T(0), T(1), T(0)}
        };

        for (int i = 0; i < 3; ++i) {   
            // ----- 步骤 1：期望位置 (Expectation) -----
            T p_base_expected[3];
            ceres::AngleAxisRotatePoint(target_rot, basis_pts[i], p_base_expected);
            p_base_expected[0] += target_trans[0]; 
            p_base_expected[1] += target_trans[1]; 
            p_base_expected[2] += target_trans[2];

            // ----- 步骤 2：正向推导位置 (Calculation) -----
            // 2.1 标定板 -> 相机 (B_i)
            T p_cam[3];
            T br[3] = {T(b_r[0]), T(b_r[1]), T(b_r[2])};
            ceres::AngleAxisRotatePoint(br, basis_pts[i], p_cam);
            p_cam[0] += T(b_t[0]); 
            p_cam[1] += T(b_t[1]); 
            p_cam[2] += T(b_t[2]);

            // 2.2 相机 -> Pitch轴 (X，待求外参)
            T p_pitch[3];
            ceres::AngleAxisRotatePoint(cam2pitch_rot, p_cam, p_pitch);
            p_pitch[0] += cam2pitch_trans[0]; 
            p_pitch[1] += cam2pitch_trans[1]; 
            p_pitch[2] += cam2pitch_trans[2];

            // 2.3 Pitch轴 -> 基座 (A_i)
            T p_base_calc[3];
            T ar[3] = {T(a_r[0]), T(a_r[1]), T(a_r[2])};
            ceres::AngleAxisRotatePoint(ar, p_pitch, p_base_calc);
            p_base_calc[0] += T(a_t[0]); 
            p_base_calc[1] += T(a_t[1]); 
            p_base_calc[2] += T(a_t[2]);

            // ----- 步骤 3：计算欧氏距离残差 -----
            residuals[i*3 + 0] = p_base_calc[0] - p_base_expected[0];
            residuals[i*3 + 1] = p_base_calc[1] - p_base_expected[1];
            residuals[i*3 + 2] = p_base_calc[2] - p_base_expected[2];
        }
        return true;
    }
};

CalibrateFly::CalibrateFly(const std::string& config_path)
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

void CalibrateFly::run(const std::string& data_folder)
{
    load_and_process_data(data_folder);
    run_calibration();
}

void CalibrateFly::load_and_process_data(const std::string& folder_path)
{
    // 1. 使用 glob 获取文件夹下所有的 .png 文件路径
    std::vector<cv::String> image_files;
    cv::glob(folder_path + "/*.png", image_files, false);
    
    if (image_files.empty()) {
        std::cerr << "错误：在文件夹 " << folder_path << " 中没有找到任何 .png 文件！" << std::endl;
        return;
    }

    int valid_count = 0;

    // 2. 遍历每一个找到的图片文件（不在乎文件名是否连续）
    for (const auto& img_filename : image_files) 
    {
        // 提取去掉后缀的基础路径，用来寻找同名的 txt (例如: "/home/robot/calib_img/5")
        std::string base_filename = img_filename.substr(0, img_filename.find_last_of('.'));
        std::string txt_filename = base_filename + ".txt";

        cv::Mat img = cv::imread(img_filename);
        if (img.empty()) continue; // 防御性编程

        // 3. 读取对应的 txt 文件
        std::ifstream in_file(txt_filename);
        if (in_file.is_open()) {
            std::string key;
            in_file >> key >> yaw;
            in_file >> key >> pitch;
            in_file.close();
        } else {
            std::cerr << "警告: 找到图片 " << img_filename << " 但缺失对应的 txt 文件，跳过此组。" << std::endl;
            continue; // 找不到 txt 就跳过这张图
        }

        // 为了控制台输出好看，我们只打印文件名部分
        std::string short_name = img_filename.substr(img_filename.find_last_of('/') + 1);
        std::cout << "\n处理文件: " << short_name << " - Yaw: " << yaw << "°, Pitch: " << pitch << "°" << std::endl;

        // 4. 寻找角点与处理
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        
        cv::Mat T_g = getGimbalPose(yaw, pitch);
        
        bool found = cv::findChessboardCorners(gray, BOARD_SIZE, corners);

        if (found) 
        { 
            R_base2gripper.push_back(T_g(cv::Rect(0, 0, 3, 3)).clone());
            t_base2gripper.push_back(T_g(cv::Rect(3, 0, 1, 3)).clone());
            
            save_sample(img, corners);
            
            valid_count++;
            std::cout << "采样成功！当前有效样本数: " << valid_count << std::endl;
            
            cv::Mat debug_img = img.clone();
            cv::drawChessboardCorners(debug_img, BOARD_SIZE, corners, found);
            cv::imshow("Calibration Preview", debug_img);
            cv::waitKey(50); 
        } 
        else 
        {
            std::cerr << "警告: 图片 " << short_name << " 未找到标定板！" << std::endl;
        }
    }
    
    std::cout << "\n遍历结束，共处理了 " << valid_count << " 组有效数据。" << std::endl;
}

void CalibrateFly::save_sample(const cv::Mat& img, std::vector<cv::Point2f>& corners)
{
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

    cv::Mat rvec, tvec, R_cam;
    cv::solvePnP(objPoints, pnp_points, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Rodrigues(rvec, R_cam);
    std::cout << "pnp结果 t (米):\n" << tvec << std::endl;
    std::cout << "pnp距离 (米): " << cv::norm(tvec) << std::endl;

    R_target2cam.push_back(R_cam.clone());
    t_target2cam.push_back(tvec.clone());
}

void CalibrateFly::run_calibration()
{
    if (R_base2gripper.size() < 3) 
    {
        std::cerr << "错误：有效采样数据不足（当前 " << R_base2gripper.size() << " 组），至少需要3组！" << std::endl;
        return;
    }
    
    // 1. 初始化旋转初值
    cv::Mat R_ideal = (cv::Mat_<double>(3,3) << 
        0,  0,  1, 
       -1,  0,  0, 
        0, -1,  0);
    cv::Mat rvec_ideal; 
    cv::Rodrigues(R_ideal, rvec_ideal);
    double cam2pitch_rot[3] = {rvec_ideal.at<double>(0), rvec_ideal.at<double>(1), rvec_ideal.at<double>(2)};
    
    // 【必填区】: 填入图纸上相机光心距离 Pitch 轴心在各方向的物理平移 (单位: 米)
    double cam2pitch_trans[3] = {0.064974, 0.0534, 0.04785}; 

    // 2. 标定板全局位姿
    cv::Mat T_A0 = cv::Mat::eye(4,4,CV_64F);
    R_base2gripper[0].copyTo(T_A0(cv::Rect(0,0,3,3)));
    t_base2gripper[0].copyTo(T_A0(cv::Rect(3,0,1,3)));

    cv::Mat T_X = cv::Mat::eye(4,4,CV_64F);
    R_ideal.copyTo(T_X(cv::Rect(0,0,3,3)));
    T_X.at<double>(0,3) = cam2pitch_trans[0];
    T_X.at<double>(1,3) = cam2pitch_trans[1];
    T_X.at<double>(2,3) = cam2pitch_trans[2];

    cv::Mat T_B0 = cv::Mat::eye(4,4,CV_64F);
    R_target2cam[0].copyTo(T_B0(cv::Rect(0,0,3,3)));
    t_target2cam[0].copyTo(T_B0(cv::Rect(3,0,1,3)));

    cv::Mat T_target = T_A0 * T_X * T_B0;
    cv::Mat rvec_target;
    cv::Rodrigues(T_target(cv::Rect(0,0,3,3)), rvec_target);
    
    double target_rot[3] = {rvec_target.at<double>(0), rvec_target.at<double>(1), rvec_target.at<double>(2)};
    double target_trans[3] = {20.7, 0.15, -0.36}; 
    std::cout << "初始板子位置猜测 (米): X: " << target_trans[0] << ", Y: " << target_trans[1] << ", Z: " << target_trans[2] << std::endl;

    ceres::Problem problem;

    for (size_t i = 0; i < R_base2gripper.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<RelativeMotionError, 9, 3, 3, 3, 3>(
                new RelativeMotionError(
                    R_base2gripper[i], t_base2gripper[i], 
                    R_target2cam[i], t_target2cam[i]
                ));
        
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.05), 
                                 cam2pitch_rot, cam2pitch_trans,
                                 target_rot, target_trans);
    }
    std::cout << "成功构建了 " << R_base2gripper.size() << " 组绝对姿态图优化网络" << std::endl;

    double bound1 = 0.1;
    double bound2 = 0.5; 
    problem.SetParameterLowerBound(cam2pitch_trans, 0, cam2pitch_trans[0] - bound1);
    problem.SetParameterUpperBound(cam2pitch_trans, 0, cam2pitch_trans[0] + bound1);
    problem.SetParameterLowerBound(cam2pitch_trans, 1, cam2pitch_trans[1] - bound2);
    problem.SetParameterUpperBound(cam2pitch_trans, 1, cam2pitch_trans[1] + bound2);
    problem.SetParameterLowerBound(cam2pitch_trans, 2, cam2pitch_trans[2] - bound2);
    problem.SetParameterUpperBound(cam2pitch_trans, 2, cam2pitch_trans[2] + bound2);
    
    double bound_target1 = 0.1; 
    double bound_target2 = 0.1;
    problem.SetParameterLowerBound(target_trans, 0, target_trans[0] - bound_target1);
    problem.SetParameterUpperBound(target_trans, 0, target_trans[0] + bound_target1);
    problem.SetParameterLowerBound(target_trans, 1, target_trans[1] - bound_target2);
    problem.SetParameterUpperBound(target_trans, 1, target_trans[1] + bound_target2);
    problem.SetParameterLowerBound(target_trans, 2, target_trans[2] - bound_target2);
    problem.SetParameterUpperBound(target_trans, 2, target_trans[2] + bound_target2);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cv::Mat R_opt;
    cv::Mat rvec_opt = (cv::Mat_<double>(3, 1) << cam2pitch_rot[0], cam2pitch_rot[1], cam2pitch_rot[2]);
    cv::Rodrigues(rvec_opt, R_opt);
    cv::Mat t_opt = (cv::Mat_<double>(3, 1) << cam2pitch_trans[0], cam2pitch_trans[1], cam2pitch_trans[2]);

    std::cout << "\n==========================================" << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    std::cout << ">>> 基于绝对姿态优化的长焦标定完成！ <<<" << std::endl;
    std::cout << "优化后的旋转矩阵 R (Pitch -> Camera):\n" << R_opt << std::endl;
    std::cout << "优化后的平移向量 t (米):\n" << t_opt << std::endl;
    
    std::cout << "\n[参考] 标定板相对云台基座的绝对位置:\n"
              << "X: " << target_trans[0] << "m, Y: " << target_trans[1] 
              << "m, Z: " << target_trans[2] << "m" << std::endl;
    std::cout << "==========================================" << std::endl;
}

cv::Mat CalibrateFly::getGimbalPose(double yaw_deg, double pitch_deg)
{
    double y = yaw_deg * CV_PI / 180.0;
    double p = pitch_deg * CV_PI / 180.0;

    cv::Mat R_yaw = (cv::Mat_<double>(3, 3) << cos(y), -sin(y), 0, sin(y),cos(y), 0, 0, 0, 1);
    cv::Mat R_pitch = (cv::Mat_<double>(3, 3) << cos(p), 0, sin(p), 0, 1, 0, -sin(p), 0, cos(p));

    cv::Mat T_yaw = cv::Mat::eye(4, 4, CV_64F);
    R_yaw.copyTo(T_yaw(cv::Rect(0, 0, 3, 3)));

    cv::Mat T_off = cv::Mat::eye(4, 4, CV_64F);
    T_off.at<double>(0, 3) = AXIS_OFFSET[0];
    T_off.at<double>(1, 3) = AXIS_OFFSET[1];
    T_off.at<double>(2, 3) = AXIS_OFFSET[2];

    cv::Mat T_pitch = cv::Mat::eye(4, 4, CV_64F);
    R_pitch.copyTo(T_pitch(cv::Rect(0, 0, 3, 3)));

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

    std::cout << "使用相机配置文件: " << config_path << std::endl;
    std::cout << "读取标定数据文件夹: " << data_folder << std::endl;

    tdt_radar::CalibrateFly calibrator(config_path);
    calibrator.run(data_folder);

    return 0;
}