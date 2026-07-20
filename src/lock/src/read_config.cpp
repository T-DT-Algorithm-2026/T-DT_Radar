#include "lock.h"

namespace tdt_lock {

void Lock::read_config()
{
    cv::FileStorage fs;
    fs.open("./config/lock_config.yaml", cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        // 配置文件不存在时，明确恢复全部默认值。节点继续运行，同时输出警告方便排查路径。
        kf_measurement_noise_px = 4.0;
        kf_measurement_noise_y_px = 16.0;
        kf_q_rad2_s3 = 0.03;
        kf_initial_velocity_std_deg_s = 10.0;
        kf_innovation_gate_base_deg = 0.15;
        kf_innovation_gate_rate_deg_s = 10.0;
        control_delay_s = 0.015;
        RCLCPP_WARN(this->get_logger(), "无法打开 ./config/lock_config.yaml，卡尔曼使用全部默认值");
    }
    else
    {
        // R 的 x 方向测量标准差。配置中不存在时使用默认值 4 px。
        if (!fs["kf_measurement_noise_px"].empty())
        {
            fs["kf_measurement_noise_px"] >> kf_measurement_noise_px;
        }
        else
        {
            kf_measurement_noise_px = 4.0;
            RCLCPP_WARN(this->get_logger(),"lock_config 缺少 kf_measurement_noise_px，使用默认值 4.0 px");
        }

        // R 的 y 方向测量标准差。配置中不存在时使用默认值 16 px。
        if (!fs["kf_measurement_noise_y_px"].empty())
        {
            fs["kf_measurement_noise_y_px"] >> kf_measurement_noise_y_px;
        }
        else
        {
            kf_measurement_noise_y_px = 16.0;
            RCLCPP_WARN(this->get_logger(), "lock_config 缺少 kf_measurement_noise_y_px，使用默认值 16.0 px");
        }

        // Q 使用的标量噪声谱密度 q，单位为 rad^2/s^3。
        if (!fs["kf_q_rad2_s3"].empty())
        {
            fs["kf_q_rad2_s3"] >> kf_q_rad2_s3;
        }
        else
        {
            kf_q_rad2_s3 = 0.03;
            RCLCPP_WARN(this->get_logger(), "lock_config 缺少 kf_q_rad2_s3，使用默认值 0.03 rad^2/s^3");
        }

        // 卡尔曼初始化时对目标角速度不确定程度的估计，配置单位为 deg/s。
        if (!fs["kf_initial_velocity_std_deg_s"].empty())
        {
            fs["kf_initial_velocity_std_deg_s"] >> kf_initial_velocity_std_deg_s;
        }
        else
        {
            kf_initial_velocity_std_deg_s = 10.0;
            RCLCPP_WARN(this->get_logger(), "lock_config 缺少 kf_initial_velocity_std_deg_s，" "使用默认值 10.0 deg/s");
        }

        // 正常测量允许偏离预测值的基础角度，配置单位为 deg。
        if (!fs["kf_innovation_gate_base_deg"].empty())
        {
            fs["kf_innovation_gate_base_deg"] >> kf_innovation_gate_base_deg;
        }
        else
        {
            kf_innovation_gate_base_deg = 0.15;
            RCLCPP_WARN(this->get_logger(), "lock_config 缺少 kf_innovation_gate_base_deg，使用默认值 0.15 deg");
        }

        // 帧间隔变大时，创新门限按该速率增大，配置单位为 deg/s。
        if (!fs["kf_innovation_gate_rate_deg_s"].empty())
        {
            fs["kf_innovation_gate_rate_deg_s"] >> kf_innovation_gate_rate_deg_s;
        }
        else
        {
            kf_innovation_gate_rate_deg_s = 10.0;
            RCLCPP_WARN(this->get_logger(), "lock_config 缺少 kf_innovation_gate_rate_deg_s，" "使用默认值 10.0 deg/s");
        }

        //延迟补偿s
        if (!fs["control_delay_s"].empty())
        {
            fs["control_delay_s"] >> control_delay_s;
        }
        else
        {
            control_delay_s = 0.015;
            RCLCPP_WARN(this->get_logger(), "lock_config 缺少 control_delay_s，" "使用默认值 0.015 s");
        }
    }

    fs.release();

    // R：像素标准差除以焦距，得到 yaw/pitch 的角度标准差（rad）。
    // kalman_cv.h 会再将这两个标准差平方，填入测量噪声矩阵 R 的对角线。
    kf_config.measurement_std_yaw_rad = kf_measurement_noise_px / fx;
    kf_config.measurement_std_pitch_rad = kf_measurement_noise_y_px / fy;

    // Q：q 已使用卡尔曼内部需要的 rad^2/s^3，直接写入配置。
    kf_config.angular_acceleration_noise = kf_q_rad2_s3;

    // 角度和角速度参数从 deg、deg/s 转换为 rad、rad/s。
    double deg_to_rad = CV_PI / 180.0;
    kf_config.initial_velocity_std_rad_s = kf_initial_velocity_std_deg_s * deg_to_rad;
    kf_config.innovation_gate_base_rad = kf_innovation_gate_base_deg * deg_to_rad;
    kf_config.innovation_gate_rate_rad_s = kf_innovation_gate_rate_deg_s * deg_to_rad;
}

}  // namespace tdt_lock
