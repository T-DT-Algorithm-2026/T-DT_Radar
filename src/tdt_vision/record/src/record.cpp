#include "record.h"

namespace tdt_radar {

Record::Record(const rclcpp::NodeOptions& options)
    : Node("record_node", options)
{
    cv::namedWindow("Calibration Preview", cv::WINDOW_NORMAL);

    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image", rclcpp::SensorDataQoS(),
            std::bind(&Record::callback, this, std::placeholders::_1));
    gimbal_sub =
        this->create_subscription<gimbal_interface::msg::GimbalAngle>(
            "gimbalUsartData", 10,
            std::bind(&Record::gimbal_callback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "云台标定节点已启动。操作说明：\n按 "
                                    "'n' 采样当前帧\n按 'c' 执行标定计算");
}

void Record::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 获取图像
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    
    // 直接显示原图（此时不画角点，保证流畅）
    cv::imshow("Calibration Preview", img);

    // 2. 关键修改：将 waitKey 改为 1ms，防止卡顿
    int key = cv::waitKey(1);

    // 3. 只有按下 'n' 键时，才执行耗时的寻找角点操作
    if (key == 'n') 
    {
        RCLCPP_INFO(this->get_logger(), "正在尝试寻找角点...");
        
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        //先保存云台位置，防止云台移动
        last_yaw=yaw;
        last_pitch=pitch;
        std::cout<<"当前时间的Yaw："<<yaw<<", Pitch:"<<pitch<<std::endl;
        // 执行耗时计算
        bool found = cv::findChessboardCorners(gray, BOARD_SIZE, corners);

        if (found) 
        { 
            // 保存数据
            number++;
            RCLCPP_INFO(this->get_logger(), "采样成功！当前样本数: %d", number);
            size_t current_index = number; // 获取当前是第几个样本
            // 将基础文件名改为递增的数字
            std::string base_filename = "/home/robot/calib_img/" + std::to_string(current_index);
            
            // 画角点并保存图片 (例如: 1.jpg)
            cv::Mat debug_img = img.clone();
            cv::drawChessboardCorners(debug_img, BOARD_SIZE, corners, found);
            std::string img_filename = base_filename + ".png";
            cv::imwrite(img_filename, img);
            
            // 储存 yaw 和 pitch 到同名 txt 文件 (例如: 1.txt)
            std::string txt_filename = base_filename + ".txt";
            std::ofstream out_file(txt_filename);
            if (out_file.is_open()) 
            {
                out_file << "yaw: " << last_yaw << "\n";
                out_file << "pitch: " << last_pitch << "\n";
                out_file.close();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "无法保存位姿文件: %s", txt_filename.c_str());
            }
            cv::imshow("Calibration Preview", debug_img);
            cv::waitKey(500); // 暂停0.5秒让用户看到“捕捉成功”的画面
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "未找到标定板，采样失败！请调整角度后重试。");
        }
    }
}

void Record::gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg)
{
    yaw = msg->yaw;
    pitch = msg->pitch;
}

}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Record)