#include "lock.h"

namespace tdt_lock {

Lock::Lock(const rclcpp::NodeOptions& options)
    : Node("lock_node", options)
{
    std::cout<<"Lock系统初始化中"<<std::endl; 
    
    cv::FileStorage fs1;
    fs1.open("./config/f.yaml", cv::FileStorage::READ);
    fs1["f_x"] >> fx;
    fs1["f_y"] >> fy;
    fs1.release();

    cv::FileStorage fs2;
    fs2.open("./config/fly_target.yaml", cv::FileStorage::READ);
    fs2["target_x"] >> target_x;
    fs2["target_y"] >> target_y;
    fs2.release();


    gimbal_sub = this->create_subscription<gimbal_interface::msg::GimbalAngle>(
        "gimbalUsartData", 10, std::bind(&Lock::gimbal_callback, this, std::placeholders::_1));

    gimbal_pub = this->create_publisher<gimbal_interface::msg::GimbalAngle>(
        "GimbalPub", rclcpp::SensorDataQoS());

    fly_sub = this->create_subscription<vision_interface::msg::DetectFly>(
        "detect_fly", 10, std::bind(&Lock::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Lock System Initialized with TF Support");
}


void Lock::gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg)
{
    // std::cout<<"Gimbal Callback!"<<std::endl;
    rclcpp::Time time_stamp = msg->header.stamp;
    double t = rclcpp::Time(msg->header.stamp).seconds();
    Gimbal gimbal;
    gimbal.yaw = msg->yaw*CV_PI/180.0;
    gimbal.pitch = msg->pitch*CV_PI/180.0;
    gimbal.time = t;
    std::lock_guard<std::mutex> lock(gimbal_mutex);
    gimbal_history.push_back(gimbal);
    if (gimbal_history.size() > 100) 
    {
        gimbal_history.pop_front();
    }
}

void Lock::callback(const vision_interface::msg::DetectFly::SharedPtr msg)
{
    std::chrono::steady_clock::time_point begin =std::chrono::steady_clock::now();
    rclcpp::Time time_stamp = msg->header.stamp;
    double t = rclcpp::Time(msg->header.stamp).seconds();
    Gimbal gimbal = find_closest_time(t);
    float yaw = gimbal.yaw;
    float pitch = gimbal.pitch;
    std::cout<<"dt"<<(t-gimbal.time)<<std::endl;
    float x = msg->x;
    float y = msg->y;
    // std::cout<<"x:"<<x<<"y:"<<y<<std::endl;
    if(x>=380&&x<=1060&&y>=270&&y<=790)
    {
        is_find ++;
        if(is_first)
        {
            base_yaw = yaw;
            base_pitch = pitch;
            is_first = false;
        }
    }
    else
    {
        is_find = 0;
        is_first = true;
    }//判断是否开启kf

    cv::Mat img = cv::Mat::zeros(cv::Size(1440, 1080), CV_8UC3);
    cv::circle(img, cv::Point2f(x, y), 4, cv::Scalar(255, 255, 255), -1);
    cv::rectangle(img, cv::Point(380, 270), cv::Point(1060, 790), cv::Scalar(0, 255, 0), 3);

    if(is_find >= 20)
    {
        
        float dx = tan(yaw - base_yaw) * fx;
        float dy = tan(pitch - base_pitch) * fy;
        cv::circle(img, cv::Point2f(x-dx, y-dy), 4, cv::Scalar(0, 255, 255), -1);
        pcl::PointXY abject_point(x-dx, y-dy);
        // rclcpp::Time now_time = this->now();
        cv::Point2f predict_point;//卡尔曼准备 

        if(kf_ptr == nullptr)
        {
            kf_ptr = std::make_shared<Kalman_filter_plus>(abject_point, time_stamp);
        }
        else if(kf_ptr != nullptr)
        {
            kf_ptr->update_predict_point();
            kf_ptr->update(abject_point, time_stamp);
            predict_point = kf_ptr->get_predict_point();
        }
        // x = predict_point.x + dx;
        // y = predict_point.y + dy;
        // x = std::clamp(x, 380.0f, 1060.0f);
        // y = std::clamp(y, 270.0f, 790.0f);
    }
    else
    {
        kf_ptr.reset();
    }
    cv::circle(img, cv::Point2f(x, y), 4, cv::Scalar(255, 255, 0), -1);

    float yaw1 = atan2(target_x - cx, fx);
    float pitch1 = atan2(target_y - cy, fy);

    float yaw2 = atan2(x - cx, fx);
    float pitch2 = atan2(y - cy, fy);

    float current_yaw = -(yaw2 - yaw1);
    float current_pitch = -(pitch2 - pitch1);//计算旋转角度

    float current_dyaw = current_yaw - last_dyaw;
    float current_dpitch = current_pitch - last_dpitch;

    float final_yaw = current_yaw * kp_x + kd * current_dyaw;
    float final_pitch = current_pitch * kp_y + kd * current_dpitch;

    last_dyaw = current_yaw;
    last_dpitch = current_pitch;

    yaw = (yaw + final_yaw)*180.0/CV_PI;
    pitch = (pitch + final_pitch)*180.0/CV_PI;

    std::cout<<"Lock Command - Yaw: "<<yaw<<", Pitch: "<<pitch<<std::endl;
    std::cout<<"dyaw:"<<final_yaw*180.0/CV_PI<<",dpitch:"<<final_pitch*180.0/CV_PI<<std::endl;

    gimbal_interface::msg::GimbalAngle gimbal_msg;
    gimbal_msg.header.stamp = this->now();
    gimbal_msg.header.frame_id = "yaw_link"; 
    gimbal_msg.yaw = yaw;   
    gimbal_msg.pitch = pitch;
    gimbal_msg.is_fire = 0;
    gimbal_msg.force_flag = 1;
    // std::cout<<"Test Callback - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    // std::cout<<"Lock Command - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    gimbal_pub->publish(gimbal_msg);//发布数据（yaw为增量，pitch为绝对角度，相对于重力)
    std::chrono::steady_clock::time_point end =std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
    // std::cout << "Lock Time: " << time_used.count() * 1000 << "ms" << std::endl;


    //cv::imshow("lock_test", img);
    cv::waitKey(1);

}

Gimbal Lock::find_closest_time(double target_time)
{
    std::lock_guard<std::mutex> lock(gimbal_mutex);
    if (gimbal_history.empty()) 
    {
        return {0.0f, 0.0f, 0.0};
    }
    auto it = std::lower_bound(gimbal_history.begin(), gimbal_history.end(), target_time,
        [](const Gimbal& g, double val) 
        {
            return g.time < val;
        }
    );

    if (it == gimbal_history.end()) 
    {
        return gimbal_history.back();
    }
    if (it == gimbal_history.begin()) 
    {
        return gimbal_history.front();
    }

    auto it_after = it;
    auto it_before = std::prev(it);

    double t0 = it_before->time;
    double t1 = it_after->time;

    // 防止时间戳完全相同导致除零异常
    if (t1 - t0 < 1e-6) 
    {
        return *it_after;
    }

    // 计算插值比重 [0, 1] 之间
    double ratio = (target_time - t0) / (t1 - t0);

    // 核心精进：由于双轴都有限位，不存在 360° 过零点突变问题，直接使用简单的普通线性插值
    Gimbal interp_gimbal;
    interp_gimbal.time = target_time;
    interp_gimbal.pitch = it_before->pitch + ratio * (it_after->pitch - it_before->pitch);
    interp_gimbal.yaw = it_before->yaw + ratio * (it_after->yaw - it_before->yaw);

    return interp_gimbal;
}
// namespace tdt_lock
}
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_lock::Lock)