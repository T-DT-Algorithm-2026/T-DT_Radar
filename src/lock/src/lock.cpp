#include "lock.h"
#include <array>

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

    read_config();

    cv::FileStorage fs2;
    fs2.open("./config/fly_target.yaml", cv::FileStorage::READ);
    constexpr std::array<float, 4> calibration_distances = {12.0f, 16.0f, 20.0f, 24.0f};
    std::array<float, 4> target_x_points{};
    std::array<float, 4> target_y_points{};
    bool calibration_valid = fs2.isOpened();
    for (std::size_t index = 0; index < calibration_distances.size(); ++index)
    {
        std::string suffix = std::to_string(index + 1);
        cv::FileNode distance_node = fs2["dist" + suffix];
        cv::FileNode target_x_node = fs2["target_x" + suffix];
        cv::FileNode target_y_node = fs2["target_y" + suffix];
        if (distance_node.empty() || target_x_node.empty() || target_y_node.empty())
        {
            calibration_valid = false;
            continue;
        }
        float distance = static_cast<float>(distance_node);
        target_x_points[index] = static_cast<float>(target_x_node);
        target_y_points[index] = static_cast<float>(target_y_node);
        if (std::abs(distance - calibration_distances[index]) > 1e-3f)
        {
            calibration_valid = false;
        }
    }

    if (calibration_valid)
    {
        cv::Mat fit_matrix(4, 2, CV_32F);
        cv::Mat target_x_matrix(4, 1, CV_32F);
        cv::Mat target_y_matrix(4, 1, CV_32F);
        for (std::size_t index = 0; index < calibration_distances.size(); ++index)
        {
            fit_matrix.at<float>(index, 0) = 1.0f / calibration_distances[index];
            fit_matrix.at<float>(index, 1) = 1.0f;
            target_x_matrix.at<float>(index, 0) = target_x_points[index];
            target_y_matrix.at<float>(index, 0) = target_y_points[index];
        }
        cv::Mat x_coefficients;
        cv::Mat y_coefficients;
        calibration_valid = cv::solve(fit_matrix, target_x_matrix, x_coefficients, cv::DECOMP_QR)
                            && cv::solve(fit_matrix, target_y_matrix, y_coefficients, cv::DECOMP_QR);
        if (calibration_valid)
        {
            A_x = x_coefficients.at<float>(0, 0);
            B_x = x_coefficients.at<float>(1, 0);
            A_y = y_coefficients.at<float>(0, 0);
            B_y = y_coefficients.at<float>(1, 0);
            std::cout << "Laser target loaded at 12/16/20/24 m! Ax=" << A_x << ", Bx=" << B_x
                      << ", Ay=" << A_y << ", By=" << B_y << std::endl;
        }
    }
    if (!calibration_valid)
    {
        RCLCPP_ERROR(this->get_logger(), "激光落点需要 12、16、20、24 m 四组完整标定数据，当前使用固定默认准心");
    }

    fs2.release();


    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);//广播
    publish_static_tf();//发布标定的tf

    gimbal_sub = this->create_subscription<gimbal_interface::msg::GimbalAngle>(
        "gimbalUsartData", 10, std::bind(&Lock::gimbal_callback, this, std::placeholders::_1));

    gimbal_pub = this->create_publisher<gimbal_interface::msg::GimbalAngle>(
        "GimbalPub", rclcpp::SensorDataQoS());

    fly_sub = this->create_subscription<vision_interface::msg::DetectFly>(
        "detect_fly", rclcpp::SensorDataQoS().keep_last(1), std::bind(&Lock::callback, this, std::placeholders::_1));

    lidar_sub = this->create_subscription<geometry_msgs::msg::Point32>(
        "/livox/lidar_fly_point", 10, std::bind(&Lock::lidar_callback, this, std::placeholders::_1));

    match_info_sub = this->create_subscription<vision_interface::msg::MatchInfo>(
        "match_info", 10, std::bind(&Lock::match_info_callback, this, std::placeholders::_1));

    last_msg_time_ = this->now();
    last_lidar_time_ = this->now();
    last_match_info_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Lock::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Lock System Initialized with TF Support");
}


void Lock::gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg)
{
    // std::cout<<"Gimbal Callback!"<<std::endl;
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
    last_msg_time_ = this->now();
    rclcpp::Time time_stamp = msg->header.stamp;
    double t = time_stamp.seconds();
    Gimbal gimbal = find_closest_time(t);
    float yaw = gimbal.yaw;
    float pitch = gimbal.pitch;
    float x = msg->x;
    float y = msg->y;
    // std::cout<<msg->x<<","<<msg->y<<std::endl;

    // 将准心像素和目标像素分别转换为相机视线角，两者之差就是本帧角度误差。
    float yaw1 = atan2(target_x - cx, fx);
    float pitch1 = atan2(target_y - cy, fy);

    float yaw2 = atan2(x - cx, fx);
    float pitch2 = atan2(y - cy, fy);

    double current_yaw = -(yaw2 - yaw1);
    double current_pitch = -(pitch2 - pitch1);
    // 图像时间戳处的云台反馈 + 相机角度误差 = 世界中的绝对目标角度测量。
    cv::Point2d measured_angle(yaw + current_yaw, pitch + current_pitch);

    rclcpp::Time now = this->now();
    double measurement_age_s = std::max(0.0, (now - time_stamp).seconds());
    // 从图像采集时刻预测到未来控制真正生效的时刻。
    double predict_time = std::clamp( measurement_age_s + control_delay_s, 0.0, max_prediction_horizon_s);

    cv::Point2d predicted_angle = measured_angle;
    cv::Point2d angle_speed(0.0, 0.0);
    if (kf_ptr == nullptr)
    {
        kf_ptr = std::make_shared<Kalman_filter_plus>(measured_angle, time_stamp, kf_config);
    }
    else
    {
        kf_ptr->update(measured_angle, time_stamp);
    }
    predicted_angle = kf_ptr->predict(predict_time);
    angle_speed = kf_ptr->angular_velocity();
    // std::cout<<"预测角度"<<predicted_angle.x<<","<<predicted_angle.y<<std::endl;
    // std::cout<<"速度"<<angle_speed.x<<","<<angle_speed.y<<std::endl;

    // 使用最新云台反馈计算控制误差；不能再用图像时刻的旧反馈，否则会重复补偿视觉延迟。
    Gimbal latest_gimbal = find_closest_time(now.seconds());
    double yaw_error = predicted_angle.x - latest_gimbal.yaw;
    double pitch_error = predicted_angle.y - latest_gimbal.pitch;
    if (std::abs(yaw_error) < angle_deadband_rad)
    {
        yaw_error = 0.0;
    }
    if (std::abs(pitch_error) < angle_deadband_rad)
    {
        pitch_error = 0.0;
    }

    double final_yaw = yaw_error * kp_x;
    double final_pitch = pitch_error * kp_y;
    yaw = (latest_gimbal.yaw + final_yaw) * 180.0 / CV_PI;
    pitch = (latest_gimbal.pitch + final_pitch) * 180.0 / CV_PI;

    // if(radar_angle_valid)
    // {
    //     yaw = std::clamp(yaw, radar_yaw - radar_yaw_limit, radar_yaw + radar_yaw_limit);
    //     pitch = std::clamp(pitch, radar_pitch - radar_pitch_limit, radar_pitch + radar_pitch_limit);
    // }

    // std::cout<<"Lock Command - Yaw: "<<yaw<<", Pitch: "<<pitch<<std::endl;
    // std::cout<<"dyaw:"<<final_yaw*180.0/CV_PI<<",dpitch:"<<final_pitch*180.0/CV_PI
    //          <<", predict_ms:"<<predict_time*1000.0
    //          <<", yaw_rate:"<<angle_speed.x*180.0/CV_PI
    //          <<", pitch_rate:"<<angle_speed.y*180.0/CV_PI<<std::endl;

    gimbal_interface::msg::GimbalAngle gimbal_msg;
    gimbal_msg.header.stamp = this->now();
    gimbal_msg.header.frame_id = "yaw_link"; 
    gimbal_msg.yaw = yaw;
    gimbal_msg.pitch = pitch;
    gimbal_msg.is_fire = is_fire;
    gimbal_msg.force_flag = 1;
    // std::cout<<"Test Callback - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    // std::cout<<"Lock Command - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    gimbal_pub->publish(gimbal_msg);// 发布预测后的绝对云台角度指令

    // end 和检测消息时间戳都使用 ROS 时钟，计算从图像时间戳到锁定回调结束的总延迟。
    rclcpp::Time end = this->now();
    double end_to_timestamp_ms = static_cast<double>((end - time_stamp).nanoseconds()) / 1.0e6;
    std::cout << "End - time_stamp: " << end_to_timestamp_ms << " ms\n";
    
    // // cv::imshow("lock_test", img);
    // cv::waitKey(1);

}

void Lock::timer_callback()
{
    if ((this->now() - last_match_info_time_).seconds() > match_info_timeout_s)
    {
        is_fire = true;
    }//match_info失联时特殊处理

    if ((this->now() - last_lidar_time_).seconds() > 1.0)
    {
        lidar_valid = false;
        radar_yaw = -10.0f;
        radar_pitch = -4.5f;
        radar_angle_valid = true;
    }
    else if (lidar_valid)
    {
        geometry_msgs::msg::PointStamped point_base;
        point_base.header.stamp = this->now();
        point_base.header.frame_id = "pitch_link";
        point_base.point.x = fly_pos.x;
        point_base.point.y = fly_pos.y;
        point_base.point.z = fly_pos.z;

        geometry_msgs::msg::PointStamped point_cam;
        tf2::doTransform(point_base, point_cam, static_transform_);
        float cx = point_cam.point.x;
        float cy = point_cam.point.y;
        float cz = point_cam.point.z;
        float dist_horizontal = std::sqrt(cx * cx + cy * cy);

        if(dist_horizontal > 1e-3)
        {
            float yaw_err = std::atan2(cy, cx); 
            float pitch_err = std::atan2(cz, dist_horizontal);

            radar_yaw = yaw_err * (180.0f / CV_PI) + first_lock_yaw_offset_deg;
            radar_pitch = pitch_err * (180.0f / CV_PI) + first_lock_pitch_offset_deg;
            radar_angle_valid = true;
        }
    }

    // 如果超过 1 秒没有收到消息，则认为目标丢失，重置卡尔曼并调用 find_callback
    if ((this->now() - last_msg_time_).seconds() > 1)
    {
        kf_ptr.reset();
        find_callback();
    }
}

void Lock::find_callback()
{
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "未接收到 detect_fly 消息，执行 find_callback 进行第一次锁定");
    if(!radar_angle_valid)
    {
        return;
    }

    // 巡航逻辑
    patrol();

    // 将误差补偿到当前云台角度上，得到最终的绝对命令角度
    float target_yaw = radar_yaw + yaw_cmd_first; // 雷达偏移补偿，经验值
    float target_pitch = radar_pitch + pitch_cmd_first; 
    std::cout<<"target_yaw: "<<target_yaw<<" , "<<"target_pitch: "<<target_pitch<<std::endl;

    gimbal_interface::msg::GimbalAngle gimbal_msg;
    gimbal_msg.header.stamp = this->now();
    gimbal_msg.header.frame_id = "yaw_link"; 
    gimbal_msg.yaw = target_yaw;   
    gimbal_msg.pitch = target_pitch; 
    gimbal_msg.is_fire = is_fire;
    gimbal_msg.force_flag = 1;

    gimbal_pub->publish(gimbal_msg);
}


void Lock::lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg)
{
    last_lidar_time_ = this->now();
    fly_pos.x = msg->x;
    fly_pos.y = msg->y;
    fly_pos.z = msg->z;
    lidar_valid = true;
    float distance = std::sqrt(fly_pos.x * fly_pos.x + fly_pos.y * fly_pos.y + fly_pos.z * fly_pos.z);
    
    // 如果启用了动态靶心，根据当前距离实时更新 target_x 和 target_y
    if(distance > 1e-3)
    {
        target_x = A_x / distance + B_x;
        target_y = A_y / distance + B_y;
    }
}

void Lock::match_info_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    last_match_info_time_ = this->now();
    if (msg->match_time <= 20)
    {
        countermeasure_count = 0;
        enemy_drone_countered = false;
        countermeasure_waiting = false;
        is_fire = true;
        return;
    }//比赛结束的初始化

    bool is_countered = msg->mark_progress[1];
    if (is_countered && !enemy_drone_countered)
    {
        countermeasure_count++;
    }//上升沿记录次数

    if (is_countered)
    {
        // 对方处于反制状态时仍持续开火，只取消反制结束后的等待状态。
        is_fire = true;
        countermeasure_waiting = false;
    }
    else if (enemy_drone_countered)
    {
        countermeasure_end_time = this->now();
        countermeasure_waiting = true;
        is_fire = false;
    }//下降沿开始等待

    if (countermeasure_waiting && (this->now() - countermeasure_end_time).seconds() >= countermeasure_interval_s)
    {
        is_fire = true;
        countermeasure_waiting = false;
    }//等待结束

    // 剩余时间只够完成剩余反制时，跳过额外等待
    if (!is_countered && countermeasure_count < 5 && msg->match_time <= (6 - countermeasure_count) * 5 + (5 - countermeasure_count) * 55)
    {
        countermeasure_waiting = false;
        is_fire = true;
    }

    enemy_drone_countered = is_countered;
}

void Lock::publish_static_tf() 
{
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "pitch_link";
    t.child_frame_id = "camera_link";
    t.header.stamp = this->now();

    // 转换 OpenCV tvec 到 ROS2 translation (米)
    // t.transform.translation.x = cam2pitch_tvec_.at<double>(0);
    // t.transform.translation.y = cam2pitch_tvec_.at<double>(1);
    // t.transform.translation.z = cam2pitch_tvec_.at<double>(2);
    t.transform.translation.x = 0.064974;
    t.transform.translation.y = 0.0524;
    t.transform.translation.z = 0.04785;

    // 转换 OpenCV rvec 到 ROS2 Quaternion
    cv::Mat R;
    cv::Rodrigues(cam2pitch_rvec_, R);
    tf2::Matrix3x3 tf2_R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    tf2::Quaternion q;
    tf2_R.getRotation(q);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    static_tf_broadcaster_->sendTransform(t);

    // 提前计算逆变换并保存到 static_transform_，
    // 供 find_callback 中将 target 坐标从 pitch_link 转回 camera_link 使用
    tf2::Transform tf2_trans;
    tf2_trans.setOrigin(tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
    tf2_trans.setRotation(q);
    
    tf2::Transform tf2_inv = tf2_trans.inverse();
    
    static_transform_.header.frame_id = "camera_link";  // 源坐标系
    static_transform_.child_frame_id = "pitch_link";    // 目标坐标系
    static_transform_.transform.translation.x = tf2_inv.getOrigin().x();
    static_transform_.transform.translation.y = tf2_inv.getOrigin().y();
    static_transform_.transform.translation.z = tf2_inv.getOrigin().z();
    static_transform_.transform.rotation.x = tf2_inv.getRotation().x();
    static_transform_.transform.rotation.y = tf2_inv.getRotation().y();
    static_transform_.transform.rotation.z = tf2_inv.getRotation().z();
    static_transform_.transform.rotation.w = tf2_inv.getRotation().w();
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


void Lock::patrol()
{
    // 巡航逻辑: 绕目标点做正方形巡逻
    float step = 0.05f;
    float max_val = 2.2f;
    float min_val = -2.2f;

    switch (patrol_state_) {
        case 0: // 向右扫 (yaw 增加)
            yaw_cmd_first += step;
            if (yaw_cmd_first >= max_val) {
                yaw_cmd_first = max_val;
                patrol_state_ = 1;
            }
            break;
        case 1: // 向上扫 (pitch 增加)
            pitch_cmd_first += step;
            if (pitch_cmd_first >= max_val) {
                pitch_cmd_first = max_val;
                patrol_state_ = 2;
            }
            break;
        case 2: // 向左扫 (yaw 减小)
            yaw_cmd_first -= step;
            if (yaw_cmd_first <= min_val) {
                yaw_cmd_first = min_val;
                patrol_state_ = 3;
            }
            break;
        case 3: // 向下扫 (pitch 减小)
            pitch_cmd_first -= step;
            if (pitch_cmd_first <= min_val) {
                pitch_cmd_first = min_val;
                patrol_state_ = 0;
            }
            break;
    }
}
// namespace tdt_lock
}
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_lock::Lock)
