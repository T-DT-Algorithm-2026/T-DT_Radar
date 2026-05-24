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

    fs2["dist1"] >> dist1;
    fs2["target_x1"] >> target_x1;
    fs2["target_y1"] >> target_y1;
    fs2["dist2"] >> dist2;
    fs2["target_x2"] >> target_x2;
    fs2["target_y2"] >> target_y2;
    
    // 使用反比例函数模型：u = A/D + B
    float inv_d1 = 1.0f / dist1;
    float inv_d2 = 1.0f / dist2;
    if (std::abs(inv_d1 - inv_d2) > 1e-5) 
    {
        A_x = (target_x1 - target_x2) / (inv_d1 - inv_d2);
        B_x = target_x1 - A_x * inv_d1;
        A_y = (target_y1 - target_y2) / (inv_d1 - inv_d2);
        B_y = target_y1 - A_y * inv_d1;
        std::cout << "Dynamic Target Loaded! Ax=" << A_x << ", Bx=" << B_x << ", Ay=" << A_y << ", By=" << B_y << std::endl;
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
        "detect_fly", 10, std::bind(&Lock::callback, this, std::placeholders::_1));

    lidar_sub = this->create_subscription<geometry_msgs::msg::Point32>(
        "/livox/lidar_fly_point", 10, std::bind(&Lock::lidar_callback, this, std::placeholders::_1));

    last_msg_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Lock::timer_callback, this));

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
    last_msg_time_ = this->now();
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

    if(radar_angle_valid)
    {
        yaw = std::clamp(yaw, radar_yaw - radar_yaw_limit, radar_yaw + radar_yaw_limit);
        pitch = std::clamp(pitch, radar_pitch - radar_pitch_limit, radar_pitch + radar_pitch_limit);
    }

    std::cout<<"Lock Command - Yaw: "<<yaw<<", Pitch: "<<pitch<<std::endl;
    std::cout<<"dyaw:"<<final_yaw*180.0/CV_PI<<",dpitch:"<<final_pitch*180.0/CV_PI<<std::endl;

    gimbal_interface::msg::GimbalAngle gimbal_msg;
    gimbal_msg.header.stamp = this->now();
    gimbal_msg.header.frame_id = "yaw_link"; 
    gimbal_msg.yaw = yaw;   
    gimbal_msg.pitch = pitch;
    gimbal_msg.is_fire = 1;
    gimbal_msg.force_flag = 1;
    // std::cout<<"Test Callback - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    // std::cout<<"Lock Command - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    gimbal_pub->publish(gimbal_msg);//发布数据（yaw为增量，pitch为绝对角度，相对于重力)
    std::chrono::steady_clock::time_point end =std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
    // std::cout << "Lock Time: " << time_used.count() * 1000 << "ms" << std::endl;
    
    // cv::imshow("lock_test", img);
    cv::waitKey(1);

}

void Lock::timer_callback()
{
    if(lidar_valid)
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

            radar_yaw = yaw_err * (180.0f / CV_PI) - 2;
            radar_pitch = pitch_err * (180.0f / CV_PI);
            radar_angle_valid = true;
        }
    }

    // 如果超过 0.5 秒没有收到消息，则认为目标丢失，调用 find_callback
    if ((this->now() - last_msg_time_).seconds() > 1)
    {
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

    gimbal_interface::msg::GimbalAngle gimbal_msg;
    gimbal_msg.header.stamp = this->now();
    gimbal_msg.header.frame_id = "yaw_link"; 
    gimbal_msg.yaw = target_yaw;   
    gimbal_msg.pitch = target_pitch;
    gimbal_msg.is_fire = 1;
    gimbal_msg.force_flag = 1;

    gimbal_pub->publish(gimbal_msg);
}


void Lock::lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg)
{
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
    float max_val = 1.0f;
    float min_val = -1.0f;

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
