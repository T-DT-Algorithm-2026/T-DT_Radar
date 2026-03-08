#include "lock.h"

namespace tdt_lock {

Lock::Lock(const rclcpp::NodeOptions& options)
    : Node("lock_node", options)
{
    std::cout<<"Lock系统初始化中"<<std::endl;
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);//广播
    std::cout<<"初始化"<<std::endl;
    
    load_camera_params();//读参
    std::cout<<"读参完成"<<std::endl;
    publish_static_tf();//发布标定的tf
    std::cout<<"发布静态tf完成"<<std::endl;

    gimbal_sub = this->create_subscription<gimbal_interface::msg::GimbalAngle>(
        "gimbalUsartData", 10, std::bind(&Lock::gimbal_callback, this, std::placeholders::_1));
    
    resolve_sub = this->create_subscription<vision_interface::msg::ResolveResult>(
        "fly_resolve_result", rclcpp::SensorDataQoS(), std::bind(&Lock::callback, this, std::placeholders::_1));

    gimbal_pub = this->create_publisher<gimbal_interface::msg::GimbalAngle>(
        "GimbalPub", rclcpp::SensorDataQoS());

    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(2), 
    //     std::bind(&Lock::timer_callback, this)
    // );

    RCLCPP_INFO(this->get_logger(), "Lock System Initialized with TF Support");
}

void Lock::load_camera_params() 
{
    cv::FileStorage fs("./config/out_matrix_fly.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML config!");
        return;
    }
    fs["cam2pitch_tvec"] >> cam2pitch_tvec_;
    fs["cam2pitch_rvec"] >> cam2pitch_rvec_;
    fs.release();
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
}

// void Lock::timer_callback()
// {
//     rclcpp::Time now_time = this->now();
//     // Base -> Yaw_link
//     geometry_msgs::msg::TransformStamped t_yaw;
//     t_yaw.header.stamp = now_time;
//     t_yaw.header.frame_id = "base_link";
//     t_yaw.child_frame_id = "yaw_link";
//     tf2::Quaternion q_yaw;
//     q_yaw.setRPY(0, 0, yaw); // 弧度单位
//     t_yaw.transform.rotation.x = q_yaw.x();
//     t_yaw.transform.rotation.y = q_yaw.y();
//     t_yaw.transform.rotation.z = q_yaw.z();
//     t_yaw.transform.rotation.w = q_yaw.w();

//     //Yaw_link -> Pitch_link 
//     geometry_msgs::msg::TransformStamped t_pitch;
//     t_pitch.header.stamp = now_time;  // 改成 t_pitch
//     t_pitch.header.frame_id = "yaw_link";
//     t_pitch.child_frame_id = "pitch_link";
//     t_pitch.transform.translation.x = pitch2yaw_tvec_.at<double>(0);
//     t_pitch.transform.translation.y = pitch2yaw_tvec_.at<double>(1);
//     t_pitch.transform.translation.z = pitch2yaw_tvec_.at<double>(2);
//     tf2::Quaternion q_pitch;
//     q_pitch.setRPY(pitch, 0, 0); // 假设 Pitch 轴绕  X 旋转
//     t_pitch.transform.rotation.x = q_pitch.x();
//     t_pitch.transform.rotation.y = q_pitch.y();
//     t_pitch.transform.rotation.z = q_pitch.z();
//     t_pitch.transform.rotation.w = q_pitch.w();

//     tf_broadcaster_->sendTransform({t_yaw, t_pitch});//广播
// }  

void Lock::gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg)
{
    // std::cout<<"Gimbal Callback!"<<std::endl;
    rclcpp::Time saved_stamp_ = this->now();
    yaw = msg->yaw * (M_PI / 180.0);
    pitch = msg->pitch * (M_PI / 180.0);
    rclcpp::Time now_time = this->now();
    // Base -> Yaw_link
    geometry_msgs::msg::TransformStamped t_yaw;
    t_yaw.header.stamp = saved_stamp_;
    t_yaw.header.frame_id = "base_link";
    t_yaw.child_frame_id = "yaw_link";
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0, 0, yaw); // 弧度单位
    t_yaw.transform.rotation.x = q_yaw.x();
    t_yaw.transform.rotation.y = q_yaw.y();
    t_yaw.transform.rotation.z = q_yaw.z();
    t_yaw.transform.rotation.w = q_yaw.w();

    //Yaw_link -> Pitch_link 
    geometry_msgs::msg::TransformStamped t_pitch;
    t_pitch.header.stamp = saved_stamp_;  // 改成 t_pitch
    t_pitch.header.frame_id = "yaw_link";
    t_pitch.child_frame_id = "pitch_link";
    t_pitch.transform.translation.x = pitch2yaw_tvec_.at<double>(0)/1000;
    t_pitch.transform.translation.y = pitch2yaw_tvec_.at<double>(1)/1000;
    t_pitch.transform.translation.z = pitch2yaw_tvec_.at<double>(2)/1000;
    tf2::Quaternion q_pitch;
    q_pitch.setRPY(0, -pitch, 0); // 假设 Pitch 轴绕  Y 旋转
    t_pitch.transform.rotation.x = q_pitch.x();
    t_pitch.transform.rotation.y = q_pitch.y();
    t_pitch.transform.rotation.z = q_pitch.z();
    t_pitch.transform.rotation.w = q_pitch.w();

    tf_broadcaster_->sendTransform({t_yaw, t_pitch});//广播
}

// void Lock::timer_callback()
// {
//     auto now_time = std::chrono::steady_clock::now();//当前时间

//     gimbal_interface::msg::GimbalAngle gimbal_msg;
//     gimbal_msg.header.stamp = this->now();
//     gimbal_msg.header.frame_id = "yaw_link"; 
//     gimbal_msg.yaw = 0;   
//     gimbal_msg.pitch = 0;
//     gimbal_msg.is_fire =0;
//     gimbal_pub->publish(gimbal_msg);
//     auto end_time = std::chrono::steady_clock::now();
//     float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
//     RCLCPP_INFO(this->get_logger(), "Timer Callback time is %f ms", dur_time);
// }

void Lock::callback(const vision_interface::msg::ResolveResult::SharedPtr msg) 
{
    geometry_msgs::msg::PointStamped point_cam;
    point_cam.header = msg->header;
    point_cam.header.frame_id = "camera_link";
    point_cam.point.x = msg->pose.position.z;
    point_cam.point.y = -msg->pose.position.x;
    point_cam.point.z = -msg->pose.position.y;//传参
    std::cout<<"pnp坐标- X: "<<point_cam.point.x<<", Y: "<<point_cam.point.y<<", Z: "<<point_cam.point.z<<std::endl;

    geometry_msgs::msg::PointStamped point_target;

    try 
    {
        point_target = tf_buffer_->transform(point_cam, "yaw_link", tf2::durationFromSec(0.05));
    } 
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        return;
    }//tf转换

    pcl::PointXYZ point_fly;
    point_fly.x = point_target.point.x;
    point_fly.y = point_target.point.y;
    point_fly.z = point_target.point.z;
    rclcpp::Time now_time = this->now();//卡尔曼准备 
    std::cout<<"云台坐标- X: "<<point_fly.x<<", Y: "<<point_fly.y<<", Z: "<<point_fly.z<<std::endl;

    if(kf_ptr == nullptr)
    {
        kf_ptr = std::make_shared<Kalman_filter_plus>(point_fly, now_time);
    }
    else if( kf_ptr != nullptr&&kf_ptr->miss_last_time<=0)
    {
        kf_ptr->update_predict_point();
        kf_ptr->deal_catch(point_fly, now_time);
    }
    else if(kf_ptr->miss_last_time<=1.5&&kf_ptr->miss_last_time>0)
    {
        kf_ptr->update_predict_point();

        kf_ptr->deal_missing(now_time);
    }
    else if(kf_ptr->miss_last_time>1.5)
    {
        kf_ptr.reset();
    }
    cv::Point3f predict_point;
    predict_point.x = kf_ptr->predict_point.x;
    predict_point.y = kf_ptr->predict_point.y;
    predict_point.z = kf_ptr->predict_point.z;

    float x = predict_point.x;
    float y = predict_point.y;
    float z = predict_point.z;
    // float x = point_fly.x;
    // float y = point_fly.y;
    // float z = point_fly.z;
    // std::cout<<"预测坐标- X: "<<x<<", Y: "<<y<<", Z: "<<z<<std::endl;
    float dist_horizontal = std::sqrt(x * x + y * y);//提取数据

    float yaw_delta = std::atan2(y, x); 
    float pitch_abs = std::atan2(z, dist_horizontal);
    float yaw_cmd = yaw_delta * (180.0f / M_PI);
    float pitch_cmd = pitch_abs * (180.0f / M_PI);

    gimbal_interface::msg::GimbalAngle gimbal_msg;
    gimbal_msg.header.stamp = this->now();
    gimbal_msg.header.frame_id = "yaw_link"; 
    gimbal_msg.yaw = 0;   
    gimbal_msg.pitch = 0;
    gimbal_msg.is_fire = 1;
    // std::cout<<"Lock Command - Yaw: "<<gimbal_msg.yaw<<", Pitch: "<<gimbal_msg.pitch<<std::endl;
    gimbal_pub->publish(gimbal_msg);//发布数据（yaw为增量，pitch为绝对角度，相对于重力）
    // std::cout<<"Lock Callback!"<<std::endl;
}


}// namespace tdt_lock
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_lock::Lock)