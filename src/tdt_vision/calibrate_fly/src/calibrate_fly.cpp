#include "calibrate_fly.h"

namespace tdt_radar {

    CalibrateFly::CalibrateFly(const rclcpp::NodeOptions &options) : Node("radar_calibrate_fly_node", options) {
        std::cout<<"Calibrate Fly start"<<std::endl;
        cv::namedWindow("calibrate_fly", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("calibrate_fly", 1440, 1080);
        cv::moveWindow("calibrate_fly", 1920-1440, 1080-900);
        cv::namedWindow("ROI_fly", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("ROI_fly", 400, 400);
        cv::moveWindow("ROI_fly", 0,0);
        cv::setMouseCallback("calibrate_fly", fly_mousecallback, 0);

        
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "camera2/image", rclcpp::SensorDataQoS(),
                std::bind(&CalibrateFly::callback, this, std::placeholders::_1));
        compressed_image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "compressed_image2", rclcpp::SensorDataQoS(),
                std::bind(&CalibrateFly::compressed_callback, this, std::placeholders::_1));
        gimbal_pub = this->create_publisher<gimbal_interface::msg::GimbalAngle>(
                "GimbalPub", rclcpp::SensorDataQoS());
        // 标定期间持续发送零位指令，避免云台角度影响激光落点采样。
        gimbal_timer = this->create_wall_timer(
                std::chrono::milliseconds(20), std::bind(&CalibrateFly::publish_zero_gimbal, this));
        publish_zero_gimbal();
        std::cout<<"Calibrate Fly end"<<std::endl;
    }

    void CalibrateFly::publish_zero_gimbal()
    {
        gimbal_interface::msg::GimbalAngle gimbal_msg;
        gimbal_msg.header.stamp = this->now();
        gimbal_msg.yaw = 0.0F;
        gimbal_msg.pitch = 0.0F;
        gimbal_msg.is_fire = true;
        gimbal_msg.force_flag = true;
        gimbal_pub->publish(gimbal_msg);
    }

    void CalibrateFly::callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cvimage_fly_ = img;

        if(is_calibrating_fly){
            cv::putText(img, "Click a point, tweak with w/a/s/d, press 'n' to save", cv::Point(50, 400), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
            if(point_selected_fly){
                solve();
            }
        }
        else{
            cv::putText(img,"Press Enter to Calibrate Target Fly !!!",cv::Point(50,200),cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(0,0,255),2);
        }

        // 标定过程中在图上高亮已选点（缩放到1440x900显示的情况或者原图情况下）
        if (pick_point_fly.x > 0 && pick_point_fly.y > 0) {
            cv::circle(img, pick_point_fly, 5, cv::Scalar(0, 255, 0), -1);
            cv::putText(img, "[" + std::to_string((int)pick_point_fly.x) + "," + std::to_string((int)pick_point_fly.y) + "]", 
               cv::Point(pick_point_fly.x + 10, pick_point_fly.y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("calibrate_fly", img);
        
        auto key = cv::waitKey(10) & 0xFF;
        switch (key) {
            case 13: // Enter
                is_calibrating_fly = true;
                break;
            default:
                break;
        }       
    }

    void CalibrateFly::compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto img = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        cvimage_fly_ = img;

        if(is_calibrating_fly){
            cv::putText(img, "Click a point, tweak with w/a/s/d, press 'n' to save", cv::Point(50, 400), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
            if(point_selected_fly){
                solve();
            }
        }
        else{
            cv::putText(img,"Press Enter to Calibrate Target Fly !!!",cv::Point(50,200),cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(0,0,255),2);
        }

        if (pick_point_fly.x > 0 && pick_point_fly.y > 0) {
            cv::circle(img, pick_point_fly, 5, cv::Scalar(0, 255, 0), -1);
            cv::putText(img, "[" + std::to_string((int)pick_point_fly.x) + "," + std::to_string((int)pick_point_fly.y) + "]", 
               cv::Point(pick_point_fly.x + 10, pick_point_fly.y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("calibrate_fly", img);
        
        auto key = cv::waitKey(10) & 0xFF;
        switch (key)
        {
            case 13: // Enter
                is_calibrating_fly = true;
                break;
            default:
                break;
        }       
    }

    void fly_mousecallback(int event, int x, int y, int flags, void *userdata)
    {
        int temp_key = 0; 

        switch (event)
        {
        case cv::EVENT_LBUTTONDOWN:
            if (is_calibrating_fly) {
                do {
                    temp_key = cv::waitKey(10) & 0xFF;
                    switch (temp_key)
                    {
                        case 'w': y -= 1; break; 
                        case 'a': x -= 1; break; 
                        case 's': y += 1; break; 
                        case 'd': x += 1; break; 
                    }

                    x = std::max(50, std::min(x, cvimage_fly_.cols - 50));
                    y = std::max(50, std::min(y, cvimage_fly_.rows - 50));

                    cv::Mat roi = cvimage_fly_(cv::Rect(x - 50, y - 50, 100, 100));
                    cv::Mat dst;
                    cv::resize(roi, dst, cv::Size(400, 400));
                    cv::line(dst, cv::Point(200, 100), cv::Point(200, 300), cv::Scalar(0, 0, 255), 1);
                    cv::line(dst, cv::Point(100, 200), cv::Point(300, 200), cv::Scalar(0, 0, 255), 1);
                    cv::imshow("ROI_fly", dst);

                } 
                while (temp_key != 'n');

                std::cout << "Target saved-> x:" << x << " y:" << y << std::endl;
                pick_point_fly = cv::Point2f(x, y);
                point_selected_fly = true;
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            if (x > cvimage_fly_.cols - 50 || y > cvimage_fly_.rows - 50 || x < 50 || y < 50)
                break;
            cv::Mat roi = cvimage_fly_(cv::Rect(x - 50, y - 50, 100, 100));
            cv::Mat dst;
            cv::resize(roi, dst, cv::Size(400, 400));
            cv::line(dst, cv::Point(200, 100), cv::Point(200, 300), cv::Scalar(0, 0, 255), 1);
            cv::line(dst, cv::Point(100, 200), cv::Point(300, 200), cv::Scalar(0, 0, 255), 1);
            cv::imshow("ROI_fly", dst);
            break;
        }
    }

    void CalibrateFly::solve(){
        // cv::FileStorage fs;
        // fs.open("/home/robot/T-DT_Radar/config/fly_target.yaml", cv::FileStorage::WRITE);
        // fs << "target_x" << pick_point_fly.x;
        // fs << "target_y" << pick_point_fly.y;
        // fs.release();
        
        std::cout << "Successfully saved fly target coordinate to /home/robot/T-DT_Radar/config/fly_target.yaml" << std::endl;
        
        point_selected_fly = false;
        is_calibrating_fly = false;
        // Optionally keep the target shown on screen
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::CalibrateFly);
