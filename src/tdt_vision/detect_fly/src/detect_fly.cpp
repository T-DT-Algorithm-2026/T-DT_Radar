#include "detect_fly.h"
#include <sstream>
#include <iomanip>

namespace tdt_radar {
DetectFly::DetectFly(const rclcpp::NodeOptions& options)
    : Node("detect_fly_node", options)
{
    // cv::namedWindow("roi", cv::WINDOW_NORMAL);
    // cv::namedWindow("bin_img", cv::WINDOW_NORMAL);
    // cv::namedWindow("detect_fly", cv::WINDOW_OPENGL);

    // 使用system函数调用nvidia-smi命令
    std::cout << "Checking CUDA with nvidia-smi...\n";
    if (system("nvidia-smi") == 0) {
        RCLCPP_INFO(this->get_logger(), "CUDA is available.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "CUDA is not available. Exiting.");
        rclcpp::shutdown();
    }
    cv::FileStorage fs;
    fs.open("./config/detect_params.yaml", cv::FileStorage::READ);
    fs["fly_path"] >> fly_path;
    fs.release();
    std::ifstream file1(fly_path.c_str());
    if (!file1.good()) 
    {
        system("python3 src/utils/onnx2trt.py "
               "--onnx=model/ONNX/fly_all.onnx "
               "--saveEngine=model/TensorRT/fly_all.engine "
               "--minBatch 1 "
               "--optBatch 1 "
               "--maxBatch 2 "
               "--Shape=960x1280 "
               "--input_name=images");
    } 
    else 
    {
        std::cout<<"Load yolo engine!"<<std::endl;
    }
    std::cout << "fly_path:" << fly_path << "\n";
    this->fly = yolo::load(fly_path, yolo::Type::V8, 0.6f, 0.45f);
    std::cout << "Load fly_yolo engine success!" << std::endl;

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

    if (save_images_) 
    {
        std::filesystem::create_directories(save_dir_);
    }
    last_save_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(500);
    
    image_sub = this->create_subscription<sensor_msgs::msg::Image>("camera2/image", rclcpp::SensorDataQoS().keep_last(1),std::bind(&DetectFly::callback, this, std::placeholders::_1));
    player_control_pub_ = this->create_publisher<std_msgs::msg::String>("/rosbag_player/control", 10);
    fly_pub_ = this->create_publisher<vision_interface::msg::DetectFly>("detect_fly", rclcpp::SensorDataQoS().keep_last(1));
    debug_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("debug_image/compressed", rclcpp::SensorDataQoS().keep_last(1));
    lidar_sub = this->create_subscription<geometry_msgs::msg::Point32>("/livox/lidar_fly_point", 10, std::bind(&DetectFly::lidar_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Detect_fly node has been started.");

}

void DetectFly::lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg)
{
    float distance = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    lidar_time = this->now();
    
    // 如果启用了动态靶心，根据当前距离实时更新 target_point
    target_point.x = A_x / distance + B_x;
    target_point.y = A_y / distance + B_y;
}

void DetectFly::callback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
    auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
    rclcpp::Time time_stamp = msg->header.stamp;
    std::chrono::steady_clock::time_point begin =std::chrono::steady_clock::now();
    const rclcpp::Time begin_ros = this->now();
    const double timestamp_to_begin_ms = static_cast<double>((begin_ros - time_stamp).nanoseconds()) / 1.0e6;
    std::cout << "Begin - time_stamp: " << timestamp_to_begin_ms << " ms" << std::endl;
    if (img.empty()) return;
    const auto debug_now = std::chrono::steady_clock::now();
    const bool publish_debug_frame = (debug_img_pub_->get_subscription_count() > 0 || debug_img_pub_->get_intra_process_subscription_count() > 0) && (debug_now - last_debug_pub_time_ >= std::chrono::milliseconds(50));
    cv::Mat roi;
    bool save_images_ = false;

    // 每 0.5 秒保存一张原始图片（受 save_images 参数控制）
    if (save_images_) 
    {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time_).count() >= 200) 
        {
            std::ostringstream oss;
            oss << save_dir_ << "/" << std::setw(4) << std::setfill('0') << image_save_counter_++ << ".png";
            cv::imwrite(oss.str(), img);
            RCLCPP_INFO(this->get_logger(), "Saved image: %s", oss.str().c_str());
            last_save_time_ = now;
        }
    }
    
    tdt_radar::Image image(img.data, img.cols, img.rows);

    auto result = fly->forward(image);
    if (result.size() == 0) 
    {
        // RCLCPP_INFO(this->get_logger(), "No Fly!");
        if (publish_debug_frame) {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";
            const std::vector<int> compression_params = { cv::IMWRITE_JPEG_QUALITY, 50 };
            if (cv::imencode(".jpg", img, compressed_msg.data, compression_params)) 
            {
                last_debug_pub_time_ = debug_now;
                debug_img_pub_->publish(std::move(compressed_msg));
            }
        }

        // if((this->now().seconds() - lidar_time.seconds()) < 1)
        // {
        //     cv::circle(img, target_point, 1, cv::Scalar(255, 0, 255), -1); //准心
        // }
        // else
        // {
        //     cv::circle(img, cv::Point(720, 540), 1, cv::Scalar(255, 0, 255), -1); //准心
        // }
        // cv::imshow("detect_fly", img);
        // cv::waitKey(1);

        // std::chrono::steady_clock::time_point end =std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_used =std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
        // std::cout << "Detect Fly Time: " << time_used.count() * 1000 << "ms" << std::endl;
        return;
    }
    auto best_result = std::max_element(
        result.begin(), result.end(),
        [](const yolo::Box& a, const yolo::Box& b) {
            return a.confidence < b.confidence;
        });
    if(result.size() > 1) 
    {
        RCLCPP_INFO(this->get_logger(), "Too Many Fly! Select confidence: %.3f", best_result->confidence);
    }
    // std::cout<<"Detect Fly Num:"<<result.size()<<std::endl;

    auto fly_rect = cv::Rect(best_result->left, best_result->top,best_result->right - best_result->left, best_result->bottom - best_result->top);//1376
    cv::rectangle(img, fly_rect, cv::Scalar(0, 255, 0), 1);
    cv::Rect safe_rect = getSafeRect(img, fly_rect);
    roi = img(safe_rect);

    //预处理
    cv::Mat hsv_img, bin_img;
    cv::cvtColor(roi, hsv_img, cv::COLOR_BGR2HSV);     
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_img, hsv_channels); 
    cv::Mat v_channel = hsv_channels[2]; 
    cv::threshold(v_channel, bin_img, 220, 255, cv::THRESH_BINARY);

    float refined_x = fly_rect.x + fly_rect.width / 2.0f; 
    float refined_y = fly_rect.y + fly_rect.height / 2.0f;

    //找质心
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);       
    // std::vector<cv::Point2f> centroids;
    // for (const auto& contour : contours) 
    // {
    //     if (cv::contourArea(contour) > 5.0) 
    //     { 
    //         cv::Moments M = cv::moments(contour);
    //         if (M.m00 > 0) 
    //         {
    //             centroids.push_back(cv::Point2f(M.m10 / M.m00, M.m01 / M.m00));
    //         }
    //     }
    // }

    // if (centroids.size() >= 2) 
    // {
    //     // Step 3: 动态上下半区分组
    //     float mean_y = 0.0f;
    //     for (const  auto& pt : centroids) 
    //     {
    //         mean_y += pt.y;
    //     }
    //     mean_y /= centroids.size();
    //     std::vector<cv::Point2f> upper_pts;
    //     std::vector<cv::Point2f> lower_pts;
    //     for (const auto& pt : centroids) 
    //     {
    //         if (pt.y < mean_y) 
    //         {
    //             upper_pts.push_back(pt);
    //         }
    //         else 
    //         {
    //             lower_pts.push_back(pt);
    //         }
    //     }

    //     //拟合直线中心
    //     cv::Point2f upper_center, lower_center;
    //     bool has_upper = getLineCenter(upper_pts, upper_center);
    //     bool has_lower = getLineCenter(lower_pts, lower_center);

    //     //得到中心
    //     if (has_upper && has_lower) 
    //     {
    //         refined_x = safe_rect.x + (upper_center.x + lower_center.x) / 2.0f;
    //         refined_y = safe_rect.y + (upper_center.y + lower_center.y) / 2.0f;
    //     } 
    //     else if (has_upper) 
    //     { 
    //         refined_x = safe_rect.x + upper_center.x;
    //         refined_y = safe_rect.y + upper_center.y;
    //     } 
    //     else if (has_lower) 
    //     {
    //         refined_x = safe_rect.x + lower_center.x;
    //         refined_y = safe_rect.y + lower_center.y;
    //     }// 极限情况：一面被全遮挡
    // }


    vision_interface::msg::DetectFly test_msg;
    test_msg.x = refined_x;
    test_msg.y = refined_y;
    test_msg.header.stamp = time_stamp;
    fly_pub_->publish(test_msg);
    std::chrono::steady_clock::time_point end_pub =std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_pub =std::chrono::duration_cast<std::chrono::duration<double>>(end_pub - begin);
    std::cout << "Publish Time: " << time_used_pub.count() * 1000 << "ms" << std::endl;
    cv::circle(img, cv::Point2f(test_msg.x, test_msg.y), 1, cv::Scalar(255, 255, 0), -1);

    if((this->now().seconds() - lidar_time.seconds()) < 1)
    {
        cv::circle(img, target_point, 1, cv::Scalar(255, 0, 255), -1); //准心
    }
    else
    {
        cv::circle(img, cv::Point(720, 540), 1, cv::Scalar(255, 0, 255), -1); //准心
    } //准心
    // std::cout<<"target_point:"<<target_point.x<<","<<target_point.y<<std::endl;

    // cv::imshow("detect_fly", img);
    // int key = cv::waitKey(1) & 0xFF; 

    //     // 创建压缩图像消息
    if (publish_debug_frame) {
        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header = msg->header;
        compressed_msg.format = "jpeg";
        const std::vector<int> compression_params = { cv::IMWRITE_JPEG_QUALITY, 50 };
        if (cv::imencode(".jpg", img, compressed_msg.data, compression_params)) 
        {
            last_debug_pub_time_ = debug_now;
            debug_img_pub_->publish(std::move(compressed_msg));
        }
    }


    // 如果按下了空格 (32) 或 'p'jiuzant
    // if (key == 32 || key == 'p') 
    // {
    //     std_msgs::msg::String msg;
    //     msg.data = "pause";
    //     player_control_pub_->publish(msg);
    //     RCLCPP_INFO(this->get_logger(), "▶ 已暂停：Rosbag停止，画面定格");
    //     bool wait_for_resume = true;
    //     while (wait_for_resume && rclcpp::ok()) 
    //     {
    //         int resume_key = cv::waitKey(30) & 0xFF; 
    //         if (resume_key == 32 || resume_key == 'p') 
    //         {
    //             wait_for_resume = false; 
    //         }
    //     }
    //     msg.data = "resume";
    //     player_control_pub_->publish(msg);
    //     RCLCPP_INFO(this->get_logger(), "▶ 已恢复：Rosbag继续播放");
    // }
    std::chrono::steady_clock::time_point end =std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
    // std::cout << "Detect Fly Time2: " << time_used.count() * 1000 << "ms" << std::endl;
}

cv::Rect DetectFly::getSafeRect(cv::Mat& image, cv::Rect& rect)
{
    cv::Rect save_rect;
    save_rect.x = std::max(0, rect.x);
    save_rect.y = std::max(0, rect.y);
    save_rect.width = std::min(image.cols - save_rect.x, rect.width);
    save_rect.height = std::min(image.rows - save_rect.y, rect.height);
    return save_rect;
} // 安全裁剪函数，防止越界

bool DetectFly::getLineCenter(const std::vector<cv::Point2f>& pts, cv::Point2f& center)
{
    if (pts.empty()) 
    {
        return false;
    }
    if (pts.size() == 1) 
    {
        center = pts[0];
        return true;
    }// 仅有一个碎片，退化为质心
    // 鲁棒直线拟合 (Huber)
    cv::Vec4f line;
    cv::fitLine(pts, line, cv::DIST_HUBER, 0, 0.01, 0.01);
    float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
    
    if (std::abs(vx) < 1e-5f) 
    { 
        center = cv::Point2f(x0, y0);
        return true;
    }// 防止除零
    
    // 几何中心投影
    float min_x = pts[0].x, max_x = pts[0].x;
    for (const auto& pt : pts) 
    {
        if (pt.x < min_x) min_x = pt.x;
        if (pt.x > max_x) max_x = pt.x;
    }
    float mid_x = (min_x + max_x) / 2.0f;
    // 将 X 中点带入直线方程求 Y
    float mid_y = (vy / vx) * (mid_x - x0) + y0;
    center = cv::Point2f(mid_x, mid_y);
    return true;
}

}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::DetectFly)
