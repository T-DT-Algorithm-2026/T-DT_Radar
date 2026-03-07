#include "detect_fly.h"
#include <sstream>
#include <iomanip>

namespace tdt_radar {
DetectFly::DetectFly(const rclcpp::NodeOptions& options)
    : Node("detect_fly_node", options)
{
    cv::namedWindow("detect_fly", cv::WINDOW_NORMAL);
    cv::namedWindow("mask", cv::WINDOW_NORMAL);

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
               "--onnx=model/ONNX/fly.onnx "
               "--saveEngine=model/TensorRT/fly.engine "
               "--minBatch 1 "
               "--optBatch 1 "
               "--maxBatch 2 "
               "--Shape=1280x1280 "
               "--input_name=images");
    } 
    else 
    {
        std::cout<<"Load yolo engine!"<<std::endl;
    }
    std::cout << "fly_path:" << fly_path << "\n";
    this->fly = yolo::load(fly_path, yolo::Type::V8, 0.4f, 0.45f);
    std::cout << "Load fly_yolo engine success!" << std::endl;


    this->declare_parameter<std::string>("save_dir", "./saved_images");
    this->get_parameter("save_dir", save_dir_);
    this->declare_parameter<bool>("save_images", true);
    this->get_parameter("save_images", save_images_);
    if (save_images_) {
        std::filesystem::create_directories(save_dir_);
    }
    last_save_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(500);

    createTrackbars();
    image_sub = this->create_subscription<sensor_msgs::msg::Image>("camera1/image", rclcpp::SensorDataQoS(),std::bind(&DetectFly::callback, this, std::placeholders::_1));
    resolve_pub = this->create_publisher<vision_interface::msg::ResolveResult>("fly_resolve_result", rclcpp::SensorDataQoS());
    player_control_pub_ = this->create_publisher<std_msgs::msg::String>("/rosbag_player/control", 10);
    RCLCPP_INFO(this->get_logger(), "Detect_fly node has been started.");

}

void DetectFly::callback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
    auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // img = cv::imread("/home/robot/T-DT_Radar/saved_images/0000.png");
    // img = cv::imread("/home/robot/T-DT_Radar/saved_images/0000.png", cv::IMREAD_COLOR);
    if (img.empty()) return;
    cv::Mat roi;

    // 每 0.5 秒保存一张原始图片（受 save_images 参数控制）
    // if (save_images_) 
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time_).count() >= 500) 
    //     {
    //         std::ostringstream oss;
    //         oss << save_dir_ << "/" << std::setw(4) << std::setfill('0') << image_save_counter_++ << ".png";
    //         cv::imwrite(oss.str(), img);
    //         RCLCPP_INFO(this->get_logger(), "Saved image: %s", oss.str().c_str());
    //         last_save_time_ = now;
    //     }
    // }
    
    // cv::Mat roi = img(roi_rect).clone();
    tdt_radar::Image image(img.data, img.cols, img.rows);

    auto result = fly->forward(image);
    if (result.size() == 0) 
    {
        RCLCPP_INFO(this->get_logger(), "No Fly!");
    }
    if(result.size() > 1) 
    {
        RCLCPP_INFO(this->get_logger(), "Too Many Fly!");
    }
    // std::cout<<"Detect Fly Num:"<<result.size()<<std::endl;

    for(int i=0;i<result.size();i++)
    {
        
        auto fly_rect = cv::Rect(result[i].left, result[i].top,result[i].right - result[i].left, result[i].bottom - result[i].top);
        std::cout<<"Fly Rect:"<<fly_rect.x<<","<<fly_rect.y<<","<<fly_rect.width<<","<<fly_rect.height<<std::endl;
        cv::rectangle(img, fly_rect, cv::Scalar(0, 255, 0), 1);
        cv::Rect safe_rect = getSafeRect(img, fly_rect);
        roi = img(safe_rect);
        cv::imshow("roi", roi);
        // std::cout<<"Fly Rect:"<<fly_rect.x<<","<<fly_rect.y<<","<<fly_rect.width<<","<<fly_rect.height<<std::endl;
    }
    // if (save_images_) 
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time_).count() >= 500) 
    //     {
    //         std::ostringstream oss;
    //         oss << save_dir_ << "/" << std::setw(4) << std::setfill('0') << image_save_counter_++ << ".png";
    //         cv::imwrite(oss.str(), roi);
    //         RCLCPP_INFO(this->get_logger(), "Saved image: %s", oss.str().c_str());
    //         last_save_time_ = now;
    //     }
    // }

    if(roi.empty())
    {
        cv::imshow("detect_fly", img);
        return;
    }

    h_min = cv::getTrackbarPos("H Min", "Control");
    h_max = cv::getTrackbarPos("H Max", "Control");
    s_min = cv::getTrackbarPos("S Min", "Control");
    s_max = cv::getTrackbarPos("S Max", "Control");
    v_min = cv::getTrackbarPos("V Min", "Control");
    v_max = cv::getTrackbarPos("V Max", "Control");
    g_min = cv::getTrackbarPos("G Min", "Control");
    g_max = cv::getTrackbarPos("G Max", "Control");
    //trackbar获取参数

    cv::GaussianBlur(roi, roi, cv::Size(3, 3), 0);
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
    cv::imshow("mask", mask);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::RotatedRect> rects;//二值化处理等
    std::vector<std::vector<cv::Point>> contours_pac;//储存pac要用的点

    cv::circle(img, cv::Point(img.cols/2, img.rows/2), 3, cv::Scalar(255, 0, 255), -1); //准心

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area=contourArea(contours[i]);
        if(area>50)
        {
            cv::RotatedRect rect = cv::minAreaRect(contours[i]);
            bool if1 = (fabs(rect.angle)<30&&rect.size.width>rect.size.height);
            bool if2 = (fabs(rect.angle)>60&&rect.size.width<rect.size.height);//筛选长方形
            if(if1||if2)
            {
                rects.push_back(rect);
                cv::Point2f vertices[4];
                rect.points(vertices);
                // for (int j = 0; j < 4; j++) 
                // {
                //     // cv::line(img, vertices[j], vertices[(j+1)%4], cv::Scalar(0, 0, 255), 2);
                // }
                // cv::circle(img, rects[i].center, 3, cv::Scalar(255, 0, 0), -1);
                contours_pac.push_back(contours[i]);
            }
        }
    }//第一次筛选

    std::vector<cv::RotatedRect> final_rect;
    std::vector<std::vector<cv::Point>> final_contours_pac;
    for(int i=0;i<rects.size();i++)
    {
        for(int j=0;j<i;j++)
        {
            if(i==j)
            {
                continue;
            }
            double angle_diff = fabs(rects[i].angle - rects[j].angle);
            bool if3=(angle_diff<10.0)||(angle_diff>80.0&&angle_diff<100.0)||(angle_diff>170.0);//平放
            double center_dist = cv::norm(rects[i].center - rects[j].center);
            bool if4 = center_dist <3*std::max(std::max(rects[i].size.width,rects[i].size.height),std::max(rects[j].size.width,rects[j].size.height));
            bool if5 = center_dist >std::max(std::max(rects[i].size.width,rects[i].size.height),std::max(rects[j].size.width,rects[j].size.height));//距离适中
            if(if3&&if4&&if5)
            {
                final_rect.push_back(rects[i]);
                final_rect.push_back(rects[j]);
                final_contours_pac.push_back(contours_pac[i]);
                final_contours_pac.push_back(contours_pac[j]);
                cv::rectangle(roi, rects[i].boundingRect(), cv::Scalar(0, 0, 0), 1);
                cv::rectangle(roi, rects[j].boundingRect(), cv::Scalar(0, 0, 0), 1);
            }

        }
    }//第二此筛选


    if(final_rect.size() == 2)
    {
        // cv::Mat gray;
        // cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        // cv::imshow("gray", gray);
        std::vector<cv::Mat> channel;
        cv::split(roi, channel);
        cv::Mat gray = channel[1];
        // cv::imshow("gray", gray);
        KeyPoints kp1 = pca_points(final_contours_pac[0],gray);//高的
        KeyPoints kp2 = pca_points(final_contours_pac[1],gray);//矮的
        if(kp1.center.y > kp2.center.y)
        {
            std::swap(kp1, kp2);
        }
        std::vector<cv::Point2f> image_points;
        image_points.push_back(kp1.left_mid);   // 左上
        image_points.push_back(kp1.center);     // 上中
        image_points.push_back(kp1.right_mid);  // 右上
        image_points.push_back(kp2.right_mid);  // 右下
        image_points.push_back(kp2.center);     // 下中
        image_points.push_back(kp2.left_mid);   // 左下
        // cv::line(img, kp1.left_mid, kp1.right_mid, cv::Scalar(255, 0, 255), 2);
        // cv::line(img, kp2.left_mid, kp2.right_mid, cv::Scalar(255, 0, 255), 2);
        cv::circle(roi, kp1.left_mid, 1, cv::Scalar(255, 0, 255), -1);
        cv::circle(roi, kp2.left_mid, 1, cv::Scalar(255, 0, 255), -1); 
        cv::circle(roi, kp2.right_mid, 1, cv::Scalar(255, 0, 255), -1); 
        cv::circle(roi, kp1.right_mid, 1, cv::Scalar(255, 0, 255), -1); 
        cv::circle(roi, kp1.center, 1, cv::Scalar(255, 0, 255), -1); 
        cv::circle(roi, kp2.center, 1, cv::Scalar(255, 0, 255), -1); 

        std::vector<cv::Point3f> object_points;
        double w = 0.066 / 2.0; // 半宽
        double h = 0.0945 / 2.0; // 半高
        std::cout<<"w:"<<w<<",h:"<<h<<std::endl;

        object_points.push_back(cv::Point3f(-w, -h, 0)); // 左上
        object_points.push_back(cv::Point3f(0, -h, 0));
        object_points.push_back(cv::Point3f( w, -h, 0)); // 右上
        object_points.push_back(cv::Point3f( w,  h, 0)); // 右下
        object_points.push_back(cv::Point3f(0, h, 0));
        object_points.push_back(cv::Point3f(-w,  h, 0)); // 左下

        double cam_D[] = { 12345.009079, 0.000000, 710.031779,
         0.000000, 12284.574745, 530.445895,
         0.000000, 0.000000, 1.000000}; //内参
        cv::Mat camera_matrix(3, 3, CV_64F, cam_D);
        double dist_D[] = {1.271697, 292.273363, 0.043964, -0.011520, 0.000000}; 
        cv::Mat dist_coeffs(1, 5, CV_64F, dist_D);


        // cv::Mat img_undistorted;
        // cv::undistort(img, img_undistorted, camera_matrix, dist_coeffs);
        // // cv::imshow("undistorted", img_undistorted);

        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);//pnp
        std::cout<<"tvec:"<<tvec.t()<<std::endl;
        vision_interface::msg::ResolveResult resolve_msg;
        resolve_msg.header.stamp = rclcpp::Clock().now();
        resolve_msg.header.frame_id = "camera_link";
        resolve_msg.pose.position.x = tvec.at<double>(0);
        resolve_msg.pose.position.y = tvec.at<double>(1);
        resolve_msg.pose.position.z = tvec.at<double>(2);
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        tf2::Matrix3x3 tf2_R(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
        );
        tf2::Quaternion q;
        tf2_R.getRotation(q);
        resolve_msg.pose.orientation.x = q.x();
        resolve_msg.pose.orientation.y = q.y();
        resolve_msg.pose.orientation.z = q.z();
        resolve_msg.pose.orientation.w = q.w();
        resolve_pub->publish(resolve_msg);//填充消息，四元数

        if (success) 
        {
            // --- 🔍【调试功能 B】PnP 结果可视化 ---
            
            // 1. 画距离
            double dist = tvec.at<double>(2);
            std::string dist_text = "Dist: " + std::to_string(dist).substr(0, 4) + "m";
            cv::Point2f text_pos = (kp1.center + kp2.center) * 0.5;
            cv::putText(roi, dist_text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            // 2. 画坐标轴 (验证姿态)
            // 应该看到：蓝色轴垂直指向屏幕内部，红色轴水平向右，绿色轴垂直向下
            // try 
            // {
            //     cv::drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, 0.05); // 0.05m 长的轴
            // } 
            // catch (const cv::Exception& e) 
            // {
            //     std::cout<<"版本太低"<<std::endl;
            // }

            std::vector<cv::Point2f> reprojected_points;
            cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected_points);
            double total_err = 0;
            for (size_t i = 0; i < object_points.size(); i++) 
            {
                double err = cv::norm(image_points[i] - reprojected_points[i]);
                total_err += err;
            }
            cv::circle(roi, (reprojected_points[2]+reprojected_points[5])/2, 1, cv::Scalar(0, 255, 255), -1); 
            cv::circle(roi, (image_points[2]+image_points[5])/2, 1, cv::Scalar(255, 255, 255), -1); 
            std::cout<<"Reprojected point:"<<(reprojected_points[2]+reprojected_points[5])/2<<std::endl;
            std::cout<<"Original point:"<<(image_points[2]+image_points[5])/2<<std::endl;
            double error = cv::norm((image_points[2]+image_points[5])/2 - (reprojected_points[2]+reprojected_points[5])/2);
            cv::putText(roi, "Err: " + std::to_string(error).substr(0, 4) + " px", cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            double mean_error = total_err / object_points.size();
            e+=error;
            de++;
            std::string err_str = "Err: " + std::to_string(mean_error).substr(0, 4) + " px";
            cv::putText(roi, err_str, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }
        
    }//对pnp验证
    else
    {
        std::cout<<"未检测到两个符合标准的灯条"<<std::endl;
    }
    // }

    cv::imshow("detect_fly", img);
    std::cout<<"Average Reprojection Error: "<<(e/de)<<std::endl;
    int key = cv::waitKey(1) & 0xFF; 

    // 如果按下了空格 (32) 或 'p'jiuzant
    if (key == 32 || key == 'p') 
    {
        std_msgs::msg::String msg;
        msg.data = "pause";
        player_control_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "▶ 已暂停：Rosbag停止，画面定格");
        bool wait_for_resume = true;
        while (wait_for_resume && rclcpp::ok()) 
        {
            int resume_key = cv::waitKey(30) & 0xFF; 
            if (resume_key == 32 || resume_key == 'p') 
            {
                wait_for_resume = false; 
            }
        }
        msg.data = "resume";
        player_control_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "▶ 已恢复：Rosbag继续播放");
    }
}

void DetectFly::createTrackbars() {
    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::createTrackbar("H Min", "Control", nullptr, 255);
    cv::createTrackbar("H Max", "Control", nullptr, 255);
    cv::createTrackbar("S Min", "Control", nullptr, 255);
    cv::createTrackbar("S Max", "Control", nullptr, 255);
    cv::createTrackbar("V Min", "Control", nullptr, 255);
    cv::createTrackbar("V Max", "Control", nullptr, 255);
    cv::createTrackbar("G Min", "Control", nullptr, 255);
    cv::createTrackbar("G Max", "Control", nullptr, 255);

    // 设置滑动条的初始位置
    cv::setTrackbarPos("H Min", "Control", h_min);
    cv::setTrackbarPos("H Max", "Control", h_max);
    cv::setTrackbarPos("S Min", "Control", s_min);
    cv::setTrackbarPos("S Max", "Control", s_max);
    cv::setTrackbarPos("V Min", "Control", v_min);
    cv::setTrackbarPos("V Max", "Control", v_max);
    cv::setTrackbarPos("G Min", "Control", g_min);
    cv::setTrackbarPos("G Max", "Control", g_max);
}//滑动条函数

KeyPoints DetectFly::pca_points(const std::vector<cv::Point>& contours, const cv::Mat& img)
{
    cv::Mat data_pts(contours.size(), 2, CV_32F);
    for (int i = 0; i < data_pts.rows; ++i) 
    {
        data_pts.at<float>(i, 0) = contours[i].x;
        data_pts.at<float>(i, 1) = contours[i].y;
    }
    // 构造 PCA 对象
    // 参数1: 输入数据
    // 参数2: 均值 (传空 cv::Mat() 让它自动计算)
    // 参数3: 数据排列方式 (CV_PCA_DATA_AS_ROW 表示每一行是一个样本/一个点)
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

    //提取主成分
    cv::Point2f center = cv::Point2f(pca_analysis.mean.at<float>(0, 0),
                                   pca_analysis.mean.at<float>(0, 1));
    // 主方向
    cv::Point2f dir1 = cv::Point2f(pca_analysis.eigenvectors.at<float>(0, 0),
                                   pca_analysis.eigenvectors.at<float>(0, 1));
    // 次方向
    cv::Point2f dir2 = cv::Point2f(pca_analysis.eigenvectors.at<float>(1, 0),
                                   pca_analysis.eigenvectors.at<float>(1, 1));
    if(dir2.y<0)
    {
        dir2 = -dir2;
    }
    if(dir1.x<0)
    {
        dir1 = -dir1;
    }//保证方向向下
    cv::Point2f bottom = grad_search(img, center, dir2);
    cv::Point2f top = grad_search(img, center, -dir2);
    cv::Point2f final_center = (bottom + top) *0.5;
    cv::Point2f left_mid = grad_search(img, final_center, -dir1);
    cv::Point2f right_mid = grad_search(img, final_center, dir1);
    KeyPoints keypoints;
    keypoints.center = final_center;
    keypoints.left_mid = left_mid;
    keypoints.right_mid = right_mid;//搜索后赋值
    float height = cv::norm(top - bottom);
    float width = cv::norm(left_mid - right_mid);
    // std::cout<<"Height:"<<height<<", Width:"<<width<<std::endl;
    return keypoints;
}//pca提取关键点


cv::Point2f DetectFly::grad_search(const cv::Mat& img, const cv::Point2f& center, const cv::Point2f& direction)
{
    int delete_val = -1;
    cv::Point2f final_point = cv::Point2f(-1.0f, -1.0f);    
    cv::Point2f dir_norm;
    float norm_value = std::sqrt(direction.x*direction.x + direction.y*direction.y);
    dir_norm.x = direction.x / norm_value;
    dir_norm.y = direction.y / norm_value;
    cv::Point2f direction_final = dir_norm;
    for(int i =0;i<200;i++)
    {
        cv::Point2f next_point = center + direction_final*(i+1);
        cv::Point2f last_point = center + direction_final*i;
        if(next_point.x>=0&&next_point.x<img.cols&&next_point.y>=0&&next_point.y<img.rows)
        {
            uchar next_value = img.at<uchar>(next_point);
            uchar last_value = img.at<uchar>(last_point);
            int delete_temp= last_value - next_value;
            if(delete_temp>delete_val)
            {
                delete_val = delete_temp;
                final_point = next_point;
            }
            if(next_value<30)
            {
                break;
            }//过小提前退出
        }
        else
        {
            break;
        }
    }
    if(final_point.x == -1.0f && final_point.y == -1.0f)
    {
        final_point = center+ direction;
    }
    return final_point;
}//梯度找差值最大点


cv::Rect DetectFly::getSafeRect(cv::Mat& image, cv::Rect& rect)
{
    cv::Rect save_rect;
    save_rect.x = std::max(0, rect.x);
    save_rect.y = std::max(0, rect.y);
    save_rect.width = std::min(image.cols - save_rect.x, rect.width);
    save_rect.height = std::min(image.rows - save_rect.y, rect.height);
    return save_rect;
}//安全裁剪函数，防止越界

}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::DetectFly)