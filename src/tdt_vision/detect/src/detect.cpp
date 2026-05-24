#include "detect.h"

#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#define TDT_INFO(msg) std::cout << msg << std::endl
#define MAX_CARS 12
#define MAX_ARMORS 20
namespace tdt_radar {
unsigned int count_img = 0;

int getColor(cv::Mat& img)
{
    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    cv::Mat    blueMinusRed = channels[0] - channels[2];
    cv::Mat    redMinusBlue = channels[2] - channels[0];
    cv::Scalar avgBlueMinusRed = cv::mean(blueMinusRed);
    cv::Scalar avgRedMinusBlue = cv::mean(redMinusBlue);
    cv::Scalar avgGreen = cv::mean(channels[1]);
    if (avgBlueMinusRed[0] > avgRedMinusBlue[0]) {
        return 0;
    } else {
        return 2;
    }
}

cv::Point2f Detect::get_2d(const cv::Point3f& point)
{
    cv::Mat world_rvec;
    cv::Mat world_tvec;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::FileStorage fs;
    std::vector<cv::Point3f> objectPoints = {point};
    fs.open("./config/camera_params.yaml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs.release();

    fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
    fs["world_tvec"] >> world_tvec;
    fs["world_rvec"] >> world_rvec;
    fs.release();//读取参数
    
    std::vector<cv::Point2f> temp;
    cv::projectPoints(objectPoints,
                    world_rvec,
                    world_tvec,
                    camera_matrix,
                    dist_coeffs,
                    temp);
    return temp[0];
}

bool isRectInside(const cv::Rect& small, const cv::Rect& big)
{
    bool topLeftInside = big.contains(small.tl());
    bool topRightInside =
        big.contains(cv::Point(small.x + small.width, small.y));
    bool bottomLeftInside =
        big.contains(cv::Point(small.x, small.y + small.height));
    bool bottomRightInside = big.contains(
        cv::Point(small.x + small.width, small.y + small.height));
    return (topLeftInside && topRightInside && bottomLeftInside &&
            bottomRightInside);
}

bool isBoxInside(const yolo::Box& small, const yolo::Box& big)
{
    cv::Rect small_rect(small.left, small.top, small.right - small.left,
                        small.bottom - small.top);
    cv::Rect big_rect(big.left, big.top, big.right - big.left,
                      big.bottom - big.top);
    return isRectInside(small_rect, big_rect);
}

cv::Rect getSafeRect(cv::Mat& image, cv::Rect& rect)
{
    cv::Rect save_rect;
    save_rect.x = std::max(0, rect.x);
    save_rect.y = std::max(0, rect.y);
    save_rect.width = std::min(image.cols - save_rect.x, rect.width);
    save_rect.height = std::min(image.rows - save_rect.y, rect.height);
    return save_rect;
}

Detect::Detect(const rclcpp::NodeOptions& node_options)
    : Node("radar_detect_node", node_options)
{
    cv::namedWindow("detect", cv::WINDOW_NORMAL);

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
    fs["yolo_path"] >> yolo_path;
    fs["armor_path"] >> armor_path;
    fs["classify_path"] >> classify_path;
    fs.release();

    fs.open("./config/locate_points.yaml", cv::FileStorage::READ);
    fs["locate_points"] >> locate_points;
    fs.release();//读取参数

    std::ifstream file1(yolo_path.c_str());
    if (!file1.good()) {
        system("python3 src/utils/onnx2trt.py "
               "--onnx=model/ONNX/RM2025.onnx "
               "--saveEngine=model/TensorRT/yolo.engine "
               "--minBatch 1 "
               "--optBatch 1 "
               "--maxBatch 2 "
               "--Shape=1280x1280 "
               "--input_name=images");
    } else {
        TDT_INFO("Load yolo engine!");
    }
    std::ifstream file2(armor_path.c_str());
    if (!file2.good()) {
        system("python3 src/utils/onnx2trt.py "
               "--onnx=model/ONNX/armor_yolo.onnx "
               "--saveEngine=model/TensorRT/armor_yolo.engine "
               "--minBatch 1 "
               "--optBatch 5 "
               "--maxBatch 12 "
               "--Shape=192x192 "
               "--input_name=images");
    } else {
        TDT_INFO("Load armor_yolo engine!");
    }
    std::ifstream file3(classify_path.c_str());
    if (!file3.good()) {
        system("python3 src/utils/onnx2trt.py "
               "--onnx=model/ONNX/classify.onnx "
               "--saveEngine=model/TensorRT/classify.engine "
               "--minBatch 1 "
               "--optBatch 10 "
               "--maxBatch 20 "
               "--Shape=224x224 "
               "--input_name=input");
    } else {
        TDT_INFO("Load classify engine!");
    }
    std::cout << "yolo_path:" << yolo_path << "\n";
    std::cout << "armor_path:" << armor_path << "\n";
    std::cout << "classify_path:" << classify_path << "\n";
    // this->densenet121 = new
    // densenet121::densenet121_classifier("model/densenet.engine");
    this->classifier =
        classify::load(classify_path, classify::Type::densenet121);
    TDT_INFO("Load classify engine success!");

    this->armor_yolo = yolo::load(armor_path, yolo::Type::V8, 0.4f, 0.45f);
    TDT_INFO("Load armor_yolo engine success!");
    // this->yolo = yolo::load(yolo_path, yolo::Type::V5);
    this->yolo = yolo::load(yolo_path, yolo::Type::V8, 0.65f, 0.45f);
    TDT_INFO("Load yolo engine success!");
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "camera1/image", rclcpp::SensorDataQoS(),
            std::bind(&Detect::callback, this, std::placeholders::_1));
    fly_sub = this->create_subscription<vision_interface::msg::FlyPoints>(
        "/livox/lidar_fly_cluster", rclcpp::SensorDataQoS(),
        std::bind(&Detect::fly_callback, this, std::placeholders::_1));
    pub = this->create_publisher<vision_interface::msg::DetectResult>(
        "detect_result", rclcpp::SensorDataQoS());
    radar_warn_pub = this->create_publisher<vision_interface::msg::RadarWarn>(
        "lidar_detect", 10);
    RCLCPP_INFO(this->get_logger(), "Detect node has been started.");
}

void Detect::callback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
    // std::cout << msg->header.stamp.sec << "." << msg->header.stamp.nanosec
    //           << std::endl;
    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    std::cout << "Detecting..." << std::endl;
    auto        img = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (locate_points.size() == 4) 
    {
        cv::Rect rect1(locate_points[0], locate_points[1]);
        cv::Rect rect2(locate_points[2], locate_points[3]);
        
        rect1 &= cv::Rect(0, 0, img.cols, img.rows);
        rect2 &= cv::Rect(0, 0, img.cols, img.rows);

        auto count_high_v = [&img](const cv::Rect& rect) {
            if (rect.area() <= 0) {
                return 0;
            }

            cv::Mat hsv_img;
            cv::cvtColor(img(rect), hsv_img, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> hsv_channels;
            cv::split(hsv_img, hsv_channels);
            cv::Mat v_mask;
            cv::inRange(hsv_channels[2], 180, 255, v_mask);
            return cv::countNonZero(v_mask);
        };

        int rect1_count = count_high_v(rect1);
        int rect2_count = count_high_v(rect2);
        int roi_count_sum = rect1_count + rect2_count;

        std::cout << "Locate ROI V>=180 count: rect1=" << rect1_count
                  << ", rect2=" << rect2_count
                  << ", sum=" << roi_count_sum << std::endl;

        vision_interface::msg::RadarWarn lidar_detect;
        lidar_detect.base_state = 1;
        lidar_detect.out_post_state = roi_count_sum > 10 ? 1 : 0;
        radar_warn_pub->publish(lidar_detect);

        cv::rectangle(img, rect1, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(img, rect2, cv::Scalar(255, 0, 0), 2);
        cv::putText(img, std::to_string(rect1_count), rect1.tl(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 125), 2);
        cv::putText(img, std::to_string(rect2_count), rect2.tl(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
    }

    tdt_radar::Image image(img.data, img.cols, img.rows);

    auto result = yolo->forward(image);
    if (result.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "No Car!");
        cv::Mat final_img;
        cv::resize(img, final_img, cv::Size(1536, 1125));
        cv::imshow("detect", final_img);
        cv::waitKey(1);
        return;
    } else if (result.size() > MAX_CARS) {
        RCLCPP_INFO(this->get_logger(), "Too Many Car!");
        cv::Mat final_img;
        cv::resize(img, final_img, cv::Size(1536, 1125));
        cv::imshow("detect", final_img);
        cv::waitKey(1);
        return;
    }

    std::vector<tdt_radar::Image> images;
    std::vector<cv::Mat>     car_imgs;
    std::vector<Car>         cars;
    for (auto& box : result) {
        if (box.class_label == 0 || box.class_label == 1) {
            Car car;
            car.car = box;
            // if(box.class_label==0)car.color=2;
            // if(box.class_label==1)car.color=0;
            cars.push_back(car);
        }
    }

    for (auto& car : cars) {
        auto     temp_rect = cv::Rect(car.car.left, car.car.top,
                                      car.car.right - car.car.left,
                                      car.car.bottom - car.car.top);
        cv::Rect temp_car_rect = getSafeRect(img, temp_rect);
        auto     car_img = img(temp_car_rect);
        // 如果车的中心在图片的上半部分，则保存
        //  float temp_y=(car.car.top+car.car.bottom)/2;
        //  if(temp_y<img.rows/2)
        //  {
        //    cv::imwrite("/home/tdt/Label/2024cars/"+std::to_string(count_img++)+".jpg",car_img);
        //  }
        car_imgs.push_back(car_img.clone());
        car.car_rect = temp_car_rect;
    }  // 将car的图片存储到car_imgs中

    for (auto& car_img : car_imgs) {
        auto image = tdt_radar::Image(car_img.data, car_img.cols, car_img.rows);
        images.push_back(image);
    }

    auto armor_boxes = armor_yolo->forwards(images);
    bool has_armor = false;
    for (int i = 0; i < armor_boxes.size(); i++) {
        if (armor_boxes[i].size() == 0) {
            continue;
        } else {
            cars[i].armors = armor_boxes[i];
            has_armor = true;
        }
    }  // 将armor_boxes存储到cars中
    if (!has_armor) {
        RCLCPP_INFO(this->get_logger(), "No Armor!");
        return;
    }
    // for(int i=0;i<cars.size();i++){
    //   if(cars[i].armors.size()==0){continue;}
    //   for(auto &armor:cars[i].armors){
    //     cv::Rect rect_img(
    //       armor.left+cars[i].car.left,
    //       armor.top+cars[i].car.top,
    //       armor.right-armor.left,
    //       armor.bottom-armor.top);
    //     // cv::rectangle(img,rect_img,cv::Scalar(255,255,255),2);
    //     cv::imwrite("/home/tdt/Label/armor_detect/"+std::to_string(count_img++)+".jpg",img(rect_img));
    //     }
    // }//这段是用来打印或者保存armor的图片
    std::vector<cv::Mat>         armor_imgs;
    std::vector<classify::Image> armor_images;

    for (auto& car : cars) {
        if (car.armors.size() == 0) {
            continue;
        }
        for (auto& armor : car.armors) {
            cv::Rect rect_img_1(
                armor.left + car.car.left, armor.top + car.car.top,
                armor.right - armor.left, armor.bottom - armor.top);
            cv::Rect rect_img = getSafeRect(img, rect_img_1);
            auto     armor_img = img(rect_img);
            armor_imgs.push_back(armor_img.clone());
        }
    }  // 将armor的图片存储到armor_imgs中
    for (auto& armor_img : armor_imgs) {
        auto image =
            classify::Image(armor_img.data, armor_img.cols, armor_img.rows);
        armor_images.push_back(image);
    }  // 将armor_imgs转换为classify::Image
    auto armor_result = classifier->forwards(armor_images);
    // std::vector<int> armor_result;

    // 保存用来训练分类
    //  for(int i=0;i<armor_imgs.size();i++){
    //    cv::imwrite("/home/mozihe/Label/fenqu_armor/"+std::to_string(armor_result[i])+"/"+std::to_string(count_img++)+".jpg",armor_imgs[i]);
    //  }
    // 按照相同的顺序将armor的分类结果存储到Cars中
    for (auto& car : cars) {
        if (car.armors.size() == 0) {
            continue;
        }
        for (auto& armor : car.armors) {
            armor.class_label = armor_result[0];
            armor_result.erase(armor_result.begin());
            if (debug) {
                cv::putText(img, std::to_string(armor.class_label),
                            cv::Point(armor.left + car.car.left,
                                      armor.top + car.car.top),
                            cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(255, 255, 255), 2);
            }
        }
    }
    // 接下来对识别结果进行处理
    // 1.将车和装甲板的信息存储到detect_result中
    vision_interface::msg::DetectResult detect_result;
    for (auto& car : cars) {
        if (car.armors.size() == 0) {
            continue;
        }
        // 找到置信度最大的非0armor
        cv::Rect max_rect;
        float    max_confidence = 0;
        for (auto& armor : car.armors) {
            if (armor.class_label != 0 &&
                armor.class_label != 5 &&
                armor.confidence > max_confidence) {
                max_rect = cv::Rect(
                    armor.left + car.car.left, armor.top + car.car.top,
                    armor.right - armor.left, armor.bottom - armor.top);
                max_confidence = armor.confidence;
                car.number = armor.class_label;
            }
        }
        if (max_confidence == 0) {
            if (debug)
                cv::rectangle(img, car.car_rect, cv::Scalar(255, 255, 255),
                              2);
            continue;
        }
        auto safe_rect = getSafeRect(img, max_rect);
        auto max_mat = img(safe_rect);
        // cv::rectangle(img,safe_rect,cv::Scalar(255,255,255),2);

        car.color = getColor(max_mat);
        car.center = cv::Point2f(max_rect.x + max_rect.width / 2,
                                 max_rect.y + max_rect.height / 2);
        // car.center_rect=cv::Rect(car.center.x-10,car.center.y-10,20,20);
        if (car.color == 0) {
            detect_result.blue_x[car.number - 1] = car.center.x;
            detect_result.blue_y[car.number - 1] = car.center.y;
            // 如果只有一个为0，打印error
            if (car.center.x * car.center.y == 0 && car.number != 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Error: blue car center is 0");
            }
            if (debug) {
                cv::rectangle(img, car.car_rect, cv::Scalar(255, 0, 0), 2);
                cv::putText(img, std::to_string(car.car.confidence),
                            cv::Point(car.car.left, car.car.top),
                            cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(255, 255, 255), 2);
            }
        }
        if (car.color == 2) {
            detect_result.red_x[car.number - 1] = car.center.x;
            detect_result.red_y[car.number - 1] = car.center.y;
            // 如果只有一个为0，打印error
            if (car.center.x * car.center.y == 0 && car.number != 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Error: red car center is 0");
            }
            if (debug) {
                cv::rectangle(img, car.car_rect, cv::Scalar(0, 0, 255), 2);
                cv::putText(img, std::to_string(car.car.confidence),
                            cv::Point(car.car.left, car.car.top),
                            cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(255, 255, 255), 2);
            }
        }
        if (car.color == 1) {
            if (debug)
                cv::rectangle(img, car.car_rect, cv::Scalar(255, 255, 255),
                              2);
        }
    }
    detect_result.header.stamp = msg->header.stamp;
    pub->publish(detect_result);
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                  begin);
    // RCLCPP_INFO(this->get_logger(), "Time used: %fms",
    // time_used.count()*1000);
    std::cout << "Detect Time: " << time_used.count() * 1000 << "ms"
              << std::endl;
    
// if(fly_point.z && fly_point.x && fly_point.y)
// {
//     cv::Point2f fly_2d = get_2d(fly_point);

//     // --- 越界拉回逻辑：确保输出永远是 1440x1080 ---
//     int crop_w = 339;
//     int crop_h = 254;

//     // 1. 初步计算左上角 (让中心对准目标)
//     int x = static_cast<int>(fly_2d.x - crop_w / 2);
//     int y = static_cast<int>(fly_2d.y - crop_h / 2);

//     // 2. 越界拉回：如果左/上出界，设为0；如果右/下出界，设为最大允许值
//     x = std::max(0, std::min(x, img.cols - crop_w));
//     y = std::max(0, std::min(y, img.rows - crop_h));

//     // 3. 裁剪并保存
//     cv::Rect roi(x, y, crop_w, crop_h);
//     // cv::imwrite("/home/robot/桌面/Radar_date/test10/" + std::to_string(count_img++) + ".png", img(roi));
//     // ------------------------------------------

//     // 原有逻辑
//     cv::rectangle(img, cv::Point(fly_2d.x - 200, fly_2d.y - 150), 
//                   cv::Point(fly_2d.x + 200, fly_2d.y + 150), cv::Scalar(0, 0, 255), 2);
// }

    cv::Mat final_img;
    cv::resize(img, final_img, cv::Size(1536, 1125));
    cv::imshow("detect", final_img);
    auto key = cv::waitKey(1);
    if (key == 'r') {
        debug = !debug;
    }
}


void Detect::fly_callback(const std::shared_ptr<vision_interface::msg::FlyPoints> msg)
{
    fly_point.x = msg->fly_enemy_x;
    fly_point.y = msg->fly_enemy_y - 15;
    fly_point.z = msg->fly_enemy_z;
    // std::cout << "Fly Point: (" << fly_point.x << ", " << fly_point.y << ", " << fly_point.z << ")\n";
}
}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Detect)
