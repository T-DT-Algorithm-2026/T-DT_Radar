#include "kalman_filter.h"
#include "filter_plus.h"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>

namespace tdt_radar{

KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options):rclcpp::Node("kalman_filter_node",node_options)
{
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10, std::bind(&KalmanFilter::callback, this, std::placeholders::_1));//聚类结果
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kalman", 10);
    radar_pub_ = this->create_publisher<vision_interface::msg::Radar2Sentry>("/radar2sentry", 10);
    radar_detect_pub_ = this->create_publisher<vision_interface::msg::DetectResult>("/kalman_detect", 10);
    sub_detect_= this->create_subscription<vision_interface::msg::DetectResult>("/resolve_result", rclcpp::SensorDataQoS(), std::bind(&KalmanFilter::detect_callback, this, std::placeholders::_1));
    sub_lidar_ = this->create_subscription<vision_interface::msg::RadarWarn>("/lidar_detect", 10, std::bind(&KalmanFilter::lidar_callback, this, std::placeholders::_1));
    sub_match_ = this->create_subscription<vision_interface::msg::MatchInfo>("/match_info", 10, std::bind(&KalmanFilter::match_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Kalman_filter_Node has been started.");
}

void KalmanFilter::match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    this->match_info = *msg;
    RCLCPP_INFO(this->get_logger(), "Match_info_callback");
}//裁判系统的消息

void KalmanFilter::detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg)//？获取点位信息？
{
    // RCLCPP_INFO(this->get_logger(), "Detect_callback");
    rclcpp::Time time = msg->header.stamp;
    // std::cout<<msg->blue_x[0]<<","<<msg->blue_y[0]<<std::endl;
    for(int i=0;i<6;i++)
    {
        pcl::PointXY red_point;
        red_point.x = msg->red_x[i];
        red_point.y = msg->red_y[i];//获取座标点
        if(red_point.x != 0 || red_point.y != 0)
        {
            for(auto &kf : KFs)
            {
                // int number = i+1;
                // if (number == 6)number++;
                kf.camera_match(time, red_point, 2, i);//相机雷达匹配
            }
        }
        pcl::PointXY blue_point;
        blue_point.x = msg->blue_x[i];
        blue_point.y = msg->blue_y[i];
        if(blue_point.x != 0 || blue_point.y != 0)
        {
            for(auto &kf : KFs)
            {
                // int number = i+1;
                // if (number == 6)number++;
                kf.camera_match(time, blue_point, 0, i);//雷达与相机匹配
            }
        }
    }
    // for(auto &kf : KFs)
    // {
    //     for(int i=kf.history.size() - 1; i >= 0; i--)
    //     {
    //         if(Kalman_filter_plus::GetTimeByRosTime(time)-kf.history[i].first > 5)
    //         {
    //             kf.history.erase(kf.history.begin() + i);
    //         }
    //     }
    // }
}//获取相机的检测结果，雷达与相机进行匹配


void KalmanFilter::lidar_callback(const vision_interface::msg::RadarWarn::SharedPtr msg)
{
    this->lidar_detect = *msg;
    // RCLCPP_INFO(this->get_logger(), "Lidar_detect_callback");
}//获取预警信息


void KalmanFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    rclcpp::Time time = msg->header.stamp;//获取聚类的时间戳
    auto now_time = std::chrono::steady_clock::now();//当前时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*msg, *cloud);//点云转换
    // pcl::PointXY point_fly;
    for(auto point : cloud->points) 
    {
        // if(point.z>10)
        // {
        //     point_fly.x = point.x;
        //     point_fly.y = point.y;
        //     break;
        // }
        pcl::PointXY point_xy;
        point_xy.x = point.x;
        point_xy.y = point.y;
        // std::cout<<point.z<<std::endl;
        cloud_xy->points.push_back(point_xy);
    }//三维传二维？
    if(cloud_xy->points.size() == 0)
        return;//没有点的处理
    for(auto &kf : KFs) 
    {
        kf.update_predict_point();//先验//提供预测点，无测量点//雷达点
        kf.has_updated = false;
    }
    //对于每个点
    //如果遍历所有卡尔曼都没找到能够匹配的，新建一个卡尔曼
    //若找到了1个，则更新这个卡尔曼
    //若找到了多个，则更新距离最近的那个
    if (KFs.empty()) 
    {

        // 如果当前没有跟踪器，则所有检测点都是新目标
        for (const auto& point : cloud_xy->points) 
        {
            KFs.emplace_back(point, time);//传递信息
        }
    } 
    else 
    {
        // --- 阶段一：分离“全新点”和“候选点” ---
        
        std::vector<pcl::PointXY> candidate_points; // 至少有一个KF能匹配上的点
        std::vector<bool> point_is_candidate(cloud_xy->points.size(), false); // 标记每个点是否为候选点
        for (size_t j = 0; j < cloud_xy->points.size(); ++j) 
        {
            bool has_at_least_one_match = false;
            for (size_t i = 0; i < KFs.size(); ++i) 
            {
                // 检查该点是否在任何一个KF的匹配门控范围内
                if (KFs[i].match(cloud_xy->points[j])) 
                {
                    has_at_least_one_match = true;
                    break; // 只要有一个匹配，就无需再检查其他KF
                }
            }

            if (has_at_least_one_match) 
            {
                // 如果至少有一个KF能匹配，则将其视为“候选点”，等待匈牙利分配
                candidate_points.push_back(cloud_xy->points[j]);
                point_is_candidate[j] = true;
            } 
            else 
            {
                // 如果没有任何一个KF能匹配，则判定为“全新点”，立即创建新滤波器
                KFs.emplace_back(cloud_xy->points[j], time);
            }
        }

        // --- 阶段二：仅对“候选点”进行匈牙利匹配 ---
        
        if (!candidate_points.empty()) 
        {
            size_t num_kfs = KFs.size();
            size_t num_candidates = candidate_points.size();
            const double max_cost = 1e9;

            Eigen::MatrixXd cost_matrix(num_kfs, num_candidates);

            for (size_t i = 0; i < num_kfs; ++i) 
            {
                for (size_t j = 0; j < num_candidates; ++j) 
                {
                    // 成本矩阵现在是 KFs 和 candidate_points 之间的
                    if (KFs[i].match(candidate_points[j])) 
                    {
                        cost_matrix(i, j) = KFs[i].Distance(KFs[i].predict_point, candidate_points[j]);
                    } 
                    else 
                    {
                        // 按理说这里不会执行，因为候选点都至少有一个匹配项，但作为安全措施保留
                        cost_matrix(i, j) = max_cost;
                    }
                }
            }

            // 运行匈牙利算法
            std::vector<int> assignments = solve_hungarian(cost_matrix);

            // 处理匹配结果
            for (size_t i = 0; i < assignments.size(); ++i) 
            {
                int candidate_idx = assignments[i];
                
                if (candidate_idx != -1 && cost_matrix(i, candidate_idx) < max_cost) 
                {
                    KFs[i].deal_catch(candidate_points[candidate_idx], time);
                }
            }

            for(auto &kf : KFs)
            {
                if(kf.has_updated == false) 
                {
                    kf.is_space = true;
                    kf.deal_missing(time);
                }
                if(kf.now_color==0)
                {
                    kf.new_id= kf.now_number;
                }
                else if(kf.now_color==2)
                {
                    kf.new_id= kf.now_number+6;
                }
                else if(kf.now_color==-1)
                {
                    kf.new_id= kf.new_id;//保持不变
                }
                // kf.test();
            }//为赋值给car作准备
            for (auto &kf : KFs) 
            {
                if(kf.new_id>=0&&kf.new_id<12)
                {
                    arr[kf.new_id].getcar(&kf);
                } 
            }
            for(auto &car: arr)
            {
                car.deal_car();
                // car.test();
            }
        }//卡尔曼匹配
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i = KFs.size() - 1; i >= 0; i--) 
        {
            if ((KFs[i].miss_last_time) > 1.5) 
            {
                KFs.erase(KFs.begin() + i);
                // std::cout<<"delete kf"<<std::endl;
            } 
            else 
            {
                // if(KFs[i].has_updated)
                // {
                pcl::PointXYZRGB point;
                point.x = KFs[i].predict_point.x;
                point.y = KFs[i].predict_point.y;
                point.z = 1.5;//？这是什么依据？
                // int color = KFs[i].get_color();
                int color = KFs[i].now_color;
                switch (color) {
                    case 0:
                        point.b = 255;
                        break;

                    case 2:
                        point.r = 255;
                        break;
                    
                    default:
                        point.r = KFs[i].color[0];
                        point.g = KFs[i].color[1];
                        point.b = KFs[i].color[2];
                        break;
                }
                cloud_filtered->points.push_back(point);
                // }
            }//赋予颜色
        }

        cloud_filtered->header.frame_id = "rm_frame";
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = "rm_frame";
        output.header.stamp = msg->header.stamp;
        pub_->publish(output);///livox/lidar_kalman//制作测试作用
        auto end_time = std::chrono::steady_clock::now();
        float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
        // RCLCPP_INFO(this->get_logger(), "Kalman Callback time is %f ms", dur_time);
        vision_interface::msg::DetectResult detect_msg;
        for(auto car: arr)//从前到后
        {
            if(car.send_point.x==0&&car.send_point.y==0)continue;
            if(car.color == 0)//蓝色
            {
                int number= car.number;
                detect_msg.blue_x[number] = car.send_point.x;
                detect_msg.blue_y[number] = car.send_point.y;
            }
            if(car.color == 2)//红色
            {
                int number= car. number;
                detect_msg.red_x[number] = car.send_point.x;
                detect_msg.red_y[number] = car.send_point.y;
            }
        }
        // if(match_info.self_color==0)
        // {
        //     detect_msg.blue_x[4]=point_fly.x;
        //     detect_msg.blue_y[4]=point_fly.y;
        // }
        // if(match_info.self_color==2)
        // {
        //     detect_msg.red_x[4]=point_fly.x;
        //     detect_msg.red_y[4]=point_fly.y;
        // }
        if(match_info.self_color==0)
        {
            for(int i=0;i<6;i++)
            {
                if(detect_msg.blue_x[i]!=0&&detect_msg.blue_y[i]!=0)
                {
                    detect_msg.blue_x[i]=28-detect_msg.blue_x[i];
                    detect_msg.blue_y[i]=15-detect_msg.blue_y[i];
                }
                if(detect_msg.red_x[i]!=0&&detect_msg.red_y[i]!=0)
                {
                    detect_msg.red_x[i]=28-detect_msg.red_x[i];
                    detect_msg.red_y[i]=15-detect_msg.red_y[i];
                }
            }
        }//优先发布新匹配的点
        radar_detect_pub_->publish(detect_msg);///kalman_detect

        vision_interface::msg::Radar2Sentry radar_msg;
        if(match_info.self_color==0)
        {
            for(int i=0;i<6;i++)
            {
                radar_msg.radar_enemy_x[i]=detect_msg.red_x[i];
                radar_msg.radar_enemy_y[i]=detect_msg.red_y[i];
            }
        }    
        radar_pub_->publish(radar_msg);///radar2sentry
    }
}
}//namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::KalmanFilter)

