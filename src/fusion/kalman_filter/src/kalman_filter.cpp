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
    sub_match_ = this->create_subscription<vision_interface::msg::MatchInfo>("/match_info", 10, std::bind(&KalmanFilter::match_callback, this, std::placeholders::_1));
    sub_radio_ = this->create_subscription<radio_interface::msg::Position>("robot_position", 10, std::bind(&KalmanFilter::radio_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Kalman_filter_Node has been started.");
}

void KalmanFilter::match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    this->match_info = *msg;
}//裁判系统的消息

void KalmanFilter::radio_callback(const radio_interface::msg::Position::SharedPtr msg)
{
    std::array<pcl::PointXY, 6> radio_points{};
    for(int i = 0; i < 6; i++)
    {
        radio_points[i].x = static_cast<float>(msg->x[i]) / 100.0f;
        radio_points[i].y = static_cast<float>(msg->y[i]) / 100.0f;
        if(match_info.self_color == 0)
        {
            radio_points[i].x = 28.0f - radio_points[i].x;
            radio_points[i].y = 15.0f - radio_points[i].y;
        }
    }
    rclcpp::Time radio_time = this->now();
    for(int i = 0; i < 6; i++)
    {
        const int target_id = match_info.self_color == 0 ? i + 6 : i;
        if((msg->x[i] == 0 && msg->y[i] == 0) || (msg->x[i] == 28 && msg->y[i] == 15) )
        {
            radio_points[i] = pcl::PointXY{0, 0};
            for(int kf_index = static_cast<int>(KFs.size()) - 1; kf_index >= 0; kf_index--)
            {
                if(KFs[kf_index].is_radio_filter && KFs[kf_index].target_id == target_id)
                {
                    KFs.erase(KFs.begin() + kf_index);
                }
            }
            continue;
        }

        // 该目标有 Radio 时只保留 Radio Kalman，不再使用原来的雷达轨迹。
        for(int kf_index = static_cast<int>(KFs.size()) - 1; kf_index >= 0; kf_index--)
        {
            if(!KFs[kf_index].is_radio_filter && KFs[kf_index].target_id == target_id)
            {
                KFs.erase(KFs.begin() + kf_index);
            }
        }

        bool updated = false;
        for(auto &kf : KFs)
        {
            if(kf.is_radio_filter && kf.target_id == target_id)
            {
                kf.update_radio(radio_time, radio_points[i]);
                updated = true;
                break;
            }
        }
        if(!updated)
        {
            KFs.emplace_back(radio_points[i], radio_time);
            KFs.back().target_id = target_id;
            KFs.back().id_history.push_back(target_id);
            KFs.back().last_radio_match_time = Kalman_filter_plus::GetTimeByRosTime(radio_time);
            KFs.back().has_radio_match = true;
            KFs.back().is_radio_filter = true;
        }
    }
}

void KalmanFilter::detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg)//？获取点位信息？
{
    std::array<pcl::PointXY, 12> camera_points{};
    std::array<rclcpp::Time, 12> camera_times{};

    for(int i = 0; i < 6; i++)
    {
        if(msg->blue_x[i] != 0 || msg->blue_y[i] != 0)
        {
            camera_points[i].x = msg->blue_x[i];
            camera_points[i].y = msg->blue_y[i];
            camera_times[i] = msg->header.stamp;
        }

        if(msg->red_x[i] != 0 || msg->red_y[i] != 0)
        {
            camera_points[i + 6].x = msg->red_x[i];
            camera_points[i + 6].y = msg->red_y[i];
            camera_times[i + 6] = msg->header.stamp;
        }
    }

    for(auto &kf : KFs)
    {
        for(int target_id = 0; target_id < 12; target_id++)
        {
            bool has_radio_filter = false;
            for(const auto &candidate_kf : KFs)
            {
                if(candidate_kf.is_radio_filter && candidate_kf.target_id == target_id)
                {
                    has_radio_filter = true;
                    break;
                }
            }
            if(has_radio_filter)
            {
                continue;
            }
            kf.camera_catch(camera_times[target_id], camera_points[target_id], target_id);
        }
    }

}


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
    }//三维传二维
    if(cloud_xy->points.size() == 0)
    {
        for(int i = KFs.size() - 1; i >= 0; i--)
        {
            if(KFs[i].should_delete(time))
            {
                KFs.erase(KFs.begin() + i);
            }
        }
        publish_car_results(time);
        return;//没有点的处理
    }
    bool has_radar_filter = false;
    for(auto &kf : KFs)
    {
        if(kf.is_radio_filter)
        {
            continue;
        }
        kf.update_predict_point();//先验//提供预测点，无测量点//雷达点
        kf.has_updated = false;
        has_radar_filter = true;
    }
    //对于每个点
    //如果遍历所有卡尔曼都没找到能够匹配的，新建一个卡尔曼
    //若找到了1个，则更新这个卡尔曼
    //若找到了多个，则更新距离最近的那个
    if(!has_radar_filter)
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
                if(KFs[i].is_radio_filter)
                {
                    continue;
                }
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
            std::vector<size_t> radar_kf_indices;
            for(size_t i = 0; i < KFs.size(); i++)
            {
                if(!KFs[i].is_radio_filter)
                {
                    radar_kf_indices.push_back(i);
                }
            }
            size_t num_kfs = radar_kf_indices.size();
            size_t num_candidates = candidate_points.size();
            const double max_cost = 1e9;

            Eigen::MatrixXd cost_matrix(num_kfs, num_candidates);

            for (size_t i = 0; i < num_kfs; ++i) 
            {
                for (size_t j = 0; j < num_candidates; ++j) 
                {
                    // 成本矩阵现在是 KFs 和 candidate_points 之间的
                    Kalman_filter_plus &kf = KFs[radar_kf_indices[i]];
                    if(kf.match(candidate_points[j]))
                    {
                        cost_matrix(i, j) = kf.Distance(kf.predict_point, candidate_points[j]);
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
                    KFs[radar_kf_indices[i]].deal_catch(candidate_points[candidate_idx], time);
                }
            }

            for(auto &kf : KFs)
            {
                if(!kf.is_radio_filter && kf.has_updated == false)
                {
                    kf.deal_missing(time);
                }
            }
        }//卡尔曼匹配
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = KFs.size() - 1; i >= 0; i--)
    {
        if (KFs[i].should_delete(time))
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
            if(KFs[i].target_id >= 0 && KFs[i].target_id < 6)
            {
                point.b = 255;
            }
            else if(KFs[i].target_id >= 6 && KFs[i].target_id < 12)
            {
                point.r = 255;
            }
            else
            {
                point.r = KFs[i].display_color[0];
                point.g = KFs[i].display_color[1];
                point.b = KFs[i].display_color[2];
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
    publish_car_results(time);
}

void KalmanFilter::publish_car_results(const rclcpp::Time &stamp)
{
    std::array<const Kalman_filter_plus *, 12> selected_kfs{};
    for(const auto &kf : KFs)
    {
        if(kf.target_id < 0 || kf.target_id >= 12)
        {
            continue;
        }

        const Kalman_filter_plus *selected_kf = selected_kfs[kf.target_id];
        const bool radio_has_priority = selected_kf != nullptr && kf.is_radio_filter && !selected_kf->is_radio_filter;
        const bool is_newer_same_source = selected_kf != nullptr && kf.is_radio_filter == selected_kf->is_radio_filter &&
                                          kf.last_radar_catch_time > selected_kf->last_radar_catch_time;
        if(selected_kf == nullptr || radio_has_priority || is_newer_same_source)
        {
            selected_kfs[kf.target_id] = &kf;
        }
    }

    vision_interface::msg::DetectResult detect_msg;
    detect_msg.header.stamp = stamp;
    detect_msg.header.frame_id = "rm_frame";
    for(int target_id = 0; target_id < 12; target_id++)
    {
        const Kalman_filter_plus *selected_kf = selected_kfs[target_id];
        if(selected_kf == nullptr)
        {
            continue;
        }

        const pcl::PointXY &send_point = selected_kf->predict_point;
        if(send_point.x == 0 && send_point.y == 0)
        {
            continue;
        }

        if(target_id < 6)//蓝色
        {
            detect_msg.blue_x[target_id] = send_point.x;
            detect_msg.blue_y[target_id] = send_point.y;
            detect_msg.blue_from_radio[target_id] = selected_kf->is_radio_filter;
        }
        else//红色
        {
            const int red_id = target_id - 6;
            detect_msg.red_x[red_id] = send_point.x;
            detect_msg.red_y[red_id] = send_point.y;
            detect_msg.red_from_radio[red_id] = selected_kf->is_radio_filter;
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
    for(int i=0;i<6;i++)
    {
        if(match_info.self_color==0)
        {
            radar_msg.radar_enemy_x[i]=detect_msg.red_x[i];
            radar_msg.radar_enemy_y[i]=detect_msg.red_y[i];
            radar_msg.radar_ally_x[i]=detect_msg.blue_x[i];
            radar_msg.radar_ally_y[i]=detect_msg.blue_y[i];
        }
        else if(match_info.self_color==2)
        {
            radar_msg.radar_enemy_x[i]=detect_msg.blue_x[i];
            radar_msg.radar_enemy_y[i]=detect_msg.blue_y[i];
            radar_msg.radar_ally_x[i]=detect_msg.red_x[i];
            radar_msg.radar_ally_y[i]=detect_msg.red_y[i];
        }
    }
    radar_pub_->publish(radar_msg);///radar2sentry
}
}//namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::KalmanFilter)
