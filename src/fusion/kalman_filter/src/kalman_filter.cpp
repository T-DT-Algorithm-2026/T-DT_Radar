#include "kalman_filter.h"
#include "filter_plus.h"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>

namespace tdt_radar{

KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options):rclcpp::Node("kalman_filter_node",node_options)
{
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10, std::bind(&KalmanFilter::callback, this, std::placeholders::_1));
    sub_detect_ = this->create_subscription<vision_interface::msg::DetectResult>("/resolve_result", rclcpp::SensorDataQoS(), std::bind(&KalmanFilter::detect_callback, this, std::placeholders::_1));
    sub_match_ = this->create_subscription<vision_interface::msg::MatchInfo>("/match_info", 10, std::bind(&KalmanFilter::match_callback, this, std::placeholders::_1));
    sub_radio_ = this->create_subscription<radio_interface::msg::Position>("/robot_position", 10, std::bind(&KalmanFilter::radio_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_kalman", 10);
    measurement_pub_ = this->create_publisher<vision_interface::msg::DetectResult>("/kalman_measurement", 10);
    radar_pub_ = this->create_publisher<vision_interface::msg::Radar2Sentry>("/radar2sentry", 10);
    radar_detect_pub_ = this->create_publisher<vision_interface::msg::DetectResult>("/kalman_detect", 10);

    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&KalmanFilter::publish_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Kalman_filter_Node has been started.");
}

void KalmanFilter::match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    match_info = *msg;
}

void KalmanFilter::radio_callback(const radio_interface::msg::Position::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    rclcpp::Time radio_time = this->now();

    for(int i = 0; i < 6; i++)
    {
        const int target_id = match_info.self_color == 0 ? i + 6 : i;
        if((msg->x[i] == 0 && msg->y[i] == 0) || (msg->x[i] == 28 && msg->y[i] == 15))
        {
            radio_filters_[target_id].reset();
            cars_[target_id].radio = SourceState{};
            continue;
        }

        pcl::PointXY radio_point;
        radio_point.x = static_cast<float>(msg->x[i]) / 100.0f;
        radio_point.y = static_cast<float>(msg->y[i]) / 100.0f;
        if(match_info.self_color == 0)
        {
            radio_point.x = 28.0f - radio_point.x;
            radio_point.y = 15.0f - radio_point.y;
        }

        auto &filter = radio_filters_[target_id];
        if(filter)
        {
            filter->update_radio(radio_time, radio_point);
        }
        else
        {
            filter = std::make_unique<Kalman_filter_plus>(radio_point, radio_time);
            filter->target_id = target_id;
            filter->id_history.push_back(target_id);
        }
        update_source_state(cars_[target_id].radio, *filter);
    }
}

void KalmanFilter::detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
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

    for(auto &kf : radar_filters)
    {
        for(int target_id = 0; target_id < 12; target_id++)
        {
            kf.camera_catch(camera_times[target_id], camera_points[target_id], target_id);
        }
    }
}

void KalmanFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    rclcpp::Time time = msg->header.stamp;//获取聚类的时间戳
    auto now_time = std::chrono::steady_clock::now();//当前时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*msg, *cloud);//点云转换
    for(auto point : cloud->points)
    {
        pcl::PointXY point_xy;
        point_xy.x = point.x;
        point_xy.y = point.y;
        cloud_xy->points.push_back(point_xy);
    }//三维传二维
    if(cloud_xy->points.size() == 0)
    {
        for(int i = radar_filters.size() - 1; i >= 0; i--)
        {
            if(radar_filters[i].should_delete(time))
            {
                radar_filters.erase(radar_filters.begin() + i);
            }
        }
        return;//没有点的处理
    }

    bool has_radar_filter = false;
    for(auto &kf : radar_filters)
    {
        kf.update_predict_point();//先验//提供预测点，无测量点//雷达点
        kf.has_updated = false;
        has_radar_filter = true;
    }

    if(!has_radar_filter)
    {
        for(const auto &point : cloud_xy->points)
        {
            radar_filters.emplace_back(point, time);//传递信息
        }
    }
    else
    {
        std::vector<pcl::PointXY> candidate_points;
        std::vector<bool> point_is_candidate(cloud_xy->points.size(), false);
        for(size_t j = 0; j < cloud_xy->points.size(); ++j)
        {
            bool has_at_least_one_match = false;
            for(size_t i = 0; i < radar_filters.size(); ++i)
            {
                if(radar_filters[i].match(cloud_xy->points[j]))
                {
                    has_at_least_one_match = true;
                    break;
                }
            }

            if(has_at_least_one_match)
            {
                candidate_points.push_back(cloud_xy->points[j]);
                point_is_candidate[j] = true;
            }
            else
            {
                radar_filters.emplace_back(cloud_xy->points[j], time);
            }
        }

        if(!candidate_points.empty())
        {
            size_t num_kfs = radar_filters.size();
            size_t num_candidates = candidate_points.size();
            const double max_cost = 1e9;
            Eigen::MatrixXd cost_matrix(num_kfs, num_candidates);

            for(size_t i = 0; i < num_kfs; ++i)
            {
                for(size_t j = 0; j < num_candidates; ++j)
                {
                    Kalman_filter_plus &kf = radar_filters[i];
                    if(kf.match(candidate_points[j]))
                    {
                        cost_matrix(i, j) = kf.Distance(kf.predict_point, candidate_points[j]);
                    }
                    else
                    {
                        cost_matrix(i, j) = max_cost;
                    }
                }
            }

            std::vector<int> assignments = solve_hungarian(cost_matrix);
            for(size_t i = 0; i < assignments.size(); ++i)
            {
                int candidate_idx = assignments[i];
                if(candidate_idx != -1 && cost_matrix(i, candidate_idx) < max_cost)
                {
                    radar_filters[i].deal_catch(candidate_points[candidate_idx], time);
                }
            }

            for(auto &kf : radar_filters)
            {
                if(kf.has_updated == false)
                {
                    kf.deal_missing(time);
                }
            }
        }//卡尔曼匹配
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = radar_filters.size() - 1; i >= 0; i--)
    {
        if(radar_filters[i].should_delete(time))
        {
            radar_filters.erase(radar_filters.begin() + i);
        }
        else
        {
            pcl::PointXYZRGB point;
            point.x = radar_filters[i].predict_point.x;
            point.y = radar_filters[i].predict_point.y;
            point.z = 1.5;
            if(radar_filters[i].target_id >= 0 && radar_filters[i].target_id < 6)
            {
                point.b = 255;
            }
            else if(radar_filters[i].target_id >= 6 && radar_filters[i].target_id < 12)
            {
                point.r = 255;
            }
            else
            {
                point.r = radar_filters[i].display_color[0];
                point.g = radar_filters[i].display_color[1];
                point.b = radar_filters[i].display_color[2];
            }
            cloud_filtered->points.push_back(point);
        }
    }

    cloud_filtered->header.frame_id = "rm_frame";
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);///livox/lidar_kalman//制作测试作用
    auto end_time = std::chrono::steady_clock::now();
    float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - now_time).count();
}

void KalmanFilter::publish_timer_callback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    const rclcpp::Time now = this->now();
    cleanup_stale_tracks(now);
    refresh_radar_states();
    publish_car_results(now);
}

void KalmanFilter::cleanup_stale_tracks(const rclcpp::Time &now)
{
    const double now_seconds = now.nanoseconds() / 1e9;
    for(int filter_index = static_cast<int>(radar_filters.size()) - 1; filter_index >= 0; filter_index--)
    {
        if(now_seconds - radar_filters[filter_index].last_measurement_time > RadarTimeout)
        {
            radar_filters.erase(radar_filters.begin() + filter_index);
        }
    }

    for(int target_id = 0; target_id < 12; target_id++)
    {
        auto &filter = radio_filters_[target_id];
        if(filter && now_seconds - filter->last_measurement_time > RadioTimeout)
        {
            filter.reset();
            cars_[target_id].radio = SourceState{};
        }
    }
}

void KalmanFilter::refresh_radar_states()
{
    for(auto &car : cars_)
    {
        car.radar = SourceState{};
    }

    for(const auto &filter : radar_filters)
    {
        if(filter.target_id < 0 || filter.target_id >= 12 || !filter.has_measurement)
        {
            continue;
        }

        SourceState &radar = cars_[filter.target_id].radar;
        if(radar.valid && radar.update_time >= filter.last_measurement_time)
        {
            continue;
        }
        update_source_state(radar, filter);
    }
}

void KalmanFilter::update_source_state(SourceState &source, const Kalman_filter_plus &filter)
{
    source.raw_point = filter.last_measurement_point;
    source.filtered_point = filter.predict_point;
    source.update_time = filter.last_measurement_time;
    source.valid = true;
}

void KalmanFilter::mirror_positions(vision_interface::msg::DetectResult &msg) const
{
    for(int i = 0; i < 6; i++)
    {
        if(msg.blue_x[i] != 0 || msg.blue_y[i] != 0)
        {
            msg.blue_x[i] = 28 - msg.blue_x[i];
            msg.blue_y[i] = 15 - msg.blue_y[i];
        }
        if(msg.red_x[i] != 0 || msg.red_y[i] != 0)
        {
            msg.red_x[i] = 28 - msg.red_x[i];
            msg.red_y[i] = 15 - msg.red_y[i];
        }
    }
}

void KalmanFilter::publish_car_results(const rclcpp::Time &stamp)
{
    vision_interface::msg::DetectResult measurement_msg;
    measurement_msg.header.stamp = stamp;
    measurement_msg.header.frame_id = "rm_frame";
    vision_interface::msg::DetectResult detect_msg;
    detect_msg.header.stamp = stamp;
    detect_msg.header.frame_id = "rm_frame";

    for(int target_id = 0; target_id < 12; target_id++)
    {
        const CarState &car = cars_[target_id];
        const SourceState *source = nullptr;
        bool from_radio = false;
        if(car.radio.valid)
        {
            source = &car.radio;
            from_radio = true;
        }
        else if(car.radar.valid)
        {
            source = &car.radar;
        }
        else
        {
            continue;
        }

        if(target_id < 6)
        {
            measurement_msg.blue_x[target_id] = source->raw_point.x;
            measurement_msg.blue_y[target_id] = source->raw_point.y;
            measurement_msg.blue_from_radio[target_id] = from_radio;
            detect_msg.blue_x[target_id] = source->filtered_point.x;
            detect_msg.blue_y[target_id] = source->filtered_point.y;
            detect_msg.blue_from_radio[target_id] = from_radio;
        }
        else
        {
            const int red_id = target_id - 6;
            measurement_msg.red_x[red_id] = source->raw_point.x;
            measurement_msg.red_y[red_id] = source->raw_point.y;
            measurement_msg.red_from_radio[red_id] = from_radio;
            detect_msg.red_x[red_id] = source->filtered_point.x;
            detect_msg.red_y[red_id] = source->filtered_point.y;
            detect_msg.red_from_radio[red_id] = from_radio;
        }
    }

    if(match_info.self_color == 0)
    {
        mirror_positions(measurement_msg);
        mirror_positions(detect_msg);
    }
    measurement_pub_->publish(measurement_msg);
    radar_detect_pub_->publish(detect_msg);

    vision_interface::msg::Radar2Sentry radar_msg;
    for(int i = 0; i < 6; i++)
    {
        if(match_info.self_color == 0)
        {
            radar_msg.radar_enemy_x[i] = detect_msg.red_x[i];
            radar_msg.radar_enemy_y[i] = detect_msg.red_y[i];
            radar_msg.radar_ally_x[i] = detect_msg.blue_x[i];
            radar_msg.radar_ally_y[i] = detect_msg.blue_y[i];
        }
        else if(match_info.self_color == 2)
        {
            radar_msg.radar_enemy_x[i] = detect_msg.blue_x[i];
            radar_msg.radar_enemy_y[i] = detect_msg.blue_y[i];
            radar_msg.radar_ally_x[i] = detect_msg.red_x[i];
            radar_msg.radar_ally_y[i] = detect_msg.red_y[i];
        }
    }
    radar_pub_->publish(radar_msg);
}

}//namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::KalmanFilter)
