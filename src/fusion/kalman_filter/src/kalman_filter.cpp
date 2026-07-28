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

    radar_detect_pub_ = this->create_publisher<vision_interface::msg::DetectResult>("/kalman_detect", 10);

    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&KalmanFilter::publish_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Kalman_filter_Node has been started.");
}

void KalmanFilter::match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    match_info = *msg;
}

void KalmanFilter::radio_callback(const radio_interface::msg::Position::SharedPtr msg)
{
    rclcpp::Time radio_time = this->now();
    const double radio_seconds = radio_time.nanoseconds() / 1e9;

    // Radio 数组只包含敌方目标，根据己方颜色映射到统一的 0~11 目标 ID。
    for(int i = 0; i < 6; i++)
    {
        const int target_id = match_info.self_color == 0 ? i + 6 : i;
        if((msg->x[i] == 0 && msg->y[i] == 0) || (msg->x[i] == 2800 && msg->y[i] == 1500))
        {
            radio_filters_[target_id].reset();
            cars_[target_id].radio.valid = false;
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
        if(filter && radio_seconds - filter->last_measurement_time > RadioTimeout)
        {
            filter.reset();
        }
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
    std::array<pcl::PointXY, 12> camera_points{};
    std::array<rclcpp::Time, 12> camera_times{};

    // 相机结果只用于给 Radar 轨迹匹配目标 ID，不参与位置滤波。
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
    rclcpp::Time time = msg->header.stamp;//获取聚类的时间戳
    for(int i = static_cast<int>(radar_filters.size()) - 1; i >= 0; i--)
    {
        if(radar_filters[i].should_delete(time))
        {
            radar_filters.erase(radar_filters.begin() + i);
        }
    }

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
        return;//没有点的处理
    }

    bool has_radar_filter = false;
    // 先将已有轨迹预测到当前雷达帧，用预测点进行后续关联。
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
        // 门控范围外的点直接建立新轨迹，范围内的点交给匈牙利算法分配。
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
            }
            else
            {
                radar_filters.emplace_back(cloud_xy->points[j], time);
            }
        }

        if(!candidate_points.empty())
        {
            // 无法匹配的组合使用极大代价，避免错误更新 Radar 轨迹。
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

        }//卡尔曼匹配

        for(auto &kf : radar_filters)
        {
            if(kf.has_updated == false)
            {
                // 本帧未匹配到测量时只接受先验状态，不进行虚假测量更新。
                kf.deal_missing(time);
            }
        }
    }

    // Radar 卡尔曼更新完成后，将最新的位置、速度和时间同步到车辆状态。
    for(auto &car : cars_)
    {
        car.radar.valid = false;
    }
    for(const auto &filter : radar_filters)
    {
        if(filter.target_id < 0 || filter.target_id >= 12 || !filter.has_measurement)
        {
            continue;
        }
        SourceState &radar = cars_[filter.target_id].radar;
        if(!radar.valid || radar.measurement_time < filter.last_measurement_time)
        {
            update_source_state(radar, filter);
        }
    }

}

void KalmanFilter::publish_timer_callback()
{
    const rclcpp::Time now = this->now();
    const double now_seconds = now.nanoseconds() / 1e9;

    // 定时器只处理车辆状态；传感器完全断流时也能根据时间将对应来源设为无效。
    for(auto &car : cars_)
    {
        if(car.radar.valid && now_seconds - car.radar.measurement_time > RadarTimeout)
        {
            car.radar.valid = false;
        }
        if(car.radio.valid && now_seconds - car.radio.measurement_time > RadioTimeout)
        {
            car.radio.valid = false;
        }
    }

    vision_interface::msg::DetectResult detect_msg;
    detect_msg.header.stamp = now;
    detect_msg.header.frame_id = "rm_frame";

    for(int target_id = 0; target_id < 12; target_id++)
    {
        const CarState &car = cars_[target_id];
        const SourceState *source = nullptr;
        // Radio 有效时优先使用，超时清除后立即回退到 Radar。
        if(car.radio.valid)
        {
            source = &car.radio;
        }
        else if(car.radar.valid)
        {
            source = &car.radar;
        }
        else
        {
            continue;
        }

        const double prediction_time = std::max(0.0, now_seconds - source->state_time) + PredictionHorizon;
        pcl::PointXY future_point;
        future_point.x = source->position.x + source->velocity.x * prediction_time;
        future_point.y = source->position.y + source->velocity.y * prediction_time;

        if(target_id < 6)
        {
            detect_msg.blue_x[target_id] = future_point.x;
            detect_msg.blue_y[target_id] = future_point.y;
        }
        else
        {
            const int red_id = target_id - 6;
            detect_msg.red_x[red_id] = future_point.x;
            detect_msg.red_y[red_id] = future_point.y;
        }
    }

    if(match_info.self_color == 0)
    {
        for(int i = 0; i < 6; i++)
        {
            if(detect_msg.blue_x[i] != 0 || detect_msg.blue_y[i] != 0)
            {
                detect_msg.blue_x[i] = 28 - detect_msg.blue_x[i];
                detect_msg.blue_y[i] = 15 - detect_msg.blue_y[i];
            }
            if(detect_msg.red_x[i] != 0 || detect_msg.red_y[i] != 0)
            {
                detect_msg.red_x[i] = 28 - detect_msg.red_x[i];
                detect_msg.red_y[i] = 15 - detect_msg.red_y[i];
            }
        }
    }
    radar_detect_pub_->publish(detect_msg);
}

void KalmanFilter::update_source_state(SourceState &source, const Kalman_filter_plus &filter)
{
    filter.get_motion_state(source.position, source.velocity);
    source.state_time = filter.state_time;
    source.measurement_time = filter.last_measurement_time;
    source.valid = true;
}

}//namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::KalmanFilter)
