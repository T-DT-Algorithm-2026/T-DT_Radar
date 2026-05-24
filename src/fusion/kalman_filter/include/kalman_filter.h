#pragma once

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
// #include "filter_plus.h"
#include "guess_point.h"
#include <rclcpp/publisher.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <radio_interface/msg/position.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace tdt_radar{
class KalmanFilter :public rclcpp::Node
{
    public:
    KalmanFilter(const rclcpp::NodeOptions& node_options);
    ~KalmanFilter(){}
    
    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr sub_detect_;
    rclcpp::Subscription<vision_interface::msg::RadarWarn>::SharedPtr sub_lidar_;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr sub_match_;
    rclcpp::Subscription<radio_interface::msg::Position>::SharedPtr sub_radio_;
    rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr radar_pub_;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr radar_detect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void detect_callback(const vision_interface::msg::DetectResult::SharedPtr msg);
    void lidar_callback(const vision_interface::msg::RadarWarn::SharedPtr msg);
    void match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg);
    void radio_callback(const radio_interface::msg::Position::SharedPtr msg);
    std::vector<Kalman_filter_plus> KFs;
    car arr[12];
    vision_interface::msg::RadarWarn lidar_detect;
    vision_interface::msg::MatchInfo match_info;
};

std::vector<int> solve_hungarian(const Eigen::MatrixXd& cost_matrix) {
    int rows = cost_matrix.rows();
    int cols = cost_matrix.cols();
    Eigen::MatrixXd costs = cost_matrix;

    // 确保是方阵，如果不是，则用0补全
    if (rows > cols) {
        costs.conservativeResize(rows, rows);
        costs.rightCols(rows - cols).setZero();
    } else if (cols > rows) {
        costs.conservativeResize(cols, cols);
        costs.bottomRows(cols - rows).setZero();
    }
    int n = costs.rows();
    
    // Munkres算法步骤
    // 步骤1: 每行减去该行的最小值
    for (int i = 0; i < n; ++i) {
        double min_val = costs.row(i).minCoeff();
        costs.row(i).array() -= min_val;
    }

    // 步骤2: 每列减去该列的最小值
    for (int j = 0; j < n; ++j) {
        double min_val = costs.col(j).minCoeff();
        costs.col(j).array() -= min_val;
    }
    
    std::vector<int> row_sol(n, -1);
    std::vector<int> col_sol(n, -1);
    std::vector<int> parent_row(n, -1);
    std::vector<int> unassigned_rows;

    auto find_path = [&](int start_row) {
        std::vector<bool> row_visited(n, false);
        std::vector<bool> col_visited(n, false);
        std::vector<int> q;
        q.push_back(start_row);
        row_visited[start_row] = true;
        parent_row.assign(n, -1);

        int head = 0;
        while(head < q.size()){
            int u = q[head++];
            for(int v=0; v<n; ++v){
                if(costs(u,v) == 0 && !col_visited[v]){
                    col_visited[v] = true;
                    if(col_sol[v] < 0){
                        int current = v;
                        int prev_row = u;
                        while(prev_row != -1){
                            int prev_col = current;
                            current = row_sol[prev_row];
                            row_sol[prev_row] = prev_col;
                            col_sol[prev_col] = prev_row;
                            prev_row = parent_row[prev_row];
                        }
                        return true;
                    }
                    int next_row = col_sol[v];
                    if(!row_visited[next_row]){
                        row_visited[next_row] = true;
                        q.push_back(next_row);
                        parent_row[next_row] = u;
                    }
                }
            }
        }
        return false;
    };

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (costs(i, j) == 0 && row_sol[i] == -1 && col_sol[j] == -1) {
                row_sol[i] = j;
                col_sol[j] = i;
                break;
            }
        }
    }
    
    for(int i=0; i<n; ++i){
        if(row_sol[i] == -1){
            if(find_path(i)) continue;
        }
    }
    
    // 最终的分配结果
    std::vector<int> result(rows, -1);
    for (int i = 0; i < rows; ++i) {
        if (row_sol[i] < cols) {
            result[i] = row_sol[i];
        }
    }

    return result;
}//匈牙利算法
}//namespace tdt_radar
