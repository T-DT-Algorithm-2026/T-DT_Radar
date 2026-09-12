#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vision_interface/msg/fly_points.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <numeric>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include <chrono>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
namespace tdt_radar{
class Cluster : public rclcpp::Node
{
    public:
    Cluster(const rclcpp::NodeOptions& node_options);
    ~Cluster(){}
    
    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fly_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<vision_interface::msg::FlyPoints>::SharedPtr fly_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr fly_enemy_point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void fly_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped transform_stamped;
};
}//namespace tdt_radar
