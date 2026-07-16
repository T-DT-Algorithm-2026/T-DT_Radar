#include "cluster.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <rclcpp/duration.hpp>
#include <pcl/segmentation/extract_clusters.h>
namespace tdt_radar{

    Cluster::Cluster(const rclcpp::NodeOptions& node_options): Node("cluster_node", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "cluster_node start");

        // try 
        // {
        //     // 获取点云采集时刻，雷达(livox_frame)相对于地图(rm_frame)的坐标
        //     transform_stamped = tf_buffer_.lookupTransform("rm_frame", "livox_frame", this->now() - rclcpp::Duration::from_seconds(0.1));
        // }
        // catch (tf2::TransformException &ex) 
        // {
        //     RCLCPP_ERROR(this->get_logger(), "TF 转换失败: %s", ex.what());
        //     return;
        // }

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10, std::bind(&Cluster::callback, this, std::placeholders::_1));
        fly_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_fly", 10, std::bind(&Cluster::fly_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);
        fly_pub_ = this->create_publisher<vision_interface::msg::FlyPoints>("/livox/lidar_fly_cluster", 10);
        fly_enemy_point_pub_ = this->create_publisher<geometry_msgs::msg::Point32>("/livox/lidar_fly_point", 10);
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Cluster::timer_callback, this));

    }


void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();//记录时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);//点云转换
    if (cloud->empty()) {return;}
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    auto time = std::chrono::system_clock::now();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.25);
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);//欧几里德聚类
    // std::cout<<(std::chrono::system_clock::now()-time).count()<<"ms"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> *out_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;//根据索引得到一个聚类后的点云

        pcl::PointXYZ move_point;
        for(auto point:cloud_cluster->points)
        {
            move_point.x += point.x;
            move_point.y += point.y;
            move_point.z += point.z;
        }
        move_point.x /= cloud_cluster->points.size();
        move_point.y /= cloud_cluster->points.size();
        move_point.z /= cloud_cluster->points.size();
        out_cloud->points.push_back(move_point);//取质心        
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*out_cloud, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
//     std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//     RCLCPP_INFO(this->get_logger(), "Cluster callback time: %f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}


void Cluster::fly_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // std::cout<<"找到飞机"<<std::endl;
    try 
    {
        transform_stamped = tf_buffer_.lookupTransform("rm_frame", "livox_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(), "TF 转换失败: %s", ex.what());
        return;
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();//记录时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);//点云转换
    if (cloud->empty()) {return;}

    // 提取雷达在地图中的坐标
    double lidar_x = transform_stamped.transform.translation.x;
    double lidar_y = transform_stamped.transform.translation.y;
    double lidar_z = transform_stamped.transform.translation.z;
    // std::cout << "雷达在地图中的坐标: " << lidar_x <<" "<< lidar_y <<" "<< lidar_z <<std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.25);
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);//欧几里德聚类//TODO: 调参

    std::vector<pcl::PointXYZ> clouds;
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;//根据索引得到一个聚类后的点云

        pcl::PointXYZ move_point;
        for(auto point:cloud_cluster->points)
        {
            move_point.x += point.x;
            move_point.y += point.y;
            move_point.z += point.z;
        }
        move_point.x /= cloud_cluster->points.size();
        move_point.y /= cloud_cluster->points.size();
        move_point.z /= cloud_cluster->points.size();
        clouds.push_back(move_point);//取质心        
    }

    vision_interface::msg::FlyPoints fly_points_msg;
    // std::cout<<"飞机数量:"<<clouds.size()<<std::endl;

    if(clouds.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "未检测到飞机");
        return;
    }
    if(clouds.size() == 1)
    {
        if(clouds[0].y<7.5)
        {
            fly_points_msg.fly_enemy_x = clouds[0].x;
            fly_points_msg.fly_enemy_y = clouds[0].y;
            fly_points_msg.fly_enemy_z = clouds[0].z;
            fly_points_msg.fly_ally_x = 0.0;
            fly_points_msg.fly_ally_y = 0.0;
            fly_points_msg.fly_ally_z = 0.0;
             RCLCPP_INFO(this->get_logger(), "检测到一个敌方飞机");
        }
        else
        {
            fly_points_msg.fly_ally_x = clouds[0].x;
            fly_points_msg.fly_ally_y = clouds[0].y;
            fly_points_msg.fly_ally_z = clouds[0].z;
            fly_points_msg.fly_enemy_x = 0.0;
            fly_points_msg.fly_enemy_y = 0.0;
            fly_points_msg.fly_enemy_z = 0.0;
            RCLCPP_INFO(this->get_logger(), "检测到一个我方飞机");
        }
        // std::cout<<"敌方飞机坐标:("<<fly_points_msg.fly_enemy_x<<","<<fly_points_msg.fly_enemy_y<<","<<fly_points_msg.fly_enemy_z<<")"<<std::endl;
        // std::cout<<"我方飞机坐标:("<<fly_points_msg.fly_ally_x<<","<<fly_points_msg.fly_ally_y<<")"<<std::endl;
    }
    if(clouds.size() >= 2)
    {
        if(clouds[0].y>clouds[1].y)
        {
            fly_points_msg.fly_ally_x = clouds[0].x;
            fly_points_msg.fly_ally_y = clouds[0].y;
            fly_points_msg.fly_ally_z = clouds[0].z;
            fly_points_msg.fly_enemy_x = clouds[1].x;
            fly_points_msg.fly_enemy_y = clouds[1].y;
            fly_points_msg.fly_enemy_z = clouds[1].z;

        }
        else
        {
            fly_points_msg.fly_ally_x = clouds[1].x;
            fly_points_msg.fly_ally_y = clouds[1].y;
            fly_points_msg.fly_ally_z = clouds[1].z;
            fly_points_msg.fly_enemy_x = clouds[0].x;
            fly_points_msg.fly_enemy_y = clouds[0].y;
            fly_points_msg.fly_enemy_z = clouds[0].z;
        }   
        // std::cout<<"敌方飞机坐标:("<<fly_points_msg.fly_enemy_x<<","<<fly_points_msg.fly_enemy_y<<","<<fly_points_msg.fly_enemy_z<<")"<<std::endl;
        RCLCPP_INFO(this->get_logger(), "检测到多个飞机");
        // std::cout<<"我方飞机坐标:("<<fly_points_msg.fly_ally_x<<","<<fly_points_msg.fly_ally_y<<")"<<std::endl;
    }
    // std::cout<<"成功"<<std::endl;
    fly_pub_->publish(fly_points_msg);

    tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);
    
    tf2::Vector3 t(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);
    tf2::Transform transform_livox_to_rm(q, t);
    std::cout<<"雷达坐标:("<<t.x()<<","<<t.y()<<","<<t.z()<<")"<<std::endl;

    tf2::Vector3 enemy_in_rm(fly_points_msg.fly_enemy_x, fly_points_msg.fly_enemy_y, fly_points_msg.fly_enemy_z);
    tf2::Vector3 enemy_in_livox = transform_livox_to_rm.inverse() * enemy_in_rm;

    geometry_msgs::msg::Point32 enemy_point;
    enemy_point.x = enemy_in_livox.x();
    enemy_point.y = enemy_in_livox.y();
    enemy_point.z = fly_points_msg.fly_enemy_z - t.z();
    std::cout<<enemy_point.x<<","<<enemy_point.y<<","<<enemy_point.z<<std::endl;
    fly_enemy_point_pub_->publish(enemy_point);
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // RCLCPP_INFO(this->get_logger(), "Fly callback time: %f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}


void Cluster::timer_callback()
{
    try 
    {
        transform_stamped = tf_buffer_.lookupTransform("rm_frame", "livox_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(), "TF 转换失败: %s", ex.what());
        return;
    }
    tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);
    tf2::Vector3 t(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);
    tf2::Transform transform_livox_to_rm(q, t);

    std::cout<<"雷达坐标:("<<t.x()<<","<<t.y()<<","<<t.z()<<")"<<std::endl;
    
    tf2::Vector3 enemy_in_rm(22.47, 2.34, 0.4);
    tf2::Vector3 enemy_in_livox = transform_livox_to_rm.inverse() * enemy_in_rm;

    geometry_msgs::msg::Point32 enemy_point;
    enemy_point.x = enemy_in_livox.x();
    enemy_point.y = enemy_in_livox.y();
    enemy_point.z = (enemy_in_rm.z() - t.z());
    std::cout<<"敌方坐标:("<<enemy_point.x<<","<<enemy_point.y<<","<<enemy_point.z<<")"<<std::endl;
    fly_enemy_point_pub_->publish(enemy_point);
    
}
}//namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Cluster)