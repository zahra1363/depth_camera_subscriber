#ifndef DEPTH_CAMERA_SUBSCRIBER_HPP_
#define DEPTH_CAMERA_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



class DepthCameraSubscriber : public rclcpp::Node
{
public:
    DepthCameraSubscriber();
    void combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in );
    //std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr, const nav_msgs::msg::OccupancyGrid::SharedPtr)> combined_callback;
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, float minYValue);
    void saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    
};

#endif 
