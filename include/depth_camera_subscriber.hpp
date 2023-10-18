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

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <message_filters/synchronizer.h>
#include "std_msgs/msg/float32_multi_array.hpp"

#include <array>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Transform.h>
#include "pcl/common/transforms.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
class DepthCameraSubscriber : public rclcpp::Node
{
public:
    DepthCameraSubscriber();
    void combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, float minYValue);
    void saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    std::array<float, 2> calculateMapOriginInPixel(const std::array<float, 2>& origin, float resolution, const std::array<int, 2>& image_size);
    std::array<float, 2> calculatePointPixelValue(const std::array<float, 2>& position, const std::array<float, 2>& map_origin_pixel, float resolution);
    void get_transformed_cloud(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform & transform);
bool findTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform & transform);
void getTransformedCloud(pcl::PointCloud<PointT>::Ptr cloud_output, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform  & transform);







private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_projected_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_distance_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_transformed_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_output_publisher_;


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr modified_map_publisher_;


    rclcpp::TimerBase::SharedPtr timer_;
   nav_msgs::msg::OccupancyGrid map_output;
    bool map_data_set = false;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output RCPPUTILS_TSA_GUARDED_BY(lock_);
   

   
 
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
 geometry_msgs::msg::Transform transform;
 std::mutex lock_;
bool is_transformed = false;
bool continue_callback = true;
  

 size_t count_;

   
};

#endif
