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

#include <vector>
#include <visualization_msgs/msg/marker.hpp>






typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
class DepthCameraSubscriber : public rclcpp::Node
{
public:
  DepthCameraSubscriber();
  void combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, float minYValue);
  void saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, std::string file_path);

  std::array<float, 2> calculateMapOriginInPixel(const std::array<float, 2> &origin, float resolution, const std::array<int, 2> &image_size);
  bool findCameraToMapTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform);
  void getTransformedCloud(pcl::PointCloud<PointT>::Ptr cloud_output, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform);
  std::array<float, 2> calculatePointPixelValue(const Eigen::Vector3f &position, const std::array<float, 2> &origin, float resolution);

//void convert1DArrayTo2DImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_transformed_member_);

//std::vector<int8_t> loadOccupancyGrid(const std::string &file_path, int &width, int &height);
//std::vector<std::vector<int8_t>> convertTo2D(const std::vector<int8_t>& occupancy_grid, int map_width, int map_height) ;

//std::vector<std::vector<int8_t>> occupancyGridTo2DImage(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg);

nav_msgs::msg::OccupancyGrid objectDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) ;
std::vector<float> findNearestOccupiedDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void writeDistancesToFile(const std::vector<float> & distances) ;

void determineObjectType();  

bool hasObject(const nav_msgs::msg::OccupancyGrid& map, int threshold);


bool compareOccupancyGridsPOV(const nav_msgs::msg::OccupancyGrid& grid1, const nav_msgs::msg::OccupancyGrid& grid2, double percentage_threshold);
void handleObjectAppearance(const std::vector<int> &counts);
geometry_msgs::msg::Point calculateCentroidInMapFrame(const nav_msgs::msg::OccupancyGrid & grid);
geometry_msgs::msg::Point transformCentroidPointToRobotBase(const geometry_msgs::msg::Point& object_centroid_in_map_frame,  geometry_msgs::msg::Transform map_to_robot_base_transform);

double calculateDistance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2);
double calculateAngle(const geometry_msgs::msg::Point& centroid_in_robot_base_frame);

geometry_msgs::msg::Transform findMapToRobotBaseTransformation(geometry_msgs::msg::Transform &transform);

geometry_msgs::msg::Point transformImgCoordinateToRos(double o_x, double o_y, double resolution, double height, double x, double y);
void printCameraToMapTransform(geometry_msgs::msg::Transform &transform);
void printMapToRobotBaseTransform(geometry_msgs::msg::Transform &transform);









private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_projected_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_distance_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_transformed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_transformed_member_publisher_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr modified_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr buffer_modified_map_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_in_;
  
  bool map_data_set = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_member_ RCPPUTILS_TSA_GUARDED_BY(lock_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_member_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Transform transform_1;
  std::mutex lock_;
  bool is_transformed = false;
  bool continue_callback = true;

  std::array<int, 2> map_size;
  int map_width;
  int map_height;
  float resolution;
  std::array<float, 2> origin_pixel;

  size_t count_;
  rclcpp::Rate rate{0.1}; // 1 / 0.05 = 20 Hz  (every 20s)


  std::deque<nav_msgs::msg::OccupancyGrid> modified_map_buffer_;
  int buffer_size = 2;  

  rclcpp::TimerBase::SharedPtr timer_; 

     rclcpp::Time timer_stamp_;
    std::vector<int> counts_;

    // geometry_msgs::msg::Point previous_object_centroid_in_robot_base_frame;
   // double initial_orientation; 
   // double distance_threshold = 15;       // 10 cm  

   //double orientation_threshold = 10.0;    // 10 degree

  //geometry_msgs::msg::Transform camera_to_map_transform_;  
   //Eigen::Affine3d map_to_robot_base_transform_ ;

 geometry_msgs::msg::Point first_time_object_seen_centroid_ros_;

 geometry_msgs::msg::Transform transform_2;
//geometry_msgs::msg::Transform map_to_robot_base_transform_; 
 nav_msgs::msg::OccupancyGrid main_object_modified_map_;
};

#endif
