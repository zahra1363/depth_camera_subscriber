#include "depth_camera_subscriber.hpp"
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/conversions.h>
#include <functional>
#include <utility>
#include "rclcpp/rclcpp.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Geometry>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

DepthCameraSubscriber::DepthCameraSubscriber()
    : Node("depth_camera_subscriber"), count_(0), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/right_camera/depth/color/points", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr pc_in)
      {
        this->combined_callback(pc_in, nullptr);
      });

  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
      {
        this->combined_callback(nullptr, map_in);
      });

  // timer_ = this->create_wall_timer(std::chrono::minutes(1), std::bind(&DepthCameraSubscriber::timerCallback, this));

  cloud_in_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_in", rclcpp::SensorDataQoS());
  cloud_projected_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_projected", rclcpp::SensorDataQoS());
  cloud_filtered_distance_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", rclcpp::SensorDataQoS());
  cloud_transformed_member_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_transformed", rclcpp::SensorDataQoS());

  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cloud_map_in", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("detected_object_grid", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  buffer_modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("buffer_modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("transformed_point_marker", 10);
}

void DepthCameraSubscriber::combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
{
  if (pc_in != nullptr)
  {
    // std::cout << "pc_in is loaded" << std::endl;
    sensor_msgs::msg::PointCloud2 output;
    output.header = pc_in->header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_in, *cam_cloud_in);

    pcl::toROSMsg(*cam_cloud_in, output); // convert PC to ROS message
    cloud_in_publisher_->publish(output);

    pcl::io::savePCDFileASCII("src/depth_camera_subscriber/param/pc_save_XYZ.pcd", *cam_cloud_in);

    // convert 3D point Clouds to 2D image(2D plane)
    PointCloudT::Ptr cloud_projected(new PointCloudT);
    //  Create a set of planar coefficients with X=Z=0,Y=1 (Ax + By + Cz + D = 0)  which is match with map like ground plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1.0;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> projection;
    projection.setModelType(pcl::SACMODEL_PLANE); // filter type is a plane
    projection.setInputCloud(cam_cloud_in);
    projection.setModelCoefficients(coefficients); //  set the plane coef.
    projection.filter(*cloud_projected);
    pcl::io::savePCDFileASCII("src/depth_camera_subscriber/param/cloud_projected.pcd", *cloud_projected);

    pcl::toROSMsg(*cloud_projected, output); // convert PC to ROS message
    cloud_projected_publisher_->publish(output);

    // removed points which are mor ethan 2 meters far away
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    float max_distance_threshold = 2.0; // distance from camera

    for (const pcl::PointXYZ &point : cloud_projected->points)
    {
      float distance_2d = std::sqrt(point.x * point.x + point.z * point.z);

      if (distance_2d <= max_distance_threshold)
      {
        cloud_filtered->push_back(point);
      }
    }
    sensor_msgs::msg::PointCloud2 output_1;
    pcl::toROSMsg(*cloud_filtered, output_1); // convert PC to ROS message
    output_1.header = pc_in->header;
    cloud_filtered_distance_publisher_->publish(output_1);

    cloud_filtered_member_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    *cloud_filtered_member_ = *cloud_filtered;
  }

  // process on the map
  if (map_in != nullptr)
  {
    std::cout << "map_in is loaded" << std::endl;
    std::string map_path = "src/depth_camera_subscriber/param/map.pgm";
    saveMapAsPGM(map_in, map_path);

    // publish map
    nav_msgs::msg::OccupancyGrid map_output;
    map_output.header = map_in->header;
    map_output.info = map_in->info;
    map_output.data = map_in->data;
    map_publisher_->publish(map_output);
    map_data_set = true;

    std::array<float, 2> origin = {map_in->info.origin.position.x, map_in->info.origin.position.y};
    resolution = map_in->info.resolution;

    map_size = {map_in->info.width, map_in->info.height};
    map_width = map_in->info.width;
    map_height = map_in->info.height;

    std::array<float, 2> map_origin_pixel = calculateMapOriginInPixel(origin, resolution, map_size);

    origin_pixel = {map_origin_pixel[0], map_origin_pixel[1]};
    std::cout << "map-origin_pixel_x : " << map_origin_pixel[0] << " , map_origin_pixel_y : " << map_origin_pixel[1] << std::endl;

    map_in_ = map_in;
  }

  // process on map &pc
  if (pc_in != nullptr && map_data_set)
  {
    // std::cout << "Applying rigid transformation to: filtered_cloud in camera frame -> map frame" << std::endl;

    PointCloudT::Ptr cloud_transformed(new PointCloudT);
    const std::string &destination_frame_id = "map";
    const std::string &current_frame_id = "right_camera_depth_optical_frame";
    is_transformed = findCameraToMapTransformation(current_frame_id, destination_frame_id, cloud_transformed, transform_1);
    if (continue_callback)
    {
      // std::cout << "point cloud is transformed to the map frame" << std::endl;
      *cloud_transformed_member_ = *cloud_transformed;

      sensor_msgs::msg::PointCloud2 output_2;
      output_2.header = pc_in->header;
      pcl::toROSMsg(*cloud_transformed_member_, output_2);
      cloud_transformed_member_publisher_->publish(output_2);

      // convert 1D pointcloud to image
      //  convert1DArrayTo2DImage(cloud_transformed_member_);

      //  size_t number_of_points = cloud_transformed_member_->points.size();
      // std::cout<<"point cloud size is: " <<number_of_points<< std::endl;

      std::vector<float> distances = findNearestOccupiedDistance(cloud_transformed_member_);
      writeDistancesToFile(distances);

      nav_msgs::msg::OccupancyGrid modified_map;
      modified_map = objectDetection(cloud_transformed_member_);
      modified_map_publisher_->publish(modified_map);

      // printCameraToMapTransform(camera_to_map_transform_);

      // map_to_robot_base_transform_= findMapToRobotBaseTransform(camera_to_map_transform_);
      // printMapToRobotBaseTransform(map_to_robot_base_transform_);

      // bool is_transformation_available = findMapToRobotBaseTransformation(transform_2);
      // printMapToRobotBaseTransform(map_to_robot_base_transform_);

      determineObjectType();

    } // continue_callback

  } // end of (map_in != 0 && map_data_set)
  rate.sleep();

} // end of call back





void DepthCameraSubscriber::determineObjectType()
{
  nav_msgs::msg::OccupancyGrid modified_map;
  modified_map = objectDetection(cloud_transformed_member_);

  int min_object_pixels = 10;
  bool has_object = hasObject(modified_map, min_object_pixels);

  if (has_object)
  {
    // Calculate the centroid of the object in the map frame
    geometry_msgs::msg::Point object_centroid_in_map_frame_image_coordinate = calculateCentroidInMapFrame(modified_map);
    // std::cout << "object centroid in map frame in image coordinate: ( " << object_centroid_in_map_frame_image_coordinate.x << ", " << object_centroid_in_map_frame_image_coordinate.y << ")" <<std::endl;

    geometry_msgs::msg::Point object_centroid_in_map_frame_ros_coordinate = transformImgCoordinateToRos(map_in_->info.origin.position.x, map_in_->info.origin.position.y, map_in_->info.resolution,
                                                                                                        map_in_->info.height, object_centroid_in_map_frame_image_coordinate.x, object_centroid_in_map_frame_image_coordinate.y);

    // std::cout << "object centroid in map frame in ros coordinate: ( " << object_centroid_in_map_frame_ros_coordinate.x << ", " << object_centroid_in_map_frame_ros_coordinate.y << ")" << std::endl;

    geometry_msgs::msg::Transform map_to_robot_base_transform = findMapToRobotBaseTransformation(transform_2);
    geometry_msgs::msg::Point object_centroid_in_robot_base_frame = transformCentroidPointToRobotBase(object_centroid_in_map_frame_ros_coordinate, map_to_robot_base_transform);
    std::cout << " New object centroid in robot base frame is (" << object_centroid_in_robot_base_frame.x << ", " << object_centroid_in_robot_base_frame.y << ", " << object_centroid_in_robot_base_frame.z << ")" << std::endl;

    // Check if this is the first time the object is seen
    if (timer_stamp_.seconds() == 0)
    {
      // Save the centroid in ros_coor and update the timer
      counts_.push_back(1);
      std::cout << "1 added to counter" << std::endl;
      first_time_object_seen_centroid_ros_ = object_centroid_in_map_frame_ros_coordinate;
      // std::cout << " C1 centroid in ros map frame : ( " << first_time_object_seen_centroid_ros_.x << ", " << first_time_object_seen_centroid_ros_.y << ", " << first_time_object_seen_centroid_ros_.z << ")" << std::endl;
      main_object_modified_map_ = modified_map;
      timer_stamp_ = this->now();
      std::cout << "First time object is seen. Timer started." << std::endl;
    }
    else
    {
      geometry_msgs::msg::Transform map_to_robot_base_transform = findMapToRobotBaseTransformation(transform_2);
      // printMapToRobotBaseTransform(map_to_robot_base_transform_);

      geometry_msgs::msg::Point c1_object_centroid_in_robot_base_frame;
      c1_object_centroid_in_robot_base_frame = transformCentroidPointToRobotBase(first_time_object_seen_centroid_ros_, map_to_robot_base_transform);
      std::cout << "Main object centroid in robot base frame is (" << c1_object_centroid_in_robot_base_frame.x << ", " << c1_object_centroid_in_robot_base_frame.y << ", " << c1_object_centroid_in_robot_base_frame.z << ")" << std::endl;

      geometry_msgs::msg::Point base_origin_coordinate; // base_link origin coordinate
      base_origin_coordinate.x = 0.0;
      base_origin_coordinate.y = 0.0;
      base_origin_coordinate.z = 0.0;

      double distance = calculateDistance(c1_object_centroid_in_robot_base_frame, base_origin_coordinate);
      std::cout << "Distance from main object : " << abs(distance) << std::endl;

      double angle = calculateAngle(c1_object_centroid_in_robot_base_frame);
      std::cout << "Angle for main object: " << abs(angle) << std::endl;

      // Check the condition whether C1 is visible
      if (std::abs(distance) < 2.0 && std::abs(angle) < (87.0 / 2.0))
      {
        std::cout << "Main object is visible "<< std::endl;
        // Calculate the distance between the centroids
        double centroids_differences = calculateDistance(c1_object_centroid_in_robot_base_frame, object_centroid_in_robot_base_frame);
        std::cout << "Distance differences from objects centroids : " << centroids_differences << std::endl;

        // Check if the centroids are close enough
        double distance_threshold = 0.1; // 10 cm
        if (centroids_differences <= distance_threshold)
        {

          // check from different POV
          modified_map_buffer_.push_back(main_object_modified_map_);
          modified_map_buffer_.push_back(modified_map);

          if (modified_map_buffer_.size() >= 2)
          {
            const nav_msgs::msg::OccupancyGrid &modified_map_1 = modified_map_buffer_[0];
            const nav_msgs::msg::OccupancyGrid &modified_map_2 = modified_map_buffer_[1];

            double percentage_threshold = 80; // 80% should be common
            bool result = compareOccupancyGridsPOV(modified_map_1, modified_map_2, percentage_threshold);
            if (result)
            {
              std::cout << "The object is same from other POV." << std::endl;
              std::cout << "---------------------------------------------" << std::endl;

              rclcpp::Duration time_difference = this->now() - timer_stamp_;
              if (time_difference.seconds() >= 60)
              {
                int intervals = static_cast<int>(time_difference.seconds()) / 60;
                for (int i = 0; i < intervals; ++i)
                {
                  counts_.push_back(1);
                  std::cout << "1 added to counter" << std::endl;
                }
                timer_stamp_ = this->now();
                modified_map_buffer_.clear();
              }
              else
              {
                std::cout << "Object seen again within 60 seconds. Ignoring." << std::endl;
              }
            }
          }
        }
        else
        {
          handleObjectAppearance(counts_);
          counts_.clear();                   // Reset counts_ for the next object
          timer_stamp_ = rclcpp::Time(0, 0); // Reset the timer_stamp
        }
        if (counts_.size() >= 8)
        {
          std::cout << "Object continuously visible for more than 8 unit time (2 hours). Running handle function." << std::endl;
          handleObjectAppearance(counts_);
          counts_.clear();                   // Reset counts_ for the next object
          timer_stamp_ = rclcpp::Time(0, 0); // Reset the timer_stamp
        }
      }
      else
      {
        std::cout << "Main object is not visible." << std::endl;
        std::cout << " ---------------------------------------------------------------------------- " << std::endl;
      }
    }
  }
}



void DepthCameraSubscriber::saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, std::string file_path)
{
  std::ofstream pgm_file(file_path, std::ios::out | std::ios::binary);

  if (pgm_file.is_open())
  {
    pgm_file << "P5\n";                                                     // Magic number for PGM binary format
    pgm_file << map_msg->info.width << " " << map_msg->info.height << "\n"; // Width and height
    pgm_file << "255\n";                                                    // Maximum gray value

    for (int i = 0; i < map_msg->info.width * map_msg->info.height; ++i)
    {
      uint8_t pixel_value = 255 - map_msg->data[i];
      pgm_file.write(reinterpret_cast<const char *>(&pixel_value), sizeof(pixel_value));
    }

    pgm_file.close();
    std::cout << "Map saved as map.pgm" << std::endl;
  }
  else
  {
    std::cerr << "Error: Unable to open file for writing." << std::endl;
  }
}

bool DepthCameraSubscriber::findCameraToMapTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform)
{
  // Return of pointcloud is empty
  if (cloud_filtered_member_->points.size() == 0)
  {
    rclcpp::get_logger("pointcloud empty / Pointcloud not recieved.");
    return false;
  }

  try
  {
    transform = tf_buffer_->lookupTransform(destination_frame_id, current_frame_id, tf2::TimePointZero).transform;

    // save as a class memebr
    // camera_to_map_transform_ = transform;

    std::scoped_lock lock(lock_);
    getTransformedCloud(cloud_filtered_member_, cloud_transformed, transform);
    cloud_transformed->header.frame_id = destination_frame_id;
    continue_callback = true;
    return true;
  }

  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
                 "Exception in lookupTransform: %s",
                 ex.what());
    continue_callback = false;
  }
  if (!continue_callback)
    return false;
}

void DepthCameraSubscriber::getTransformedCloud(pcl::PointCloud<PointT>::Ptr cloud_filtered_member_, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform)
{
  Eigen::Quaternion<float> rotation(transform.rotation.w,
                                    transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z);

  Eigen::Vector3f translation(transform.translation.x,
                              transform.translation.y,
                              transform.translation.z);

  if (cloud_filtered_member_->size() < 1)
  {
    cloud_transformed = cloud_filtered_member_;

    return;
  }
  pcl::transformPointCloud(*cloud_filtered_member_, *cloud_transformed, translation, rotation);

  cloud_transformed_member_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  *cloud_transformed_member_ = *cloud_transformed;
}

std::array<float, 2> DepthCameraSubscriber::calculateMapOriginInPixel(const std::array<float, 2> &origin, float resolution, const std::array<int, 2> &map_size)
{
  float x = -1 * origin[0] / resolution;
  float y = map_size[1] + (origin[1] / resolution); // y axis is flipped

  return {x, y};
}

// calculate pixel coordinate of each point
std::array<float, 2> DepthCameraSubscriber::calculatePointPixelValue(const Eigen::Vector3f &position, const std::array<float, 2> &origin, float resolution)
{
  float delta_x = position[0] / resolution;
  float delta_y = position[1] / resolution;

  float x = origin[0] + delta_x;
  float y = origin[1] + (-1 * delta_y);

  return {x, y};
}

nav_msgs::msg::OccupancyGrid DepthCameraSubscriber::objectDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  nav_msgs::msg::OccupancyGrid modified_map;
  modified_map.info = map_in_->info;
  modified_map.header = map_in_->header;
  modified_map.data.resize(map_in_->info.width * map_in_->info.height, 0);

  for (const pcl::PointXYZ &point : cloud->points)
  {
    Eigen::Vector3f position(point.x, point.y, point.z);
    std::array<float, 2> point_pixel = calculatePointPixelValue(position, origin_pixel, resolution);

    int x = static_cast<int>(point_pixel[0]);
    int y = static_cast<int>(point_pixel[1]);

    if (x >= 1 && x < map_in_->info.width - 1 && y >= 1 && y < map_in_->info.height - 1)
    {
      bool isOccupied = false;

      // Check 10 neighboring points
      for (int dx = -2; dx <= 2; ++dx)
      {
        for (int dy = -2; dy <= 2; ++dy)
        {
          int nx = x + dx;
          int ny = y + dy;

          int index = map_in_->info.width * (map_in_->info.height - ny - 1) + nx;
          if (map_in_->data[index] == 100 | map_in_->data[index] == -1) 
          {
            isOccupied = true;
            break;
          }
        }
      }

      if (isOccupied)
      {
        int index = map_in_->info.width * (map_in_->info.height - y - 1) + x;
        modified_map.data[index] = 0; 
      }

      else
      {
        int index = map_in_->info.width * (map_in_->info.height - y - 1) + x;
        modified_map.data[index] = 100; 
      }
    }
  }
  return modified_map;
}

std::vector<float> DepthCameraSubscriber::findNearestOccupiedDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  std::vector<float> distances; 
  for (const pcl::PointXYZ &point : cloud->points)
  {
    Eigen::Vector3f position(point.x, point.y, point.z);
    std::array<float, 2> point_pixel = calculatePointPixelValue(position, origin_pixel, resolution);

    int x = static_cast<int>(point_pixel[0]);
    int y = static_cast<int>(point_pixel[1]);

    float min_distance = std::numeric_limits<float>::max();

    for (int dx = -20; dx <= 20; ++dx)
    {
      for (int dy = -20; dy <= 20; ++dy)
      {
        int nx = x + dx;
        int ny = y + dy;

        if (nx >= 0 && nx < map_in_->info.width && ny >= 0 && ny < map_in_->info.height)
        {
          int index = map_in_->info.width * (map_in_->info.height - ny - 1) + nx;

          if (map_in_->data[index] == 100)
          {
            float distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            if (distance < min_distance)
            {
              min_distance = distance;
            }
          }
        }
      }
    }

    if (min_distance != std::numeric_limits<float>::max())
    {
      distances.push_back(min_distance);
    }
  }
  return distances;
}

void DepthCameraSubscriber::writeDistancesToFile(const std::vector<float> &distances)
{
  const std::string &file_path = "src/depth_camera_subscriber/param/distances.txt";
  std::ofstream outputFile(file_path);

  if (outputFile.is_open())
  {
    for (const float &distance : distances)
    {
      outputFile << distance << "\n";
    }
    outputFile.close();
    // std::cout << "Distances are written to distances.txt " << std::endl;
  }
  else
  {
    // std::cerr << "Unable to open file for writing" << std::endl;
  }
}

bool DepthCameraSubscriber::hasObject(const nav_msgs::msg::OccupancyGrid &occupancy_grid, int threshold)
{
  int occupied_count = 0;
  for (int i = 0; i < occupancy_grid.data.size(); ++i)
  {
    if (occupancy_grid.data[i] == 100)
    {
      occupied_count++;
    }
  }
  if (occupied_count >= threshold)
  {

    std::cout << "Map has an object with " << occupied_count << " pixels. " << std::endl;
    return true;
  }

  std::cout << "Map with " << occupied_count << " pixels "
            << "is considered empty." << std::endl;
  return false;
}

bool DepthCameraSubscriber::compareOccupancyGridsPOV(const nav_msgs::msg::OccupancyGrid &grid1, const nav_msgs::msg::OccupancyGrid &grid2, double percentage_threshold)
{
  if (grid1.info.width != grid2.info.width || grid1.info.height != grid2.info.height)
  {
    return false;
  }

  int counter = 0;

  int occupied_count_grid1 = 0;
  int occupied_count_grid2 = 0;

  for (int i = 0; i < grid1.data.size(); ++i)
  {
    if (grid1.data[i] == 100)
    {
      occupied_count_grid1++;
    }
  }

  for (int i = 0; i < grid2.data.size(); ++i)
  {
    if (grid2.data[i] == 100)
    {
      occupied_count_grid2++;
    }
  }

  int common_threshold = std::round(std::min(occupied_count_grid1, occupied_count_grid2) * (percentage_threshold / 100.0));
  std::cout << "common_threshold is :" << common_threshold << std::endl;

  if (occupied_count_grid1 > occupied_count_grid2)
  {
    for (int i = 1; i < grid2.info.width - 1; ++i)
    {
      for (int j = 1; j < grid2.info.height - 1; ++j)
      {
        int index_2 = grid2.info.width * j + i;

        if (grid2.data[index_2] == 100)
        {
          int x = i;
          int y = grid2.info.height - j - 1;

          bool foundOccupied = false;

          for (int dx = -occupied_count_grid1; dx <= occupied_count_grid1; ++dx)
          {
            for (int dy = -occupied_count_grid1; dy <= occupied_count_grid1; ++dy)
            {
              int nx = x + dx;
              int ny = y + dy;

              if (nx >= 0 && nx < grid1.info.width && ny >= 0 && ny < grid1.info.height)
              {
                int index_1 = grid1.info.width * (grid1.info.height - ny - 1) + nx;

                if (grid1.data[index_1] == 100)
                {
                  foundOccupied = true;
                  break;
                }
              }
            }
            if (foundOccupied)
            {
              break;
            }
          }

          if (foundOccupied)
          {
            counter++;
          }
        }
      }
    }
  }
  else
  {
    for (int i = 1; i < grid1.info.width - 1; ++i)
    {
      for (int j = 1; j < grid1.info.height - 1; ++j)
      {
        int index_1 = grid1.info.width * j + i;

        if (grid1.data[index_1] == 100)
        {
          int x = i;
          int y = grid1.info.height - j - 1;

          bool foundOccupied = false;

          for (int dx = -occupied_count_grid2; dx <= occupied_count_grid2; ++dx)
          {
            for (int dy = -occupied_count_grid2; dy <= occupied_count_grid2; ++dy)
            {
              int nx = x + dx;
              int ny = y + dy;

              if (nx >= 0 && nx < grid2.info.width && ny >= 0 && ny < grid2.info.height)
              {
                int index_2 = grid2.info.width * (grid2.info.height - ny - 1) + nx;

                if (grid2.data[index_2] == 100)
                {
                  foundOccupied = true;
                  break;
                }
              }
            }
            if (foundOccupied)
            {
              break;
            }
          }

          if (foundOccupied)
          {
            counter++;
          }
        }
      }
    }
  }

  std::cout << "Occupied pixels in grid1: " << occupied_count_grid1 << std::endl;
  std::cout << "Occupied pixels in grid2: " << occupied_count_grid2 << std::endl;
  std::cout << "Number of common pixels is: " << counter << " (" << (static_cast<double>(std::min(counter, common_threshold)) / common_threshold) * 100 << "%)" << std::endl;
  return counter >= common_threshold; 
}

void DepthCameraSubscriber::handleObjectAppearance(const std::vector<int> &counts)
{

  for (size_t i = 0; i < counts.size(); ++i)
  {
    RCLCPP_INFO(this->get_logger(), "Counter[%zu]: %d", i, counts[i]);
  }
  int total_count = 0;
  for (const auto &count : counts)
  {
    total_count += count;
  }
  if (total_count == 1)
  {
    RCLCPP_INFO(this->get_logger(), "The object belongs to Group 1, including people or other robots.");
  }
  else if (total_count == 2)
  {
    RCLCPP_INFO(this->get_logger(), "The object belongs to Group 2, including trolley, forklift or shopping cart.");
  }
  else if (total_count >= 3)
  {
    
    RCLCPP_INFO(this->get_logger(), "The object is identified as a static entity, and the map should be updated by adding this object into it.");
  }
}

geometry_msgs::msg::Point DepthCameraSubscriber::calculateCentroidInMapFrame(const nav_msgs::msg::OccupancyGrid &grid)
{
  geometry_msgs::msg::Point centroid;
  int total_occupied_cell = 0;
  float sum_x = 0.0;
  float sum_y = 0.0;

  for (int y = 0; y < grid.info.height; ++y)
  {
    for (int x = 0; x < grid.info.width; ++x)
    {
      int index = grid.info.width * (grid.info.height - y - 1) + x;
      if (grid.data[index] == 100)
      {
        sum_x += x;
        sum_y += y;
        ++total_occupied_cell;
      }
    }
  }

  if (total_occupied_cell > 0)
  {
    centroid.x = sum_x / total_occupied_cell;
    centroid.y = sum_y / total_occupied_cell;
    centroid.z = 0.0;
  }

  return centroid;
}

geometry_msgs::msg::Point DepthCameraSubscriber::transformCentroidPointToRobotBase(const geometry_msgs::msg::Point &object_centroid_in_map_frame, geometry_msgs::msg::Transform transform)
{
  
  Eigen::Vector4d object_centroid_vector;
  object_centroid_vector << object_centroid_in_map_frame.x, object_centroid_in_map_frame.y, 0.0, 1.0;

  // Extract rotation and translation
  Eigen::Quaterniond rotation(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3d translation(transform.translation.x, transform.translation.y, transform.translation.z);

  // Construct transformation matrix
  Eigen::Affine3d transform_affine = Eigen::Translation3d(translation) * rotation;

  // Apply transformation to the object centroid
  Eigen::Vector4d point_in_robot_link_frame = transform_affine * object_centroid_vector;

  // Extract the transformed coordinates
  geometry_msgs::msg::Point point_in_robot_link_frame_msg;
  point_in_robot_link_frame_msg.x = point_in_robot_link_frame(0);
  point_in_robot_link_frame_msg.y = point_in_robot_link_frame(1);
  point_in_robot_link_frame_msg.z = point_in_robot_link_frame(2);

  return point_in_robot_link_frame_msg;
}

// Calculate distance of object centroid in map frame to transformed point in robot base frame
double DepthCameraSubscriber::calculateDistance(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double dz = point1.z - point2.z;

  return sqrt(dx * dx + dy * dy + dz * dz);
}

double DepthCameraSubscriber::calculateAngle(const geometry_msgs::msg::Point &centroid_in_robot_base_frame)
{
  // Calculate the vector components
  double vector_x = centroid_in_robot_base_frame.x;
  double vector_y = centroid_in_robot_base_frame.y;

  double angle = std::atan2(vector_y, vector_x);

  // Convert rad to degrees
  angle = angle * (180.0 / M_PI);

  return angle;
}

void DepthCameraSubscriber::printCameraToMapTransform(geometry_msgs::msg::Transform &transform)
{
  Eigen::Quaternion<float> rotation(transform.rotation.w,
                                    transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z);

  Eigen::Vector3f translation(transform.translation.x,
                              transform.translation.y,
                              transform.translation.z);

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  transform_matrix.block<3, 1>(0, 3) = translation;

  std::cout << "Camera To Map Transformation Matrix:" << std::endl;
  std::cout << transform_matrix << std::endl;
}

geometry_msgs::msg::Transform DepthCameraSubscriber::findMapToRobotBaseTransformation(geometry_msgs::msg::Transform &transform)
{
  try
  {
    std::string current_frame_id = "map";
    std::string destination_frame_id = "base_link";
    transform = tf_buffer_->lookupTransform(destination_frame_id, current_frame_id, tf2::TimePointZero).transform;
    return transform;
  }

  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in lookupTransform: %s", ex.what());
  }
}

void DepthCameraSubscriber::printMapToRobotBaseTransform(geometry_msgs::msg::Transform &transform)
{
  Eigen::Quaternion<float> rotation(transform.rotation.w,
                                    transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z);

  Eigen::Vector3f translation(transform.translation.x,
                              transform.translation.y,
                              transform.translation.z);

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  transform_matrix.block<3, 1>(0, 3) = translation;

  std::cout << "Map To Robot Base Transformation Matrix" << std::endl;
  std::cout << transform_matrix << std::endl;
}

// getting map origin in meter from map.yml
geometry_msgs::msg::Point DepthCameraSubscriber::transformImgCoordinateToRos(double map_origin_x, double map_origin_y, double map_resolution, double map_height, double pixel_x, double pixel_y)
{
  double x = static_cast<double>(pixel_x);
  double y = static_cast<double>(pixel_y);
  double o_x = static_cast<double>(map_origin_x);
  double o_y = static_cast<double>(map_origin_y);
  double resolution = static_cast<double>(map_resolution);
  double height = static_cast<double>(map_height);

  geometry_msgs::msg::Point object_centroid_in_ros_map_frame;
  object_centroid_in_ros_map_frame.x = round(x * resolution + o_x);
  object_centroid_in_ros_map_frame.y = round(height * resolution - y * resolution + o_y);

  return object_centroid_in_ros_map_frame;
}







/**
std::vector<int8_t> DepthCameraSubscriber::loadOccupancyGrid(const std::string &file_path, int &width, int &height)
{
  std::ifstream pgm_file(file_path, std::ios::in | std::ios::binary);

  if (!pgm_file.is_open())
  {
    std::cerr << "Error: Unable to open file for reading." << std::endl;
    return {};
  }

  // Read the PGM header
  std::string magic_number;
  int max_val;
  pgm_file >> magic_number >> width >> height >> max_val;

  if (magic_number != "P5")
  {
    std::cerr << "Error: Invalid PGM format." << std::endl;
    return {};
  }

  // Read the pixel data
  std::vector<int8_t> occupancy_grid(width * height);
  pgm_file.read(reinterpret_cast<char *>(occupancy_grid.data()), width * height);

  // Close the file
  pgm_file.close();

  return occupancy_grid;
}
**/

/**
std::vector<std::vector<int8_t>> DepthCameraSubscriber::occupancyGridTo2DImage(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg)
{
  int map_width = map_msg->info.width;
  int map_height = map_msg->info.height;

  std::vector<std::vector<int8_t>> map_2d(map_height, std::vector<int8_t>(map_width));

  for (int i = 0; i < map_height; ++i)
  {
    for (int j = 0; j < map_width; ++j)
    {
      int index = i * map_width + j;
      map_2d[i][j] = map_msg->data[index];
    }
  }

  return map_2d;
}
**/

/**
std::vector<std::vector<int8_t>> DepthCameraSubscriber::convertTo2D(const std::vector<int8_t> &occupancy_grid, int width, int height)
{
  std::vector<std::vector<int8_t>> result(height, std::vector<int8_t>(width));

  for (int y = 0; y < map_height; ++y)
  {
    for (int x = 0; x < map_width; ++x)
    {
      result[y][x] = occupancy_grid[y * map_width + x];
    }
  }

  return result;
}
**/

/**
 pcl::PointCloud<pcl::PointXYZ>::Ptr DepthCameraSubscriber::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, float minYValue)
{
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // inliers:ground points
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // koef. for ground plane(Ax+By+Cz+D=0)

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    // Points farther than this threshold will be considered outliers.
    seg.setDistanceThreshold(minYValue + 0.005); // 0.5 cm

    seg.setInputCloud(inputCloud);

    seg.segment(*inliers, *coefficients); // Segment the largest planar component and put it in the inliers

    // Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // true: extracting non-ground points; false: extract ground points
    extract.filter(*ground_removed_cloud);
    return ground_removed_cloud;
} **/
