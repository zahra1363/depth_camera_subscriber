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

  timer_ = this->create_wall_timer(std::chrono::minutes(1), std::bind(&DepthCameraSubscriber::timerCallback, this));

  cloud_in_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", rclcpp::SensorDataQoS());
  cloud_projected_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_projected", rclcpp::SensorDataQoS());
  cloud_filtered_distance_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered_distance_2d", rclcpp::SensorDataQoS());
  cloud_transformed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_transformed", rclcpp::SensorDataQoS());
  cloud_transformed_member_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_transformed_member", rclcpp::SensorDataQoS());

  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cloud_map_in", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  buffer_modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("buffer_modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void DepthCameraSubscriber::combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
{
  if (pc_in != nullptr)
  {
    std::cout << "pc_in is loaded" << std::endl;
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

    // removed points which are mor ethan 3 meters far away
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    float max_distance_threshold = 1.3; // distance from camera

    for (const pcl::PointXYZ &point : cloud_projected->points)
    {
      float distance_2d = std::sqrt(point.x * point.x + point.y * point.y);

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
    is_transformed = findTransformation(current_frame_id, destination_frame_id, cloud_transformed, transform);
    if (continue_callback)
    {
      std::cout << "point cloud is transformed to the map frame" << std::endl;
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
  modified_map = comparePointCloudWithMap(cloud_transformed_member_);
  modified_map_publisher_->publish(modified_map);































    } // continue_callback

  } // end of (map_in != 0 && map_data_set)
  rate.sleep();

} // end of call back





















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


bool DepthCameraSubscriber::findTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform &transform)
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
  /**
  // printout transformation matrix
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    transform_matrix.block<3, 1>(0, 3) = translation;

    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << transform_matrix << std::endl;
    **/

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







nav_msgs::msg::OccupancyGrid DepthCameraSubscriber::comparePointCloudWithMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  /**
  // print out the map element
  std::set<int> uniqueValues;
      for(auto data : map_in_->data)
     {
         uniqueValues.insert(static_cast<int>(data));
      }
      for (int value : uniqueValues) {
      std::cout << value << " ";
      }
      std::cout << std::endl;
      **/

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

    // std::cout<< "x:"<<x <<"  y:"<<y <<std::endl;

    if (x >= 1 && x < map_in_->info.width - 1 && y >= 1 && y < map_in_->info.height - 1)
    {
      bool isOccupied = false;

      // Check 10 neighboring points
      for (int dx = -4; dx <= 4; ++dx)
      {
        for (int dy = -4; dy <= 4; ++dy)
        {
          int nx = x + dx;
          int ny = y + dy;

          int index = map_in_->info.width * (map_in_->info.height - ny - 1) + nx;
          if (map_in_->data[index] == 100)
          {
            isOccupied = true;
            break;
          }
        }
      }

      if (isOccupied)
      {
        int index = map_in_->info.width * (map_in_->info.height - y - 1) + x;
        modified_map.data[index] = 0; // Set corresponding cell as "unoccupied"
      }

      else
      {
        int index = map_in_->info.width * (map_in_->info.height - y - 1) + x;
        modified_map.data[index] = 100; // Set corresponding cell as "occupied"
      }
    }
  }

  return modified_map;
}


std::vector<float> DepthCameraSubscriber::findNearestOccupiedDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  std::vector<float> distances; // Vector to store distance
   // std::set<float> distances; 

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
       //distances.insert(min_distance);
    }
  }
  
  //for (const auto & distance : distances)
  //{
   // std::cout << "Distance: " << distance << std::endl;
  //}
  return distances;
}


void DepthCameraSubscriber::writeDistancesToFile(const std::vector<float> & distances) 
{
   const std::string& file_path = "src/depth_camera_subscriber/param/distances.txt";
    std::ofstream outputFile(file_path);

    if (outputFile.is_open()) {
        for (const float& distance : distances) {
            outputFile << distance << "\n"; 
        }
        outputFile.close();
       // std::cout << "Distances are written to distances.txt " << std::endl;
    } else {
       // std::cerr << "Unable to open file for writing" << std::endl;
    }
}



// This function will be called every 15 minutes
void DepthCameraSubscriber::timerCallback()
{
    if (map_data_set)
    {
        nav_msgs::msg::OccupancyGrid modified_map;
        modified_map = comparePointCloudWithMap(cloud_transformed_member_);
      // int min_object_pixels = 100;
      //  bool has_object = hasObject(modified_map, min_object_pixels);

       printOccupiedPointsCount(modified_map);

        if (modified_map_buffer.size() < buffer_size)
        {
            modified_map_buffer.push_back(modified_map);
            buffer_modified_map_publisher_->publish(modified_map);

        }
        else
        {
            modified_map_buffer.pop_front();
            modified_map_buffer.push_back(modified_map);
             buffer_modified_map_publisher_->publish(modified_map);
        }
    }
        RCLCPP_INFO(this->get_logger(), "Modified map added to buffer. Buffer size: %d", modified_map_buffer.size());



  if (modified_map_buffer.size() >= 2) {
    const nav_msgs::msg::OccupancyGrid& modified_map_1 = modified_map_buffer[0];
    const nav_msgs::msg::OccupancyGrid& modified_map_2 = modified_map_buffer[1];

       int  min_commmon_pixels = 20;
        bool result = compareOccupancyGrids(modified_map_1, modified_map_2,min_commmon_pixels );

        if (result)
          {
              std::cout << "The objects are located in the same place in both maps." << std::endl;
          }
        else
         {
            std::cout << "The objects are not located in the same place in both maps." << std::endl;
         }
  }
  else {
    std::cout << "Not enough maps in the buffer for comparison." << std::endl;
}

}


bool DepthCameraSubscriber::hasObject(const nav_msgs::msg::OccupancyGrid& map, int threshold) {
    int count = 0;
    for(auto data : map.data) 
    {
        if (data == 100) {
            count++;
        }
        if (count >= threshold) {
          std::cout<<" map with object"<<std::endl;
            return true; // Found enough cells with value 100
        }
    }
              std::cout<<" empty map "<<std::endl;
    return false; // Didn't find enough cells with value 100
}


bool DepthCameraSubscriber::compareOccupancyGrids(const nav_msgs::msg::OccupancyGrid& grid1, const nav_msgs::msg::OccupancyGrid& grid2, int threshold)
{
    if (grid1.info.width != grid2.info.width || grid1.info.height != grid2.info.height)
    {
        return false;
    }

    int counter = 0;

    for (int i = 0; i < grid1.data.size(); ++i)
    {
        if (grid1.data[i] == 100)
        {
            int x = i % grid1.info.width;
            int y = grid1.info.height - (i / grid1.info.width) - 1;

            bool foundOccupied = false;

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx >= 0 && nx < grid2.info.width && ny >= 0 && ny < grid2.info.height)
                    {
                       // int index = grid2.info.width * ny + nx;
                       int index = grid2.info.width * (grid2.info.height - ny - 1) + nx;

                        if (grid2.data[index] == 100)
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
    std::cout<< "number of common pixels is :  "<< counter<< std::endl;
    return counter > threshold;
}

void DepthCameraSubscriber::printOccupiedPointsCount(const nav_msgs::msg::OccupancyGrid& occupancy_grid)
{
    int occupied_count = 0;

    for (int i = 0; i < occupancy_grid.data.size(); ++i)
    {
        if (occupancy_grid.data[i] == 100)
        {
            occupied_count++;
        }
    }

    std::cout << "Number of occupied points: " << occupied_count << std::endl;
}





















/**
void DepthCameraSubscriber::convert1DArrayTo2DImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_transformed_member_)
{
  std::vector<std::vector<unsigned char>> image(map_height, std::vector<unsigned char>(map_width, 0));

  for (const pcl::PointXYZ &point : cloud_transformed_member_->points)
  {
    Eigen::Vector3f position(point.x, point.y, point.z);
    std::array<float, 2> point_pixel_value = calculatePointPixelValue(position, origin_pixel, resolution);
    float x_px = static_cast<int>(point_pixel_value[0]);
    float y_px = static_cast<int>(point_pixel_value[1]);
    if (x_px >= 0 && x_px < map_width && y_px >= 0 && y_px < map_height)
    {
      image[y_px][x_px] = 255; // 255 for white
    }
  }
  std::ofstream outfile("src/depth_camera_subscriber/param/point_cloud_2D.pgm", std::ios::out | std::ios::binary);
  outfile << "P5\n"
          << map_width << " " << map_height << "\n255\n";

  // Write the pixel data
  for (int i = 0; i < map_height; i++)
  {
    for (int j = 0; j < map_width; j++)
    {
      outfile.put(static_cast<char>(image[i][j]));
    }
  }
  outfile.close();
  std::cout << "Point Cloud image saved as point_cloud_2D.pgm" << std::endl;
}
**/


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
