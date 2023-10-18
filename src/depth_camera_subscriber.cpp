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
    : Node("depth_camera_subscriber"), count_(0),  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{ // SystemDefaultsQoS()   , SensorDataQoS()
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


    cloud_in_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", rclcpp::SensorDataQoS());
    cloud_projected_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_projected", rclcpp::SensorDataQoS());
    cloud_filtered_distance_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered_distance_2d", rclcpp::SensorDataQoS());
    cloud_transformed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_transformed", rclcpp::SensorDataQoS());
    cloud_output_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_output", rclcpp::SensorDataQoS());


    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cloud_map_in", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    modified_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void DepthCameraSubscriber::combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
{
    if (pc_in != nullptr )
    {
        std::cout << "pc_in is loaded" << std::endl;
        sensor_msgs::msg::PointCloud2 output;
        output.header = pc_in->header;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
       // pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
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
        float max_distance_threshold = 1.5; // distance from camera

        for (const pcl::PointXYZ &point : cloud_projected->points)
        {
            float distance_2d = std::sqrt(point.x * point.x + point.y * point.y);

            if (distance_2d <= max_distance_threshold)
            {
                cloud_filtered->push_back(point);
            }
        }
         cloud_output = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        *cloud_output = *cloud_filtered;

        pcl::toROSMsg(*cloud_filtered, output); // convert PC to ROS message
        output.header = pc_in->header;
        cloud_filtered_distance_publisher_->publish(output);

        pcl::toROSMsg(*cloud_output, output); // convert PC to ROS message
        output.header = pc_in->header;
        cloud_output_publisher_->publish(output);

         }

        // process on the map

          if (map_in != nullptr)
        {
        std::cout << "map_in is loaded" << std::endl;
        saveMapAsPGM(map_in);

        map_output.header = map_in->header;
        map_output.info = map_in->info;
        map_output.data = map_in->data;
        map_publisher_->publish(map_output);
       map_data_set = true;

        }

    if(pc_in != nullptr && map_data_set)
    {
//(using ros2 run tf2_ros tf2_echo right_camera_depth_optical_frame map)
std::cout << "Applying rigid transformation to: filtered_cloud in camera frame -> map frame" << std::endl;
  /** Eigen::Matrix4f transform_matrix;
    transform_matrix << -0.613, 0.790, 0.019, -14.405,
                        -0.028, 0.003, -1.000, -0.027,
                        -0.790, -0.613, 0.020, 9.161,
                         0.000, 0.000, 0.000, 1.000;   **/

    PointCloudT::Ptr cloud_transformed(new PointCloudT);
    //pcl::transformPointCloud(*cloud_output, *cloud_transformed, transform_matrix);  


    /** Subtract PointCloud from OccupancyGrid
    for (size_t i = 0; i < cloud_transformed->points.size(); ++i)
    {
        int x = static_cast<int>((cloud_transformed->points[i].x - map_in->info.origin.position.x) / map_in->info.resolution);
        int y = static_cast<int>((cloud_transformed->points[i].y - map_in->info.origin.position.y) / map_in->info.resolution);
        int z = static_cast<int>((cloud_transformed->points[i].z - map_in->info.origin.position.z) / map_in->info.resolution);

        if (x >= 0 && y >= 0 && z >= 0 && x < map_in->info.width && y < map_in->info.height )
        {
            int index = y * map_in->info.width + x;  // calculates a 1D index from 2D coordinates (x, y) where x is the column index and y is the row index. map_in->info.width is the number of columns in the grid
            if (map_in->data[index] != 0)  // if cell at index(x,y) has non-zero value(occupied), set it to zero
            {
                map_in->data[index] = 0;
            }
        }
    }

    nav_msgs::msg::OccupancyGrid modified_map ;
    modified_map = *map_in;     
    modified_map_publisher_->publish(modified_map);**/


/**

   std::array<float, 2> origin = {map_in->info.origin.position.x, map_in->info.origin.position.y};
   float resolution = map_in->info.resolution;
   std::array<int, 2> image_size = {map_in->info.width, map_in->info.height};
     
    
   std::array<float, 2> map_origin_pixel = calculateMapOriginInPixel(origin, resolution, image_size);
    std::cout << "xmap-origin_pixel_x : " << map_origin_pixel[0] << " , map_origin_y : " << map_origin_pixel[1] << std::endl;


for (size_t i = 0; i < cloud_transformed->points.size(); ++i)
{
   std::array<float, 2> position = {cloud_transformed->points[i].x , cloud_transformed->points[i].y };
   std::array<float, 2> pixel_point = calculatePointPixelValue(position, map_origin_pixel, resolution);
   float x = pixel_point[0];
   float y = pixel_point[1];
}     **/

const std::string & current_frame_id = "right_camera_depth_optical_frame";
const std::string & destination_frame_id = "map";
is_transformed = findTransformation(current_frame_id, destination_frame_id, cloud_transformed, transform);
if(continue_callback )
{
std::cout<< "pointcloud is transformd to the map frame"<<std::endl;
}



    }









        

       

} // end of call back


bool DepthCameraSubscriber::findTransformation(std::string current_frame_id, std::string destination_frame_id, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform & transform)
{
    // Return of pointcloud is empty
   if(cloud_output->points.size() == 0 )
  {
    rclcpp::get_logger("pointcloud empty / Pointcloud not recieved.");
    return false;
  } 
 
try{
     
      transform =
        tf_buffer_->lookupTransform(
        destination_frame_id, current_frame_id,
        tf2::TimePointZero).transform; 
        

 std::scoped_lock lock(lock_);
        getTransformedCloud(cloud_output, cloud_transformed, transform);
cloud_transformed->header.frame_id = destination_frame_id;
return true;
}

catch(tf2::TransformException & ex ){
   RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
        "Exception in lookupTransform: %s",
        ex.what()
      ); 
      continue_callback = false;
}
if(!continue_callback)
  return false;


} 





void DepthCameraSubscriber::getTransformedCloud(pcl::PointCloud<PointT>::Ptr cloud_output, pcl::PointCloud<PointT>::Ptr cloud_transformed, geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaternion<float> rotation(transform.rotation.w, 
                            transform.rotation.x,
                            transform.rotation.y,
                            transform.rotation.z);

  Eigen::Vector3f translation (transform.translation.x, 
                          transform.translation.y,
                          transform.translation.z);

 Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    transform_matrix.block<3, 1>(0, 3) = translation;

    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << transform_matrix << std::endl;

 if (cloud_output->size() < 1)
  {
    cloud_transformed = cloud_output;
    
    return;
  }
  pcl::transformPointCloud (*cloud_output, *cloud_transformed, translation, rotation);

} 






std::array<float, 2> DepthCameraSubscriber::calculateMapOriginInPixel(const std::array<float, 2>& origin, float resolution, const std::array<int, 2>& image_size) {
    float x = -1 * origin[0] / resolution;
    float y = image_size[1]- (origin[1] / resolution);  // y axis is flipped

    return {x, y};
}


std::array<float, 2> DepthCameraSubscriber::calculatePointPixelValue(const std::array<float, 2>& position, const std::array<float, 2>& map_origin_pixel, float resolution) {
    float delta_x_px = position[0] / resolution;
    float delta_y_px = position[1] / resolution;

    float x = map_origin_pixel[0] + delta_x_px;
    float y = map_origin_pixel[1] - delta_y_px;  // y axis is flipped

    return {x, y};
}



void DepthCameraSubscriber::saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    std::ofstream pgm_file("src/depth_camera_subscriber/param/map.pgm", std::ios::out | std::ios::binary);

    if (pgm_file.is_open())
    {
        pgm_file << "P5\n";                                                     // Magic number for PGM binary format
        pgm_file << map_msg->info.width << " " << map_msg->info.height << "\n"; // Width and height
        pgm_file << "255\n";                                                    // Maximum gray value

        for (int i = 0; i < map_msg->info.width * map_msg->info.height; ++i)
        {
            uint8_t pixel_value = 255 - map_msg->data[i]; // Assuming map data is in the range [0, 100]
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



/**

         // remove point clouds that are more than 3 meters far from camera
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointXYZ camera_position(0, 0, 0);
          float max_distance_threshold = 2.5; // distance from camera
          for (const pcl::PointXYZ &point : cam_cloud_in->points)
          {
            // Euclidean distance (d = sqrt(x2-x1)² + (y2-y1)² + (z2-z1)²)
            float distance = pcl::geometry::distance(point, camera_position);
            if (distance <= max_distance_threshold)
            {
              cloud_filtered_1->push_back(point);
             // std::cout<< "distance    "<< distance<<std::endl;
            }
            else {
               // std::cout<< "distance2    "<< distance<<std::endl;
            }
          }

          sensor_msgs::msg::PointCloud2 output;
            rclcpp::Time t = rclcpp::Node::now();
            output.header.stamp = t;
            pcl::toROSMsg(*cloud_filtered_1, output); // convert PCL point cloud to ROS point cloud
            //output.header.frame_id = "cloud_show_frame";
            output.header = pc_in->header;
            publisher_->publish(output);

        std::cout << "\nfile with filter out 3 meters has  "
                    << " (" << cloud_filtered_1->size() << " points) " << std::endl;




         // find min and max points
          pcl::PointXYZ cam_minPt, cam_maxPt;
          pcl::getMinMax3D(*cam_cloud_in, cam_minPt, cam_maxPt);
          std::cout << "min point is " << cam_minPt << std::endl; // min point is (-1.39475,-2.5373,0)
          std::cout << "max point is " << cam_maxPt << std::endl; // max point is (6.07938,0.385508,13.197)

          // remove points which has y < 0 or y > maxPt.y
          PointCloudT::Ptr cloud_filtered_2(new PointCloudT);
          pcl::PassThrough<pcl::PointXYZ> pass_y;
          pass_y.setInputCloud(cloud_filtered_1);
          pass_y.setFilterFieldName("y");
          pass_y.setFilterLimits(-0.22,  cam_maxPt.y); // just keep the points between 0 og maxPt.y
          pass_y.filter(*cloud_filtered_2);

          //pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removed_cloud = DepthCameraSubscriber::removeGround(cloud_filtered_2, cam_minPt.y);
          std::cout << "\nfile with filter out regarding to height has  "
                    << " (" << cloud_filtered_2->size() << " points) " << std::endl;

            **/