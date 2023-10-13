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
    : Node("depth_camera_subscriber")
{
 point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/right_camera/depth/color/points", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr pc_in) {
        this->combined_callback(pc_in, nullptr); 
    });

map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr map_in) {
        this->combined_callback(nullptr, map_in); 
    });

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_projected", 10);
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cloud_map_in", 10);
}

void DepthCameraSubscriber::combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_in, const nav_msgs::msg::OccupancyGrid::SharedPtr map_in)
{
 if (pc_in !=nullptr){   

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_in, *cam_cloud_in);
      // Save the point clouds data as a PCD file
    pcl::io::savePCDFileASCII("src/depth_camera_subscriber/param/pc_save_XYZ.pcd", *cam_cloud_in);

    // print out th cam_cloud_in
    /** for (size_t i = 0; i < cam_cloud_in->points.size(); ++i)
     {
         float x = cam_cloud_in->points[i].x;
         float y = cam_cloud_in->points[i].y;
         float z = cam_cloud_in->points[i].z;
         std::cout << "Point " << i << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
     } **/

    // RCLCPP_INFO(this->get_logger(), "Data saved to pc_save_XYZ.pcd");


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
    pcl::io::savePCDFileASCII ("src/depth_camera_subscriber/param/cloud_projected.pcd", *cloud_projected);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_projected, output); // convert PC to ROS message
    output.header = pc_in->header;
    point_cloud_publisher_->publish(output);

 }

// working on map
if (map_in != nullptr){
saveMapAsPGM(map_in);
std::cout <<"map is saved"<< std::endl;
}

 




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


void DepthCameraSubscriber::saveMapAsPGM(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    std::ofstream pgm_file("src/depth_camera_subscriber/param/map_in2.pgm", std::ios::out | std::ios::binary);

    if (pgm_file.is_open()) {
        pgm_file << "P5\n"; // Magic number for PGM binary format
        pgm_file << map_msg->info.width << " " << map_msg->info.height << "\n"; // Width and height
        pgm_file << "255\n"; // Maximum gray value

        for (int i = 0; i < map_msg->info.width * map_msg->info.height; ++i) {
            uint8_t pixel_value = 255 - map_msg->data[i]; // Assuming map data is in the range [0, 100]
            pgm_file.write(reinterpret_cast<const char*>(&pixel_value), sizeof(pixel_value));
        }

        pgm_file.close();
        std::cout << "Map saved as map_in2.pgm" << std::endl;
    } else {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
    }
}

