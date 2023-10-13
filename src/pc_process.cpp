#include "depth_camera_subscriber.hpp"
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>

#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/conversions.h>
#include <functional>
#include <utility>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

// pcl::visualization::CloudViewer viewer ("Test Cloud Viewer");

pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, float minYValue)
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
}

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // Change color (here, green)
  viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
  viewer.setBackgroundColor(255, 255, 255); // Set background color to white
  viewer.addCoordinateSystem(1.0);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // Set point size
  double time_limit = 5.0;                                                                                   // second
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

    if (elapsed_time.count() >= time_limit)
    {
      break;
    }
  }
}

int countZeroPoints(const pcl::PointCloud<PointT>::Ptr& cloud) {
    int zeroPointsCount = 0;
    
    for (const PointT& point : cloud->points) {
        if (point.x == 0 && point.y == 0 && point.z == 0) {
            zeroPointsCount++;
        }
    }

    return zeroPointsCount;
}





int main(int argc, char *argv[])
{
  // Defining the point cloud
  PointCloudT::Ptr cam_cloud_in(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  PointCloudT::Ptr cloud_projected(new PointCloudT);

  // Load the point clouds
  if (pcl::io::loadPCDFile("pc_save_XYZ.pcd", *cam_cloud_in) < 0)
  {
    PCL_ERROR("Error loading clouds %s./n", argv[1]);
    return (-1);
  }

  std::cout << "\nLoaded file has  "
            << " (" << cam_cloud_in->size() << " points) " << std::endl;

  int zeroPointsCount = countZeroPoints(cam_cloud_in);
std::cout << "Number of points with coordinates (0,0,0): " << zeroPointsCount << std::endl;


  //visualizePointCloud(cam_cloud_in);

   //pcl::visualization::CloudViewer viewer_2 ("Test cloud Viewer");
     //viewer_2.showCloud(cam_cloud_in);


  // remove point clouds that are more than 3 meters far from camera
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ camera_position(0, 0, 0);
  float max_distance_threshold = 3.0; // distance from camera
  for (const pcl::PointXYZ &point : cam_cloud_in->points)
  {
    // Euclidean distance (d = sqrt(x2-x1)² + (y2-y1)² + (z2-z1)²)
    float distance = pcl::geometry::distance(point, camera_position);
    if (distance <= max_distance_threshold)
    {
      cloud_filtered_1->push_back(point);
    }
  }

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
  pass_y.setFilterLimits(0, cam_maxPt.y); // just keep the points between 0 og maxPt.y
  pass_y.filter(*cloud_filtered_2);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removed_cloud = removeGround(cloud_filtered_2, cam_minPt.y);
  std::cout << "\nfile with filter out regarding to height has  "
            << " (" << cloud_filtered_2->size() << " points) " << std::endl;

  // visualizePointCloud(ground_removed_cloud);

  /**

     // convert 3D point Clouds to 2D image(2D plane)  for using in object detection
     PointCloudT::Ptr cloud_projected(new PointCloudT);
     //  Create a set of planar coefficients with X=Z=0,Y=1 (Ax + By + Cz + D = 0)  a plane particular to y axis which is the ground plane
     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
     coefficients->values.resize(4);
     coefficients->values[0] = 0;
     coefficients->values[1] = 1.0;
     coefficients->values[2] = 0;
     coefficients->values[3] = 0;

     // Create the filtering object
     pcl::ProjectInliers<pcl::PointXYZ> projection;
     projection.setModelType(pcl::SACMODEL_PLANE); // filter type is a plane
     projection.setInputCloud(cloud_filtered);
     projection.setModelCoefficients(coefficients); //  set the plane coef.
     projection.filter(*cloud_projected);


    //pcl::visualization::PCLVisualizer viewer ("ICP demo");
    //viewer_2.showCloud(cloud_projected);


     sensor_msgs::msg::PointCloud2 output;
     pcl::toROSMsg(*cloud_projected, output); // convert PC to ROS message
     publisher_->publish(output);  **/


  return 0;
}
