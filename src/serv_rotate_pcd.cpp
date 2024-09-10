#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <ros/service_server.h>
#include "pointCloud/RotatePCD.h"

// Define the service callback function
bool rotatePointCloudCallback(pointCloud::RotatePCD::Request &request, pointCloud::RotatePCD::Response &response) {
  // Load the PCD file
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(request.input_pcd_file, cloud) == -1) {
    ROS_ERROR("Couldn't read file");
    response.success = false;
    return true;
  }

  // Convert the angle to radians
  double rotation_angle_rad = request.rotation_angle * M_PI / 180.0;

  // Create a rotation matrix
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(rotation_angle_rad, Eigen::Vector3f::UnitY()));

  // Rotate the point cloud
  pcl::PointCloud<pcl::PointXYZ> rotated_cloud;
  pcl::transformPointCloud(cloud, rotated_cloud, transform);

  // Save the rotated point cloud to a PCD file
  if (pcl::io::savePCDFileASCII(request.output_pcd_file, rotated_cloud) == -1) {
    ROS_ERROR("Couldn't write file");
    response.success = false;
    return true;
  }
  
  response.success = true;
  return true;
}

int main(int argc, char** argv) {
//   Initialize ROS
  ros::init(argc, argv, "rotate_pcd_service");
  ros::NodeHandle nh;

  // Create a ROS service server
  ros::ServiceServer service = nh.advertiseService("rotate_pcd", rotatePointCloudCallback);

  // Spin to keep the node running
  ros::spin();

  return 0;
}