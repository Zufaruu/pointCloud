#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

sensor_msgs::PointCloud2 rotate_pcd(double rotation_angle_rad){
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zufaruu/widya_ws/src/pointCloud/pcd/horse.pcd", cloud) == -1)
    {
        ROS_ERROR("Couldn't read file");
    }

    // Create a rotation matrix
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(rotation_angle_rad, Eigen::Vector3f::UnitY()));

    // Rotate the point cloud
    pcl::PointCloud<pcl::PointXYZ> rotated_cloud;
    pcl::transformPointCloud(cloud, rotated_cloud, transform);

    // Convert the PCL point cloud to a ROS PointCloud2 message
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(rotated_cloud, ros_cloud);
    ros_cloud.header.stamp = ros::Time::now();
    ros_cloud.header.frame_id = "map";

    return ros_cloud;

}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcd_publisher");
  ros::NodeHandle nh;
  double rotation_angle, rotation_angle_rad;
  sensor_msgs::PointCloud2 ros_cloud;
  
  // Create a ROS publisher
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pcd", 1);

  ros::Rate loop_rate(4);
  while(nh.ok()){
    std::cout << "Enter the rotation angle (in degrees): ";
    std::cin >> rotation_angle;
    rotation_angle_rad = rotation_angle * M_PI / 180.0;

    ros_cloud = rotate_pcd(rotation_angle_rad);

    // Publish the PointCloud2 message
    pub.publish(ros_cloud);
    std::cout << "publishing pointcloud" << std::endl;
    
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}