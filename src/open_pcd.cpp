#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <string>

sensor_msgs::PointCloud2 extract_pcd(std::string dir){
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(dir, cloud) == -1)
    {
        ROS_ERROR("Couldn't read file");
    }

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header.stamp = ros::Time::now();
    ros_cloud.header.frame_id = "map";

    return ros_cloud;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcd_publisher");
  ros::NodeHandle nh;

  sensor_msgs::PointCloud2 ros_cloud_1, ros_cloud_2, ros_cloud_3, ros_cloud_4;
  
//   ros_cloud_1 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/cat.pcd");
//   ros_cloud_2 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/horse.pcd");
//   ros_cloud_3 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/lioness.pcd");
//   ros_cloud_4 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/wolf.pcd");

  ros_cloud_1 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/horse_rotated_45.000000.pcd");
  ros_cloud_2 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/horse_rotated_90.000000.pcd");
  ros_cloud_3 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/horse_rotated_180.000000.pcd");
  ros_cloud_4 = extract_pcd("/home/zufaruu/widya_ws/src/pointCloud/pcd/horse_rotated_270.000000.pcd");
  

  // Create a ROS publisher
  ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcd_1", 1);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcd_2", 1);
  ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>("pcd_3", 1);
  ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2>("pcd_4", 1);
  
  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    // Publish the PointCloud2 message
    pub1.publish(ros_cloud_1);
    pub2.publish(ros_cloud_2);
    pub3.publish(ros_cloud_3);
    pub4.publish(ros_cloud_4);
    std::cout << "publishing pointcloud" << std::endl;
    
    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}