#include <ros/ros.h>
#include <ros/service_client.h>
#include <string.h> 
#include "pointCloud/RotatePCD.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "rotate_pcd_client");
  ros::NodeHandle nh;

  // Create a ROS service client
  ros::ServiceClient client = nh.serviceClient<pointCloud::RotatePCD>("rotate_pcd");
  double rotation_angle;
  pointCloud::RotatePCD::Request request;
  pointCloud::RotatePCD::Response response;

  ros::Rate loop_rate(4);
  while(nh.ok()){
    std::cout << "Enter the rotation angle (in degrees): ";
    std::cin >> rotation_angle;

    // Set the request message
    request.rotation_angle = rotation_angle; // rotate by 45 degrees
    std::string rotation_angle_str = std::to_string(rotation_angle);
    request.input_pcd_file = "/home/zufaruu/widya_ws/src/pointCloud/pcd/horse.pcd";
    request.output_pcd_file = "/home/zufaruu/widya_ws/src/pointCloud/pcd/horse_rotated_" + rotation_angle_str +  ".pcd";

    // Call the service
    if(client.call(request, response)) {
        if (response.success) {
        ROS_INFO("PCD file rotated successfully");
        } 
        else {
        ROS_ERROR("Failed to rotate PCD file");
        }
    } 
    else {
        ROS_ERROR("Failed to call service");
    }
  }
  return 0;
}