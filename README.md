# How to Run this Package
- Create [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) and [build this package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
- There are 4 nodes which are able to execute. Use this commands to execute each nodes. Launch the `roscore` first and source the setup file of the workspace before running the node. <br>
  `rosrun pointCloud open_pcd_node` <br>
  `rosrun pointCloud rotate_pcd_node` <br>
  `rosrun pointCloud serv_rotate_pcd` <br>
  `rosrun pointCloud cli_rotate_pcd`<br>
- use this command to launch rviz to visualize the point cloud data. <br>
  `rosrun rviz rviz`

  
  
