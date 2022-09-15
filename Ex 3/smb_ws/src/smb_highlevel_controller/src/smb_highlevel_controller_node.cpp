#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

#include "smb_highlevel_controller/SmbHighlevelController.hpp"



int main(int argc, char** argv) {
  ros::init(argc, argv, "smb_highlevel_controller");
  ros::NodeHandle nodeHandle("~");
  smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);


  ros::spin();
  return 0;
}
