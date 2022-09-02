#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"

void scanCallback(const sensor_msgs::LaserScan& msg){

  ROS_INFO_STREAM("I heard " << msg.range_min);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "smb_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);

  std::string topic_name;
  int queue_size;
  if(!nodeHandle.getParam("/topic_name", topic_name)){
    ROS_ERROR("No topic_name parameter defined!");
  }
  if(!nodeHandle.getParam("/queue_size", queue_size)){
    ROS_ERROR("No queue size parameter defined!");
  }

  ros::Subscriber subscriber = nodeHandle.subscribe("/scan", queue_size, scanCallback);
  ros::spin();
  return 0;
}
