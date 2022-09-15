#include <smb_highlevel_controller/SmbHighlevelController.hpp>


namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
    std::string topic_name;
    int queue_size;

    if (!nodeHandle_.getParam("scanner/topic_name", topic_name)) {
      ROS_ERROR("topic_name not defined!");
    }

    if (!nodeHandle_.getParam("scanner/queue_size", queue_size)) {
      ROS_ERROR("queue_size not defined!");
    }
//    p_control_gain = 0.5;
    if (!nodeHandle_.getParam("controller/p", p_control_gain)) {
          ROS_ERROR("p_control_gain not defined!");
        }

    subscriber_ = nodeHandle_.subscribe(topic_name, queue_size, &SmbHighlevelController::scanCallback, this);
    cmd_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    vis_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
  int size = msg.ranges.size();
  float min = msg.range_max;
  int min_index = 0;

  for (int i = 0; i < size; i++) {
    if (msg.ranges.at(i) < min && msg.ranges.at(i) > msg.range_min) {
      min = msg.ranges.at(i);
      min_index = i;
    }
  }

  float change_angle = msg.angle_min + (msg.angle_increment * min_index);

    geometry_msgs::Twist cmd;

    if (min > 1) {
      cmd.linear.x = 0.5;
      cmd.angular.z = p_control_gain * (change_angle);
    } else {
      cmd.linear.x = 0;
      cmd.angular.z = 0;
    }

    cmd_publisher_.publish(cmd);


    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();

    marker.ns = "pillar";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (min + 1) * cos(change_angle);
    marker.pose.position.y = -(min) * sin(change_angle);
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    vis_publisher_.publish(marker);

  ROS_INFO_STREAM("Minimum distance: " << min << "Minimum index: "<<min_index<< "Proprtional constant "<< change_angle);
}

} /* namespace */


