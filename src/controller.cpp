#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"


// fucntions of the system
void send_odometry(nav_msgs::Odometry message, Eigen::MatrixXd h, ros::Publisher odom_publisher);

int main(int argc, char **argv)
{
  // Name definition of the node
  ros::init(argc, argv, "Controller_cpp");
  ros::NodeHandle nh;
  // rate definition Node Frecuency
  ros::Rate loop_rate(50);

  // Publisher Definition
  ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("Mavic_pro/cmd_vel", 10);
  // Definition message type odometry
  nav_msgs::Odometry odometry_message;

  // Show message
  ROS_WARN_ONCE("Verify that the simulation is runnin.\nNode runnin..\n");
  printf("To finish this node, press ctrl+C\n");

  // Initial state initialization
  Eigen::MatrixXd h(4,1);
  while (ros::ok())
  {
    h = Eigen::MatrixXd::Random(4,1);
    send_odometry(odometry_message, h, odometry_publisher);
    loop_rate.sleep();
  }
  return 0;
}

void send_odometry(nav_msgs::Odometry message, Eigen::MatrixXd h, ros::Publisher odom_publisher)
{
  message.pose.pose.position.x = h(0,0);
  message.pose.pose.position.y = h(1,0);
  message.pose.pose.position.z = h(2,0);
  message.pose.pose.orientation.z = h(3,0);
  odom_publisher.publish(message);
}
