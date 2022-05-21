#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// Global variables definition
double x_odometry = 0;
double y_odometry = 0;
double z_odometry = 0;
double phi_odometry = 0;

double ul_odometry = 0;
double um_odometry = 0;
double un_odometry = 0;
double w_odometry = 0;


// init system
void init_system(ros::Rate toc, int initialitation_time);
Eigen::MatrixXf time_vector(int number_samples, Eigen::MatrixXf t, float sample_time);

// callback funtion definition
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_odometry = msg->pose.pose.position.x;
  y_odometry = msg->pose.pose.position.y;
  z_odometry = msg->pose.pose.position.z;
  phi_odometry = msg->pose.pose.orientation.z;

  ul_odometry = msg->twist.twist.linear.x;
  um_odometry = msg->twist.twist.linear.y;
  un_odometry = msg->twist.twist.linear.z;
  w_odometry = msg->twist.twist.angular.z;
}

int main(int argc, char **argv)
{
  // Name definition of the node
  ros::init(argc, argv, "Listner_odometry");
  ros::NodeHandle nh;

  // Loop rate
  float sample_time = 0.1;
  int hz = (int)1/sample_time;
  ros::Rate loop_rate(hz);

  // Subcriber Definition
  ros::Subscriber odometry_subcriber = nh.subscribe("/Mavic_pro/cmd_vel", 10, odometryCallback);

  // Print message
  ROS_WARN_ONCE("Check that simulation is running.\nNode runnin..\n");
  init_system(loop_rate, hz);

  // vector time definition
  float final_time = 1;
  int number_samples = (final_time+sample_time)/sample_time;
  Eigen::MatrixXf t(1, (int)number_samples);
  t = Eigen::MatrixXf::Zero(1,(int)number_samples);
  t = time_vector(number_samples, t, sample_time);

  // Vector of system odometry

  Eigen::MatrixXf h(4, (int)number_samples+1);
  h = Eigen::MatrixXf::Zero(4,(int)number_samples+1);

  // Control vector definition
  Eigen::MatrixXd u_c(4,1);
  u_c = Eigen::MatrixXd::Zero(4,1);
  Eigen::ArrayXd indices(4);
  indices << 0, 1, 2, 3;


  // Instan time evolution definition
  double t_k = 0;
  double tic;
  double toc;
  double delta;
  int k = 0;

  // Initial values of the system
  ros::spinOnce();
  h(0,0) = x_odometry;
  h(1,0) = y_odometry;
  h(2,0) = z_odometry;
  h(3,0) = phi_odometry;

  while (ros::ok())
  {
    // Time verification break
    tic = ros::Time::now().toSec();


    // Control loop
    std::cout << h(indices,k) << std::endl;
    std::cout << "-----------------" << std::endl;
    //u_c = controller_cinematic(h(Eigen::,k), h(Eigen::Dense::placeholders::all,k));

    // Update values from odometry
    ros::spinOnce();
    h(0,k+1) = x_odometry;
    h(1,k+1) = y_odometry;
    h(2,k+1) = z_odometry;
    h(3,k+1) = phi_odometry;



    // Time restriction similiat to tic toc matlab
    loop_rate.sleep();
    toc = ros::Time::now().toSec();
    delta = toc-tic;
    t_k = t_k+delta;

    // Update index position
    k = k+1;

    // Update values of the time vector
    if(k>=number_samples)
    {
      break;
    }
  }
  std::cout << h << std::endl;
  return 0;
}

// function initialitation
void init_system(ros::Rate toc, int initialitation_time)
{
  for (int k = 0; k<initialitation_time; k++)
  {
    ros::spinOnce();
    toc.sleep();
  }
}

// function of time initiaitation
Eigen::MatrixXf time_vector(int number_samples, Eigen::MatrixXf t, float sample_time){
  for (int i = 0; i < number_samples-1; ++i)
  {
    t(0,i+1) = t(i)+sample_time;
  }
  return t;
}

