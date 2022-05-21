#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <math.h>


// init system
void init_system(ros::Rate toc, int initialitation_time);
Eigen::MatrixXd time_vector(int number_samples, Eigen::MatrixXd t, double sample_time);
Eigen::MatrixXd jacobian_matrix(Eigen::MatrixXd h, Eigen::MatrixXd L);
Eigen::MatrixXd func_drone(Eigen::MatrixXd h, Eigen::MatrixXd v, Eigen::MatrixXd L);
Eigen::MatrixXd system_drone(Eigen::MatrixXd h, Eigen::MatrixXd v, Eigen::MatrixXd L, double ts);
void send_odometry(Eigen::MatrixXd h, nav_msgs::Odometry message, ros::Publisher odom_publisher);




int main(int argc, char **argv)
{
  // Name definition of the node
  ros::init(argc, argv, "Drone_simulation");
  ros::NodeHandle nh;

  // Loop rate
  double sample_time = 0.05;
  int hz = (int)1/sample_time;
  ros::Rate loop_rate(hz);

  // Publisher definition
  ros::Publisher odometry_publisher;
  odometry_publisher = nh.advertise<nav_msgs::Odometry>("Mavic_pro/odom", 10);

  // Definition odometry message
  nav_msgs::Odometry odometry_message;

  // Print message
  ROS_WARN_ONCE("Node for Drone Simulation.\n");
  init_system(loop_rate, hz);

  // vector time definition
  double final_time = 30;
  int number_samples = (final_time+sample_time)/sample_time;
  Eigen::MatrixXd t(1, (int)number_samples);
  t = Eigen::MatrixXd::Zero(1,(int)number_samples);
  t = time_vector(number_samples, t, sample_time);

  // Constand values of the robot
  double a, b, c;
  a = 0.1;
  b = 0.0;
  c = 0;
  Eigen::MatrixXd L(3,1);
  L(0, 0) = a;
  L(1, 0) = b;
  L(2, 0) = c;

  // Initial position Drone
  double x, y, z, psi;
  x = 0.0;
  y = 0.0;
  z = 0.0;
  psi = 90*((M_PI)/180);

  // Forward kinematics
  x = x + a*cos(psi) - b*sin(psi);
  y = y + a*sin(psi) + b*cos(psi);
  z = z + c;

  // Auxiliar variable dor index
  Eigen::ArrayXd index(4);
  index << 0,1,2,3;


  // Vector of system odometry
  Eigen::MatrixXd h(4, (int)(number_samples+1));
  h = Eigen::MatrixXd::Zero(4,(int)(number_samples+1));
  h(index, 0) << x,
                 y,
                 z,
                 psi;


  // Control vector definition
  Eigen::MatrixXd u_c(4,(int)(number_samples));
  u_c = Eigen::MatrixXd::Zero(4,(int)number_samples);

  // Instan time evolution definition
  double t_k = 0;
  double tic;
  double toc;
  double delta;
  int k = 0;

  // Send Initial values
  send_odometry(h(index, k), odometry_message, odometry_publisher);



  while (ros::ok())
  {
    // Time verification break
    tic = ros::Time::now().toSec();


    // Control loop

    u_c(index, k) << 0.1,
                     0.1,
                     1,
                     0;

    // Update values of the system
    h(index, k+1) = system_drone(h(index, k), u_c(index,k), L, sample_time);

    //send values odometry
    send_odometry(h(index, k+1), odometry_message, odometry_publisher);
    std::cout << h(index,k+1) << std::endl;
    std::cout << "-------------------------" << std::endl;


    // Time restriction similiat to tic toc matlab
    loop_rate.sleep();
    toc = ros::Time::now().toSec();
    delta = toc-tic;
    t_k = t_k+delta;


    // update index position
    k = k+1;

    // Break point
    if (k>=number_samples)
    {
      break;
    }
  }
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

// Function time vector defintion
Eigen::MatrixXd time_vector(int number_samples, Eigen::MatrixXd t, double sample_time){
  for (int i = 0; i < number_samples-1; ++i)
  {
    t(0,i+1) = t(i)+sample_time;
  }
  return t;
}

// JAcobian matrix function
Eigen::MatrixXd jacobian_matrix(Eigen::MatrixXd h, Eigen::MatrixXd L)
{
  // Constant values system
  double a, b, c;
  a = L(0, 0);
  b = L(1, 0);
  c = L(2, 0);

  // states of the system
  double x, y, z, psi;
  x = h(0, 0);
  y = h(1, 0);
  z = h(2, 0);
  psi = h(3, 0);

  // jacobian Matrix
  double j11, j12, j13, j14;
  double j21, j22, j23, j24;
  double j31, j32, j33, j34;
  double j41, j42, j43, j44;
  Eigen::MatrixXd J(4,4);

  // Definition of the elements of the matrix
  j11 = cos(psi);
  j12 = -sin(psi);
  j13 = 0;
  j14 = -(a*sin(psi)+b*cos(psi));

  j21 = sin(psi);
  j22 = cos(psi);
  j23 = 0;
  j24 = a*cos(psi)-b*sin(psi);

  j31 = 0;
  j32 = 0;
  j33 = 1;
  j34 = 0;

  j41 = 0;
  j42 = 0;
  j43 = 0;
  j44 = 1;

  J << j11, j12, j13, j14,
       j21, j22, j23, j24,
       j31, j32, j33, j34,
       j41, j42, j43, j44;

  return J;
}

// Drone function hp
Eigen::MatrixXd func_drone(Eigen::MatrixXd h, Eigen::MatrixXd v, Eigen::MatrixXd L)
{
  // Definition velocity vector
  Eigen::MatrixXd hp(4,1);
  // Jacobian Matrix Definition
  Eigen::MatrixXd J(4,4);

  J = jacobian_matrix(h, L);
  hp = J*v;
  return hp;

}

// Function numerical integration
Eigen::MatrixXd system_drone(Eigen::MatrixXd h, Eigen::MatrixXd v, Eigen::MatrixXd L, double ts)
{
  // Define variable dor runge kutta 4
  Eigen::MatrixXd k1(4,1);
  Eigen::MatrixXd k2(4,1);
  Eigen::MatrixXd k3(4,1);
  Eigen::MatrixXd k4(4,1);

  // New State
  Eigen::MatrixXd h_k(4,1);

  // Runge kuta Integrator
  k1 = func_drone(h, v, L);
  k2 = func_drone(h+(ts/2)*k1, v, L);
  k3 = func_drone(h+(ts/2)*k2, v, L);
  k4 = func_drone(h+(ts)*k3, v, L);

  // Integration
  h_k = h + (ts/6)*(k1+2*k2+2*k3+k4);

  return h_k;
}

void send_odometry(Eigen::MatrixXd h, nav_msgs::Odometry message, ros::Publisher odom_publisher)
{
  message.pose.pose.position.x = h(0,0);
  message.pose.pose.position.y = h(1,0);
  message.pose.pose.position.z = h(2,0);
  message.pose.pose.orientation.z = h(3,0);
  odom_publisher.publish(message);
}
