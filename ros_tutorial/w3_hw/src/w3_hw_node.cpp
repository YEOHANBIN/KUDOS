#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

int button=0;
float x,y,theta=0.0;

void buttonCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  button = msg->buttons[0];
}

void poseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  x = msg->linear.x;
  y = msg->linear.z;
  theta = msg->angular.z;
  ROS_INFO("x: [%f]\ty: [%f]\ttheta: [%f]\n",x,y,theta);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "w3_hw_node");
  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 1000, poseCallback);

  ros::spin();

  return 0;
}
