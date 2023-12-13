#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist msg;
int i = 0;
int j = 0;

void stlin(){
  msg.linear.x=2;
  msg.angular.z=0;
}

void rot(){
  msg.linear.x=0;
  msg.angular.z=-2.5;
}

void star(){
  if(i==j){
    rot();
    i++;
  }
  else if(i>j){
    stlin();
    j++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "w2_hw2_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(0.5);
  while (ros::ok())
  {
    ROS_INFO("i=%d,h=%d",i,j);
    star();

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
