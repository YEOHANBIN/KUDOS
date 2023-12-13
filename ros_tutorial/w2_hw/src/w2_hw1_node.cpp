#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "w2_hw1_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;

    chatter_pub.publish(msg);

    ros::spin();

    loop_rate.sleep();
  }

  return 0;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]\n", msg->data.c_str());
}
