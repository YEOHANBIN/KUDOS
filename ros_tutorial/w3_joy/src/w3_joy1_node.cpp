#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

int lin_x=0;
int lin_y=0;
int lin_z=0;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  lin_x = msg->buttons[0];
  lin_y = msg->buttons[1];
  lin_z = msg->buttons[2];
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "w3_joy1_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  ros::Subscriber sub = nh.subscribe("joy", 1000, chatterCallback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    if(lin_x==1){
     msg.linear.x=2;
    }
    else if(lin_x==-1) {
     msg.linear.x=-2;
    }


     if (lin_y==1) {
     msg.linear.y=2;
    }
    else if(lin_y==-1){
      msg.linear.y=-2;
    }


     if (lin_z==1){
      msg.angular.z=2;
     }
     else if(lin_x=-1){
       msg.angular.z=-2;
     }

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
