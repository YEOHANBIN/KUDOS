#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

float lin_x,lin_y,ang_z = 0.0;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  lin_x = msg->axes[1];
  //lin_y = msg->axes[1];
  ang_z = msg->axes[0];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "w3_joy_node");
  ros::NodeHandle nh;

  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Subscriber sub = nh.subscribe("joy", 1000, chatterCallback);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x= lin_x*0.7;
    //msg.linear.y= lin_y*0.7;
    msg.angular.z= ang_z*0.7;
    printf("%lf,%lf\n",lin_x*0.7,ang_z*0.7);

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

