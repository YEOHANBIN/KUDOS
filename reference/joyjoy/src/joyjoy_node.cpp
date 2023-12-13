#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

//*******************//
float lin_x = 0.0;
float ang_z = 0.0;
//*******************//

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)

{
  //*******************//
  lin_x = msg->axes[1];
  ang_z = msg->axes[0];
  //*******************//
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joyjoy_node");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = nh.subscribe("/joy", 1000, chatterCallback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
//****************************//
    msg.linear.x = lin_x;
    msg.angular.z = ang_z;
//****************************//
    cmd_vel_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
