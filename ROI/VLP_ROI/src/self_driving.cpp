#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher cmd_pub;

void modeCB(const std_msgs::Int32ConstPtr& mode)
{
    geometry_msgs::Twist driving;

    if (mode->data == 0)
    {
        driving.linear.x = 1;
        driving.linear.y = 0;
        driving.linear.z = 0;
        driving.angular.x = 0;
        driving.angular.y = 0;
        driving.angular.z = 0;
    }
    else if (mode->data == 1)
    {
        driving.linear.x = 0;
        driving.linear.y = 0;
        driving.linear.z = 0;
        driving.angular.x = 0;
        driving.angular.y = 0;
        driving.angular.z = 0.5;
    }
    else
    {
        driving.linear.x = 0;
        driving.linear.y = 0;
        driving.linear.z = 0;
        driving.angular.x = 0;
        driving.angular.y = 0;
        driving.angular.z = -0.5;
    }

    cmd_pub.publish(driving);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "driving");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Int32>("/steer_mode",10,modeCB);
    cmd_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);

    ros::spin();
}