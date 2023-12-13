#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher cmd_pub;
geometry_msgs::Twist cmd_msg;

void cmdCB(const sensor_msgs::ImuConstPtr& data)
{
    if(abs(data->angular_velocity.x) > 0.4)
    {
        if (data->angular_velocity.x < 0)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.linear.y = 0.0;
            cmd_msg.linear.z = 0.0;
            cmd_msg.angular.x = 0.0;
            cmd_msg.angular.y = 0.0;
            cmd_msg.angular.z = 0.15;
        }
        else
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.linear.y = 0.0;
            cmd_msg.linear.z = 0.0;
            cmd_msg.angular.x = 0.0;
            cmd_msg.angular.y = 0.0;
            cmd_msg.angular.z = -0.15;
        }
    }
    else
    {
        cmd_msg.linear.x = 0.3;
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = 0.0;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        cmd_msg.angular.z = 0.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "self_imu_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 100, cmdCB);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel",10);

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        cmd_pub.publish(cmd_msg);

        ros::spinOnce();
        
        loop_rate.sleep();
    }

}