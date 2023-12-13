#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

void pointCB(const sensor_msgs::PointCloud2ConstPtr& num_msg){
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*num_msg, pcl_cloud);

    int point_count = pcl_cloud.size();

    ROS_INFO("num: %d", point_count);
}

int main(int argc, char** argv){
    ros::init(argc, argv,"points_counter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_roi", 1, pointCB);

    ros::spin();
}