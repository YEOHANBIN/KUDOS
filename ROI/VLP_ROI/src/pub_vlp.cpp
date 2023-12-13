#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

ros::Publisher pub1;
ros::Publisher pub2;

#define VPoint velodyne_pointcloud::PointXYZIR
#define Point2 pcl::PointXYZI

void input(const sensor_msgs::PointCloud2ConstPtr& scan)
{
// Msg to pointcloud
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg(*scan,*cloud);

//ROS로 다시 바꿔주기위해서 필요한 작업
pcl::PCLPointCloud2 cloud_p;

pcl::toPCLPointCloud2(*cloud, cloud_p);

sensor_msgs::PointCloud2 output;
pcl_conversions::fromPCL(cloud_p, output);


output.header.frame_id = "velodyne";
pub1.publish(output);
//ROS_INFO("published it.");


pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PCLPointCloud2 cloud_v;
pcl::VoxelGrid<pcl::PointXYZI> sor;

sor.setInputCloud (cloud);
sor.setLeafSize (0.5f, 0.5f, 0.5f);
sor.filter (*cloud_filtered); 
pcl::toPCLPointCloud2(*cloud_filtered, cloud_v);

sensor_msgs::PointCloud2 output_v;
pcl_conversions::fromPCL(cloud_v, output_v);
output_v.header.frame_id = "velodyne";
pub2.publish(output_v);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "input");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, input); //front ouster
	pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_voxelized", 100);
	ros::spin();
}
