#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#define VPoint velodyne_pointcloud::PointXYZIR
#define Point2 pcl::PointXYZI
#define PI 3.14159265359

using namespace std;

ros::Publisher pub1;

float theta_r = 180 * M_PI/ 180; // 라디안 각도로 회전 (180도 회전)

void input(const sensor_msgs::PointCloud2ConstPtr& scan)
{

    // Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan,*cloud); // ros msg 에서 pcl cloud 데이터로 변환

    //회전변환행렬
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)

    transform_1 (0,0) = std::cos (theta_r);
    transform_1 (0,1) = -sin(theta_r);
    transform_1 (1,0) = sin (theta_r);
    transform_1 (1,1) = std::cos (theta_r);
    //    (row, column)

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_p); 

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";
    pub1.publish(output);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "input");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, input); //front ouster
	pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_rotated", 100);
	ros::spin();
}