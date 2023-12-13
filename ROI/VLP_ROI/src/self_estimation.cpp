#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define PI 3.14159265359

double ROI_theta(double x, double y);
int left_right(int x, int y);

using namespace std;

ros::Publisher pub1;
//ros::Publisher pub4;
//ros::Publisher pub5;
sensor_msgs::PointCloud2 output_ROI;
std_msgs::Int32 steer_mode;

float theta_r = 180 * M_PI/ 180;

double ROI_theta(double x, double y)
{
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = acos(x/r)*180/PI;
    return theta;
}

int left_right(int x, int y)
{
    if(x > y) return 1;
    else if(x < y) return 2;
    else return 0;
}

void input(const sensor_msgs::PointCloud2ConstPtr& scan)
{
	//1. Msg to pointcloud
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::fromROSMsg(*scan,*cloud);

    ////2. 회전변환행렬
    //Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    //transform_1 (0,0) = std::cos (theta_r);
    //transform_1 (0,1) = -sin(theta_r);
    //transform_1 (1,0) = sin (theta_r);
    //transform_1 (1,1) = std::cos (theta_r);
    //// Executing the transformation
  
    //pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::<pcl::PointXYZI> ());
    //pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1); 
    ////transform_1 의형식으로 cloud 를 transformed_cloud로 변환
  
    //pcl::PCLPointCloud2 cloud_p;
    //pcl::toPCLPointCloud2(*transformed_cloud, cloud_p);
    //sensor_msgs::PointCloud2 output;
    //pcl_conversions::fromPCL(cloud_p, output);
    //output.header.frame_id = "velodyne";
    //pub1.publish(output);
    
	//3. ROI 설정
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::fromROSMsg(*scan,laserCloudIn); 
    pcl::PCLPointCloud2 cloud_ROI;

    int left_size, right_size = 0;

    for(unsigned int j=0; j<laserCloudIn.points.size(); j++)
	{
        if(ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) < 45)
        {
            laserCloudIn.points[j].x = 0;
            laserCloudIn.points[j].y = 0;
            laserCloudIn.points[j].z = 0;
        }
        if(ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) > 135)
        {
            laserCloudIn.points[j].x = 0;
            laserCloudIn.points[j].y = 0;
            laserCloudIn.points[j].z = 0;
        }
        if(laserCloudIn.points[j].x < 0)
        {
            laserCloudIn.points[j].x = 0;
            laserCloudIn.points[j].y = 0;
            laserCloudIn.points[j].z = 0;
        }
        if(laserCloudIn.points[j].z < 0 || laserCloudIn.points[j].z > 0.5)
        {
            laserCloudIn.points[j].x = 0;
            laserCloudIn.points[j].y = 0;
            laserCloudIn.points[j].z = 0;
        }
        if(ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) > 45 && ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) < 90)
        {
            if(laserCloudIn.points[j].x != 0 && laserCloudIn.points[j].y != 0 && laserCloudIn.points[j].z != 0)
            {
                left_size += 1;
            }
        }
        if(ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) > 90 && ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) < 135)
        {
            if(laserCloudIn.points[j].x != 0 && laserCloudIn.points[j].y != 0 && laserCloudIn.points[j].z != 0)
            {
                right_size += 1;
            }
        }
    }

    pcl::toPCLPointCloud2(laserCloudIn, cloud_ROI);
    //sensor_msgs::PointCloud2 output_ROI; //출력할 방식인 PC2 선정 및 이름 output_ROI 정의
    pcl_conversions::fromPCL(cloud_ROI, output_ROI);
    output_ROI.header.frame_id = "velodyne";
    //pub4.publish(output_ROI);
    //std_msgs::Int32 steer_mode;
    steer_mode.data = left_right(left_size,right_size);
    //pub5.publish(steer_mode);


    if(left_right(left_size,right_size) == 0) ROS_INFO("Go straight");
    else if(left_right(left_size,right_size) == 1) ROS_INFO("Turn left");
    else ROS_INFO("Trun right");
    ROS_INFO("left: %d, right: %d",left_size,right_size);
    printf("====================================================================\n");


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, input);
    //pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_trans", 100);
	ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_roi", 100);
    ros::Publisher pub5 = nh.advertise<std_msgs::Int32> ("/steer_mode", 10);
	
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        //ros::Publisher pub4;
        //ros::Publisher pub5;

        pub4.publish(output_ROI);
        pub5.publish(steer_mode);

        ros::spinOnce();

        loop_rate.sleep();
    }
    //ros::spin();
}
