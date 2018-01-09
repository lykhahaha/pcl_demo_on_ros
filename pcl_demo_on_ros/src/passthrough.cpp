#include <ros/ros.h>  
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>  
#include <pcl/ModelCoefficients.h>//模型系数定义头文件
#include <pcl/filters/project_inliers.h>//投影滤波类头文件
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>  
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "passthrough");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  sensor_msgs::PointCloud2 output;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("~/drive27_target_map.pcd", *cloud);  

  // 创建滤波器对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);  
  pcl::toROSMsg(*cloud_filtered, output);
  output.header.frame_id = "odom";
  //把点云保存写到文件中
  pcl::PCDWriter writer;
  writer.write("~/passthroughs_map.pcd", *cloud_filtered, false);
  
  //发布点云
  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {  
    pcl_pub.publish(output);  
    ros::spinOnce();  
    loop_rate.sleep();  
  }  

  return (0);
}
