#include <iostream>
#include <ros/ros.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
int
main (int argc, char** argv)
{ 
  ros::init (argc, argv, "statistical_removal");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // 把路径改为自己存放文件的路径
  pcl::PCDReader reader;
  reader.read ("~/table_scene_lms400.pcd", *cloud); 
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
  // 创建滤波器对象
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("～/table_scene_lms400_inliers.pcd", *cloud_filtered, false);
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("～/table_scene_lms400_outliers.pcd", *cloud_filtered, false);
  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {   
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  return (0);
}
