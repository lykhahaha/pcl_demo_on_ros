#include <ros/ros.h>  
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>  
#include <pcl/ModelCoefficients.h>//模型系数定义头文件
#include <pcl/filters/project_inliers.h>//投影滤波类头文件
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>  
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "project_inliers");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  sensor_msgs::PointCloud2 output;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ> cloud_projected
  pcl::io::loadPCDFile ("~/drive27_target_map.pcd", *cloud);  
  //打开点云文件

  // Create a set of planar coefficients with X=Y=0,Z=1
  //定义模型系数对象，并填充对应的数据//定义模型系数对象，并填充对应的数据
/*  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
*/
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = 0;
  coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;//创建投影滤波对象
  proj.setModelType (pcl::SACMODEL_PLANE);//设置对象对应的投影模型
  proj.setInputCloud (cloud);//设置输入点云
  proj.setModelCoefficients (coefficients);//设置模型对应的系数
  proj.filter (*cloud_projected);//执行投影滤波存储结果cloud_projected
  pcl::toROSMsg(*cloud_projected, output);
  output.header.frame_id = "odom";
  //把点云保存写到文件中
  pcl::PCDWriter writer;
  //writer.write("original_points.pcd", *cloud, false);
  writer.write("/home/exbot/segmatch/laser_mapper/demonstration_files/kitti/project_inliers_xymap.pcd", *cloud_projected, false);
  
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
