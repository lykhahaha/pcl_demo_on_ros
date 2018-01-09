#include <ros/ros.h>  
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>//点类型头文件
#include <pcl/registration/icp.h>//ICP配准类头文件
#include <pcl/point_cloud.h>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  // ROS
  ros::init (argc, argv, "iterative_closest_point");
  ros::NodeHandle nh;
  
  // 创建对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
      
  // 填入点云数据
  cloud_in->width    = 5;//设置点云宽度
  cloud_in->height   = 1;//设置点云为无序点云
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i=0; i< cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x=1024*rand()/(RAND_MAX+1.0f);
    cloud_in->points[i].y=1024*rand()/(RAND_MAX+1.0f);
    cloud_in->points[i].z=1024*rand()/(RAND_MAX+1.0f);
  }
  
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
      
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<    
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<      
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;       
  std::cout << "size:" << cloud_out->points.size() << std::endl;  
  
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
      cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
      
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
      std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;    
  icp.setInputCloud(cloud_in);  
  icp.setInputTarget(cloud_out);      
  pcl::PointCloud<pcl::PointXYZ> Final;//存储经过配准源点云后的点云      
  icp.align(Final);//执行配准存储变换后的源点云到Final  
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<//打印配准相关输入信息
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;//打印输出最终估计的变换矩阵
    
  // ROS
  ros::Rate loop_rate(1);
    
  while (ros::ok())
  {
     ros::spinOnce();  
     loop_rate.sleep();  
  }
  
  return 0;
}

