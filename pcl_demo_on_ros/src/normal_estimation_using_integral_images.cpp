#include <ros/ros.h>  
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>  

int
main (int argc, char** argv)
{
  // ROS
  ros::init (argc, argv, "normal_estimation_using_integral_images");
  ros::NodeHandle nh;

  // 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("~/table_scene_mug_stereo_textured.pcd", *cloud);
  
  // 估计法线   
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);//设置估计方法
  //COVARIANCE_MATRIX模式从具体某个点的局部邻域的协方差矩阵创建9个积分图，来计算这个点的法线  
  //AVERAGE_3D_GRADIENT模式创建了6个积分图来计算水平和垂直方向的平滑后的三维梯度，并使用两个梯度间的向量积计算法线 
  //AVERAGE_DEPTH_CHANGE模式只创建了一个单一的积分图，并从平均深度变化计算法线     
  ne.setMaxDepthChangeFactor(0.02f);//最大深度变化系数  
  ne.setNormalSmoothingSize(10.0f);//优化法线方向时考虑邻域大小 
  ne.setInputCloud(cloud);//输入点云，必须为有序点云  
  ne.compute(*normals);//执行法线估计存储结果到normals
  
  // 法线可视化
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
  
  // ROS
  ros::Rate loop_rate(1);
  while (!viewer.wasStopped()&&ros::ok())
  {  
     viewer.spinOnce ();
     ros::spinOnce();
     loop_rate.sleep(); 
  }
  return 0;  
  
}    
