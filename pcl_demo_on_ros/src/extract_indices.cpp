#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <ros/ros.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
int
main (int argc, char** argv)
{ 
  ros::init (argc, argv, "statistical_removal");
  ros::NodeHandle nh;
//sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_blob (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // 把路径改为自己存放文件的路径
  pcl::PCDReader reader;
  reader.read ("~/table_scene_lms400.pcd", *cloud_blob); 
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  // 创建滤波器对象:使用叶大小为1cm的下采样
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);
  // 转化为模板点云
//pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered_blob->width * cloud_filtered_blob->height << " data points." << std::endl;
  // 将下采样后的数据存入磁盘    
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("~/table_scene_lms400_downsampled.pcd", *cloud_filtered_blob, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // 创建分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // 可选
  seg.setOptimizeCoefficients (true);
  // 必选
  //必须配置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
  seg.setModelType (pcl::SACMODEL_PLANE);//设置模型类型，检测平面
  seg.setMethodType (pcl::SAC_RANSAC);//设置方法【聚类或随机样本一致性】
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  // 创建滤波器对象
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 0, nr_points = (int) cloud_filtered_blob->points.size ();
  // 当还有30%原始点云数据时
  while (cloud_filtered_blob->points.size () > 0.3 * nr_points)
  {
    // 从余下的点云中分割最大平面组成部分
    seg.setInputCloud (cloud_filtered_blob);
    //引发分割实现，并存储分割结果到点集合inliers及存储平面模型的系数coefficients
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // 分离内层
    extract.setInputCloud (cloud_filtered_blob);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
    // 创建滤波器对象
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered_blob.swap (cloud_f);
    i++;
  }
      
  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {   
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  return (0);
}
