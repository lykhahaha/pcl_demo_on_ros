#include <iostream>
#include <ros/ros.h> 
#include <pcl/ModelCoefficients.h>//模型系数
#include <pcl/io/pcd_io.h> //输入输出
#include <pcl/point_types.h> //点云（类型）
#include <sensor_msgs/PointCloud2.h>//点云（类型）  
#include <pcl/sample_consensus/method_types.h>//随机样本一致性算法 方法类型
#include <pcl/sample_consensus/model_types.h>//随机样本一致性算法 模型类型
#include <pcl/segmentation/sac_segmentation.h>//随机样本一致性算法 分割方法

int
main (int argc, char** argv)
{ 
  ros::init (argc, argv, "planar_segmentation");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


  // 把路径改为自己存放文件的路径
  pcl::PCDReader reader;
  reader.read ("~/table_scene_lms400.pcd", *cloud); 
  //reader.read ("~/test1121(2.0).pcd", *cloud);
  
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
  // 创建滤波器对象
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//存储输出的模型的系数
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//存储内点，使用的点
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;    //创建分割对象
  // Optional
  seg.setOptimizeCoefficients (true);    //可选设置
  // Mandatory
  //必须配置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
  seg.setModelType (pcl::SACMODEL_PLANE);//设置模型类型，检测平面
  seg.setMethodType (pcl::SAC_RANSAC);//设置方法【聚类或随机样本一致性】
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  //引发分割实现，并存储分割结果到点集合inliers及存储平面模型的系数coefficients
  seg.segment (*inliers, *coefficients);//分割操作

  if (inliers->indices.size () == 0)//根据内点数量，判断是否成功
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  //显示模型的系数
/* 　std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                        <<coefficients->values[1] << " "
                        <<coefficients->values[2] << " " 
                        <<coefficients->values[3] <<std::endl;

  std::cerr << "PointCloud after filtering: " << inliers->width * inliers->height 
       << " data points (" << pcl::getFieldsList (*inliers) << ").";

  pcl::PCDWriter writer;
  writer.write ("~/table_scene_lms400_plannerseg.pcd", *inliers, false);*/
  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {   
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  return (0);
}
