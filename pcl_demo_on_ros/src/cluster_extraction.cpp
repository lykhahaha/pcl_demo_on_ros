#include <pcl/ModelCoefficients.h>//模型系数定义头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>  
#include <iostream>


int
main (int argc, char** argv)
{
  // ROS
  ros::init (argc, argv, "cluser_extraction");
  ros::NodeHandle nh;
  
  // Read in the cloud data
  pcl::PCDReader reader;  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("~/table_scene_lms400.pcd", *cloud);  
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);          
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*  
  
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);                    
  
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
   
   // Segment the largest planar component from the remaining cloud
   seg.setInputCloud (cloud_filtered);     
   seg.segment (*inliers, *coefficients);   
   if (inliers->indices.size () == 0)     
   {  
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;   
    break;
   } 
   
   // Extract the planar inliers from the input cloud    
   pcl::ExtractIndices<pcl::PointXYZ> extract;  
   extract.setInputCloud (cloud_filtered);  
   extract.setIndices (inliers);   
   extract.setNegative (false);  
  
   // Write the planar inliers to disk  
   extract.filter (*cloud_plane);
   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
   
   // Remove the planar inliers, extract the rest  
   // 移去平面局内点，提取剩余点云    
   extract.setNegative (true);  
   extract.filter (*cloud_f);   
   cloud_filtered = cloud_f;  
  }
  
   // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);       
  tree->setInputCloud (cloud_filtered);  
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//欧式聚类对象
  ec.setClusterTolerance (0.02); // 设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize (100); // 设置一个聚类需要的最少的点数目为100   
  ec.setMaxClusterSize (25000); // 设置一个聚类需要的最大点数目为25000  
  ec.setSearchMethod (tree);  // 设置点云的搜索机制 
  ec.setInputCloud (cloud_filtered);  
  ec.extract (cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中
  
  //迭代访问点云索引cluster_indices,直到分割处所有聚类 
  int j = 0; 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
  {
  
    // 创建新的点云集cloud_cluster,将所有当前的聚类写入到点云数据集中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)    
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*   
      cloud_cluster->width = cloud_cluster->points.size ();  
      cloud_cluster->height = 1;  
      cloud_cluster->is_dense = true;  
  
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl; 
      std::stringstream ss;
      ss << "~/cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
    }      
  // ROS
  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {  

    ros::spinOnce();  
    loop_rate.sleep();  
  }  

  return (0);
  }
  }
