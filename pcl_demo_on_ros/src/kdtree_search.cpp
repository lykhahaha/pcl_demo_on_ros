#include <ros/ros.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>  
#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{ 
  ros::init (argc, argv, "kdtree_search");
  ros::NodeHandle nh;
  srand (time (NULL));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

/*
  // 把路径改为自己存放文件的路径
  pcl::PCDReader reader;
  reader.read ("/home/exbot/segmatch/laser_mapper/demonstration_files/kitti/table_scene_lms400.pcd", *cloud); 
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
*/      

  // 点云生成
  cloud->width =1000;
  cloud->height =1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i=0; i< cloud->points.size (); ++i)
  {
  cloud->points[i].x =1024.0f* rand () / (RAND_MAX +1.0f);
  cloud->points[i].y =1024.0f* rand () / (RAND_MAX +1.0f);
  cloud->points[i].z =1024.0f* rand () / (RAND_MAX +1.0f);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
  kdtree.setInputCloud (cloud);
  pcl::PointXYZ searchPoint;
  searchPoint.x=1024.0f* rand () / (RAND_MAX +1.0f);
  searchPoint.y=1024.0f* rand () / (RAND_MAX +1.0f);
  searchPoint.z=1024.0f* rand () / (RAND_MAX +1.0f);
  
  // k近邻搜索
  int K =10;
  std::vector<int>pointIdxNKNSearch(K);
  std::vector<float>pointNKNSquaredDistance(K);
  std::cout<<"K nearest neighbor search at ("<<searchPoint.x
  <<" "<<searchPoint.y
  <<" "<<searchPoint.z
  <<") with K="<< K <<std::endl;
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0 )
  {
  for (size_t i=0; i<pointIdxNKNSearch.size (); ++i)
  std::cout<<"    "<<   cloud->points[ pointIdxNKNSearch[i] ].x 
  <<" "<< cloud->points[pointIdxNKNSearch[i] ].y 
  <<" "<< cloud->points[pointIdxNKNSearch[i] ].z 
  <<" (squared distance: "<<pointNKNSquaredDistance[i] <<")"<<std::endl;
  }
   
  // 在半径r内搜索近邻
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius =256.0f* rand () / (RAND_MAX +1.0f);
  std::cout<<"Neighbors within radius search at ("<<searchPoint.x
  <<" "<<searchPoint.y
  <<" "<<searchPoint.z
  <<") with radius="<< radius <<std::endl;
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
  {
  for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
  std::cout<<"    "<<   cloud->points[ pointIdxRadiusSearch[i] ].x 
  <<" "<< cloud->points[pointIdxRadiusSearch[i] ].y 
  <<" "<< cloud->points[pointIdxRadiusSearch[i] ].z 
  <<" (squared distance: "<<pointRadiusSquaredDistance[i] <<")"<<std::endl;
  }     
       

  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {   
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  return (0);
}
