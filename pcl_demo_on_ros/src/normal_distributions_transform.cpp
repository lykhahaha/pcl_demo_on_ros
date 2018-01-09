/*
使用正态分布变换进行配准的实验 。其中room_scan1.pcd  room_scan2.pcd这些点云包含同一房间360不同视角的扫描数据
*/
#include <iostream>//　标准输入输出
//pcd文件输入/输出
#include <pcl/io/pcd_io.h>//PCD文件打开存储类相关类头文件
//点/点云
#include <pcl/point_types.h>//点类型头文件
#include <pcl/point_cloud.h>//点云类相关头文件
#include <pcl/registration/ndt.h>////NDT(正态分布)配准类头文件正态分布配准类对应头文件
//滤波
#include <pcl/filters/approximate_voxel_grid.h>//滤波类对应头文件（使用体素网格过滤器处理的效果比较好）
//可视化
#include <pcl/visualization/pcl_visualizer.h>//可视化类相关头文件
#include <boost/thread/thread.hpp>
#include <ros/ros.h>  //ROS
int
main (int argc, char** argv)
{

  // ROS
  ros::init (argc, argv, "normal_distributions_transform");
  ros::NodeHandle nh;
  
  //加载房间的第一次扫描
  //加载房间的第一次扫描点云数据作为目标
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("~/room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file capture0001.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;
  //加载从新视角得到的房间的第二次扫描
  //加载从新视角得到的第二次扫描点云数据作为源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("~/room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;   
  //以上的代码加载了两个PCD文件得到共享指针，后续配准是完成对源点云到目标点云的参考坐标系的变换矩阵的估计，得到第二组点云变换到第一组点云坐标系下的变换矩阵
  //将输入的扫描点云数据过滤到原始尺寸的10%以提高匹配的速度，只对源点云进行滤波，减少其数据量，而目标点云不需要滤波处理
  //因为在NDT算法中在目标点云对应的体素网格数据结构的统计计算不使用单个点，而是使用包含在每个体素单元格中的点的统计数据
  //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;//类ApproximateVoxelGrid根据给定的点云形成三维体素栅格，并利用所有体素的中心点近似体素中包含的点集，这样完成下采样得到滤波结果，该类比较合适对海量点云数据在处理前进行压缩，提高算法效率
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;
  //初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  //设置依赖尺度NDT参数
  //为终止条件设置最小转换差异             
  ndt.setTransformationEpsilon (0.01);
  //为More-Thuente线搜索设置最大步长
  ndt.setStepSize (0.1);
  //设置NDT网格结构的分辨率（VoxelGridCovariance）
  //以上参数在使用房间尺寸比例下运算比较好，但是如果需要处理例如一个咖啡杯子的扫描之类更小的物体，需要对参数进行很大程度的缩小
  //设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
  ndt.setResolution (1.0);
  //设置匹配迭代的最大次数
  //添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
  ndt.setMaximumIterations (35);
  //设置要配准的点云
  ndt.setInputCloud (filtered_cloud);//源点云
  //设置点云配准目标
  ndt.setInputTarget (target_cloud);//目标点云
  //设置使用机器人测距法得到的初始对准估计结果
  //设置使用机器人测距法得到的粗略初始变换矩阵结果
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  //计算需要的刚体变换以便将输入的点云匹配到目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);
  //这个地方的output_cloud不能作为最终的源点云变换，因为上面对点云进行了滤波处理
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
  //使用创建的变换对未过滤的输入点云进行变换
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  //保存转换的输入点云
  //保存转换后的源点云作为最终的变换输出
  pcl::io::savePCDFileASCII ("~/room_scan2_transformed.pcd", *output_cloud);
  //初始化点云可视化界面
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);//设置背景颜色为黑色
  //对目标点云着色（红色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");
  //对转换后的目标点云着色（绿色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");
  //启动可视化
  viewer_final->addCoordinateSystem (1.0);//显示XYZ指示轴
  viewer_final->initCameraParameters ();//初始化摄像头参数
  //等待直到可视化窗口关闭。
  // ROS
  ros::Rate loop_rate(1);
  while (!viewer_final->wasStopped ()&&ros::ok())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    ros::spinOnce();
    loop_rate.sleep(); 
  }
  return (0);
}     
