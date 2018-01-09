#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <ros/ros.h> 

typedef pcl::PointXYZ PointType; //定义别名

//参数的设置,全局变量
float angular_resolution = 0.5f;//角坐标分辨率
float support_size = 0.2f;//感兴趣点的尺寸（球面的直径）
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//坐标框架：相机框架（而不是激光框架）
bool setUnseenToMaxRange = false;//是否将所有不可见的点 看作 最大距离
bool rotation_invariant = true;

//命令帮助
//当用户输入命令行参数-h，打印帮助信息
void 
printUsage (const char* progName)
{
   std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
                                                                  "default "<<support_size<<")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            <<               " (default "<< (int)rotation_invariant<<")\n"
            << "-h           this help\n"
            << "\n\n";
}

void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)//setViewerPose
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}






int
main (int argc, char** argv)
{ 
  // ROS
  ros::init (argc, argv, "passthrough");
  ros::NodeHandle nh;
  
  //解析 命令行 参数
  // 设置参数检测
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
    cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);//以函数的方式初始化（0：相机框架；1：激光框架）
    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
    cout << "Setting support size to "<<support_size<<".\n";
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);


  // 打开一个磁盘中的.pcd文件  但是如果没有指定就会自动生成
  // 读取pcd文件；如果没有指定文件，就创建样本点
  pcl::PointCloud<PointType>::Ptr    point_cloud_ptr (new pcl::PointCloud<PointType>);//点云指针
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;//上面点云的别名

  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;//带视角的点构成的点云
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());//仿射变换
  //检查参数中是否有pcd格式文件名，返回参数向量中的索引号
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  //如果指定了pcd文件，读取pcd文件和对应的远距离pcd文件
  if (!pcd_filename_indices.empty ())   //检测是否有far_ranges.pcd
  {
    std::string filename = argv[pcd_filename_indices[0]];//文件名
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)//读取pcd文件
    {
      cerr << "Was not able to open file \""<<filename<<"\".\n"; //是否应该是std::cerr
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);//设置传感器的姿势
    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd"; //远距离文件名
    if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)//读取远距离pcd文件
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else//没有指定pcd文件，生成点云，并填充它
  {
    setUnseenToMaxRange = true;
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)   //如果没有打开的文件就生成一个矩形的点云
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point); //设置点云中点的坐标
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  }

  //从点云数据，创建深度图像
  // 从点云中建立生成深度图
  float noise_level = 0.0;    
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);//创建RangeImage对象（指针）
  pcl::RangeImage& range_image = *range_image_ptr;   //引用  
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);//从点云创建深度图像
  range_image.integrateFarRanges (far_ranges); //整合远距离点云
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();


  // 打开3D viewer并加入点云
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");//创建3D Viewer对象
  viewer.setBackgroundColor (1, 1, 1);//设置背景色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");//添加点云
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  
  // 显示深度图像（平面图，上面的3D显示）
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (range_image);
  
  // 提取NARF关键点
  //创建深度图像的边界提取器，用于提取NARF关键点
  pcl::RangeImageBorderExtractor range_image_border_extractor;    //申明深度图边缘提取器
  //创建NARF对象
  pcl::NarfKeypoint narf_keypoint_detector;                       //narf_keypoint_detector为点云对象

  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;    //获得特征提取的大小
  
  pcl::PointCloud<int> keypoint_indices;//用于存储关键点的索引
  narf_keypoint_detector.compute (keypoint_indices);//计算NARF关键点
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";
  
  
  //在range_image_widget中显示关键点
  // ----------------------------------------------
  // -----Show keypoints in range image widget-----
  // ----------------------------------------------
  //for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
                                  //keypoint_indices.points[i]/range_image.width);
                                  
                                  
  // 在3D图形窗口中显示关键点                                
  // 在3Dviewer显示提取的特征信息
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);//创建关键点指针
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;//引用
  keypoints.points.resize (keypoint_indices.points.size ()); //点云变形，无序
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  // 在关键点提取NARF描述子
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];      ///建立NARF关键点的索引向量，此矢量作为NARF特征计算的输入来使用

  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);//创建narf_descriptor对象。并给了此对象输入数据（特征点索引和深度像）
  narf_descriptor.getParameters ().support_size = support_size;//support_size确定计算描述子时考虑的区域大小
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;    //设置旋转不变的NARF描述子
  pcl::PointCloud<pcl::Narf36> narf_descriptors;               //创建Narf36的点类型输入点云对象并进行实际计算
  narf_descriptor.compute (narf_descriptors);                 //计算描述子
  cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "   //打印输出特征点的数目和提取描述子的数目
                      <<keypoint_indices.points.size ()<< " keypoints.\n";      
                             
  //ROS
  ros::Rate loop_rate(1);    
  
  // -----Main loop-----  
  //主循环函数
  while (ros::ok()&&!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();  // process GUI events
    viewer.spinOnce ();
    pcl_sleep(0.01);
    ros::spinOnce();  
    loop_rate.sleep();  
  }          
  return (0);
}




















