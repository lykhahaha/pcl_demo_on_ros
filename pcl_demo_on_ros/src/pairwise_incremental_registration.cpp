#include <boost/make_shared.hpp>// boost指针相关头文件，共享指针
//点/点云
#include <pcl/point_types.h>//点类型头文件
#include <pcl/point_cloud.h>//点云类相关头文件
#include <pcl/point_representation.h>//点表示相关头文件
//pcd文件输入/输出
#include <pcl/io/pcd_io.h>//PCD文件打开存储类相关类头文件
//滤波
#include <pcl/filters/voxel_grid.h>//基于体素网格化的滤波类相关头文件
#include <pcl/filters/filter.h>//滤波相关头文件
//特征
#include <pcl/features/normal_3d.h>//法线特征相关头文件
//配准
#include <pcl/registration/icp.h>//ICP配准类头文件
#include <pcl/registration/icp_nl.h>//非线性icp类相关头文件
#include <pcl/registration/transforms.h>//变换矩阵类相关头文件
//可视化
#include <pcl/visualization/pcl_visualizer.h>//可视化类相关头文件
#include <ros/ros.h>  //ROS
#include <iostream>//　标准输入输出
//命名空间
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//定义类型的别名
//简单类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
  //这是一个辅助教程，因此我们可以负担全局变量
  //全局变量
  //创建可视化工具
  pcl::visualization::PCLVisualizer *p;
  //定义存储左右视点 
  int vp_1, vp_2;
  //处理点云的方便的结构定义
  //定义结构体，用于处理点云
  struct PCD
  {
    PointCloud::Ptr cloud; //点云指针
    std::string f_name;  //文件名
    PCD() : cloud (new PointCloud) {};  //初始化
  };

  struct PCDComparator  
  {
    bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }  

  };
  
  // 定义新的点表达方式< x, y, z, curvature > 坐标+曲率
  //以< x, y, z, curvature >形式定义一个新的点
  class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT> //继承关系
  {
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
    public:
    MyPointRepresentation ()  
    {
      //指定维数
      //定义尺寸值
      nr_dimensions_ = 4;
    }
  //重载函数copyToFloatArray，以定义自己的特征向量
  //覆盖copyToFloatArray方法来定义我们的特征矢量
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    //< x, y, z, curvature > 坐标xyz和曲率
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }  
  };

/** 在可视化窗口的第一视点显示源点云和目标点云
*
 */
  void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
  {
    p->removePointCloud ("vp1_target");//根据给定的ID，从屏幕中去除一个点云。参数是ID
    p->removePointCloud ("vp1_source");
    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);//目标点云绿色
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);//源点云红色    
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1); //加载点云
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
    PCL_INFO ("Press q to begin the registration.\n");//在命令行中显示提示信息
    p-> spin();
  }
/**在可视化窗口的第二视点显示源点云和目标点云
 *
 */
  void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
  {
  p->removePointCloud ("source");//根据给定的ID，从屏幕中去除一个点云。参数是ID
  p->removePointCloud ("target");
  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");//目标点云彩色句柄
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature"); //源点云彩色句柄
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);//加载点云
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
  p->spinOnce();
  } 
  
  /**加载一组我们想要匹配在一起的PCD文件， 读取一系列的PCD文件（希望配准的点云文件）
  * 参数argc是参数的数量 (pass from main ())，参数argc 参数的数量（来自main()）
  *参数 argv 实际的命令行参数 (pass from main ())， 参数argv 参数的列表（来自main()）
  *参数models点云数据集的合成矢量，参数models 点云数据集的结果向量
  */
  void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
  {
  std::string extension (".pcd");//声明并初始化string类型变量extension，表示文件后缀名
  //通过遍历文件名，读取pcd文件
  //假定第一个参数是实际测试模型
  for (int i = 1; i < argc; i++)//遍历所有的文件名（略过程序名）
  {
  std::string fname = std::string (argv[i]);
  // 至少需要5个字符长（因为.plot就有 5个字符）
  if (fname.size () <= extension.size ()) //文件名的长度是否符合要求 
     continue;
  std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);//将某操作(小写字母化)应用于指定范围
  //检查文件是否是pcd文件
  //检查参数是一个pcd文件  
  if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0) 
  {  
  //加载点云并保存在总体的模型列表中
  // 读取点云，并保存到models
  PCD m;
  m.f_name = argv[i];
  pcl::io::loadPCDFile (argv[i], *m.cloud);//读取点云数据
  //从点云中移除NAN点　
  //去除点云中的NaN点（xyz都是NaN）
  std::vector<int> indices;//保存去除的点的索引
  pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);//去除点云中的NaN点
  models.push_back (m);
  }
  }
  }  
/**匹配一对点云数据集并且返还结果
  *参数 cloud_src 是源点云
  *参数 cloud_tgt 是目标点云
  *参数output输出的配准结果的源点云
  *参数final_transform是在来源和目标之间的转换,参数final_transform 成对变换矩阵
  *参数downsample 是否下 采样
  */
  void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
  {
  //为了一致性和高速的下采样
  //注意：为了大数据集需要允许这项
  PointCloud::Ptr src (new PointCloud);//创建点云指针
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;//VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
  if (downsample)//下采样
  {
    grid.setLeafSize (0.05, 0.05, 0.05);//设置体元网格的叶子大小
    //下采样 源点云
    grid.setInputCloud (cloud_src);//设置输入点云
    grid.filter (*src);//下采样和滤波，并存储在src中
    //下采样 目标点云
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else//不下采样
  {
    src = cloud_src; //直接复制
    tgt = cloud_tgt;
  }
  //计算曲面法线和曲率
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals); //创建源点云指针（注意点的类型包含坐标和法向量）
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals); //创建目标点云指针（注意点的类型包含坐标和法向量）
  pcl::NormalEstimation<PointT, PointNormalT> norm_est; //该对象用于计算法向量
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());//创建kd树，用于计算法向量的搜索方法
  norm_est.setSearchMethod (tree);//设置搜索方法
  norm_est.setKSearch (30);//设置最近邻的数量
  norm_est.setInputCloud (src);//设置输入云
  norm_est.compute (*points_with_normals_src);//计算法向量，并存储在points_with_normals_src
  pcl::copyPointCloud (*src, *points_with_normals_src);//复制点云（坐标）到points_with_normals_src（包含坐标和法向量）
  norm_est.setInputCloud (tgt); //这3行计算目标点云的法向量，同上
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
  //
  //举例说明我们自定义点的表示（以上定义）
  //创建一个 自定义点表达方式的 实例
  MyPointRepresentation point_representation;
  //调整'curvature'尺寸权重以便使它和x, y, z平衡
  //加权曲率维度，以和坐标xyz保持平衡
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha); //设置缩放值（向量化点时使用）
  //
  // 配准
  //创建非线性ICP对象 并设置参数
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;//创建非线性ICP对象（ICP变体，使用Levenberg-Marquardt最优化
  reg.setTransformationEpsilon (1e-6);//设置容许的最大误差（迭代最优化）
  //将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
  //注意：根据你的数据集大小来调整
  //***** 注意：根据自己数据库的大小调节该参数
  reg.setMaxCorrespondenceDistance (0.1);//设置对应点之间的最大距离（0.1m）,在配准过程中，忽略大于该阈值的点  
  //设置点表示
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation)); //设置点表达
  //设置源点云和目标点云
  reg.setInputCloud (points_with_normals_src); //设置输入点云（待变换的点云）
  reg.setInputTarget (points_with_normals_tgt); //设置目标点云
  //
  //在一个循环中运行相同的最优化并且使结果可视化
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;//用于存储结果（坐标+法向量）
  reg.setMaximumIterations (2);//设置内部优化的迭代次数
  for (int i = 0; i < 30; ++i)//迭代
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);//命令行显示迭代的次数
    //为了可视化的目的保存点云
    //保存点云，用于可视化
    points_with_normals_src = reg_result;
    //估计
    reg.setInputCloud (points_with_normals_src); //重新设置输入点云（待变换的点云），因为经过上一次迭代，已经发生变换了
    reg.align (*reg_result);//对齐（配准）两个点云
		//在每一个迭代之间累积转换
    Ti = reg.getFinalTransformation () * Ti;//累积（每次迭代的）变换矩阵
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);//减小对应点之间的最大距离（上面设置过）
      prev = reg.getLastIncrementalTransformation ();//上一次变换的误差
    //可视化当前状态
    //显示当前配准状态，在窗口的右视区，简单的显示源点云和目标点云
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }
	//
  // 得到目标点云到源点云的变换
  targetToSource = Ti.inverse();//计算从目标点云到源点云的变换矩阵s
  //
  //把目标点云转换回源框架
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);//将目标点云 变换回到 源点云帧
  p->removePointCloud ("source");//根据给定的ID，从屏幕中去除一个点云。参数是ID
  p->removePointCloud ("target");
  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);//设置点云显示颜色，下同
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);//添加点云数据，下同
	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();
  p->removePointCloud ("source"); 
  p->removePointCloud ("target");
  //添加源点云到转换目标
  *output += *cloud_src;// 拼接点云图（的点）点数数目是两个点云的点数和
    final_transform = targetToSource;//最终的变换。目标点云到源点云的变换矩阵
 }    
  
 //****************  入口函数  ************************
 //主函数  
    
  
int
main (int argc, char** argv)
{
  // ROS
  ros::init (argc, argv, "pairwise_incremental_registration");
  ros::NodeHandle nh;
  
  // 加载数据
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;//模型
  loadData (argc, argv, data);//读取pcd文件数据，定义见上面  
  //检查用户输入
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);//语法
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    PCL_INFO ("Example: %s `rospack find pcl`/test/bun0.pcd `rospack find pcl`/test/bun4.pcd", argv[0]);
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());//显示读取了多少个点云文件
    //创建一个PCL可视化对象
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example"); //p是全局变量
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1); //创建左视区
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2); //创建右视区
  
    //创建点云指针和变换矩阵
	PointCloud::Ptr result (new PointCloud), source, target;//创建3个点云指针，分别用于结果，源点云和目标点云
    //全局变换矩阵，单位矩阵，成对变换
	//逗号表达式，先创建一个单位矩阵，然后将成对变换 赋给 全局变换
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    //遍历所有的点云文件
    for (size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1].cloud;//源点云
    target = data[i].cloud;//目标点云
    //添加可视化数据
    showCloudsLeft(source, target); //在左视区，简单的显示源点云和目标点云
    PointCloud::Ptr temp (new PointCloud);//创建临时点云指针
    //显示正在配准的点云文件名和各自的点数
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    //********************************************************
    //配准2个点云，函数定义见上面
    pairAlign (source, target, temp, pairTransform, true);
    //把当前的两两配对转换到全局变换
    //将当前的一对点云数据，变换到全局变换中。
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    //update the global transform更新全局变换
    //更新全局变换s 
    GlobalTransform = pairTransform * GlobalTransform;
    //********************************************************
    //保存配准对，转换到第一个点云框架中
    //保存成对的配准结果，变换到第一个点云帧
    std::stringstream ss; //这两句是生成文件名
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);//保存成对的配准结果
    
  }
   // ROS
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
     ros::spinOnce();  
     loop_rate.sleep();  
  }      
  
   
   return 0;
}      
