cmake_minimum_required(VERSION 2.8.3)
project(my_pcl_demo_on_ros)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
#find_package(PCL 1.8 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  include 
  ${catkin_INCLUDE_DIRS}
)
 

add_executable(pcl_load src/pcl_load.cpp)
add_executable(pcl_write src/pcl_write.cpp)
add_executable(project_inliers src/project_inliers.cpp)
add_executable(passthrough src/passthrough.cpp)
add_executable(voxelgrid src/voxel_grid.cpp)
add_executable(statistical_removal src/statistical_removal.cpp)
add_executable(extract_indices src/extract_indices.cpp)
add_executable(remove_outliers src/remove_outliers.cpp)
add_executable(planar_segmentation src/planar_segmentation.cpp)
add_executable(cylinder_segmentation src/cylinder_segmentation.cpp)
add_executable(kdtree_search src/kdtree_search.cpp)
add_executable(cluster_extraction src/cluster_extraction.cpp)
add_executable(normal_estimation src/normal_estimation.cpp)
add_executable(normal_estimation_using_integral_images src/normal_estimation_using_integral_images.cpp)
add_executable(iterative_closest_point src/iterative_closest_point.cpp)
add_executable(pairwise_incremental_registration src/pairwise_incremental_registration.cpp)
add_executable(normal_distributions_transform src/normal_distributions_transform.cpp)
add_executable(cloud_viewer_demo src/cloud_viewer_demo.cpp)
add_executable(pcl_visualizer src/pcl_visualizer.cpp)
add_executable(map_planar_segmentation src/map_planar_segmentation.cpp)
# add_executable(progressive src/progressive.cpp)

 
target_link_libraries(pcl_load ${catkin_LIBRARIES})
target_link_libraries(pcl_write ${catkin_LIBRARIES})
target_link_libraries(project_inliers ${catkin_LIBRARIES})
target_link_libraries(passthrough ${catkin_LIBRARIES})
target_link_libraries(voxelgrid ${catkin_LIBRARIES})
target_link_libraries(statistical_removal ${catkin_LIBRARIES})
target_link_libraries(extract_indices ${catkin_LIBRARIES})
target_link_libraries(remove_outliers ${catkin_LIBRARIES})
target_link_libraries(planar_segmentation ${catkin_LIBRARIES})
target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES})
target_link_libraries(kdtree_search ${catkin_LIBRARIES})
target_link_libraries(cluster_extraction ${catkin_LIBRARIES})
target_link_libraries(normal_estimation libvtkFiltering.so ${catkin_LIBRARIES})
target_link_libraries(normal_estimation_using_integral_images libvtkFiltering.so ${catkin_LIBRARIES})
target_link_libraries(iterative_closest_point ${catkin_LIBRARIES})
target_link_libraries(pairwise_incremental_registration libvtkFiltering.so ${catkin_LIBRARIES})
target_link_libraries(normal_distributions_transform libvtkFiltering.so ${catkin_LIBRARIES})
target_link_libraries(cloud_viewer_demo libvtkGraphics.so libvtkFiltering.so ${catkin_LIBRARIES})
target_link_libraries(pcl_visualizer libvtkFiltering.so libvtkGraphics.so ${catkin_LIBRARIES})
target_link_libraries(map_planar_segmentation libvtkFiltering.so libvtkGraphics.so ${catkin_LIBRARIES})
# target_link_libraries(progressive ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(narf_keypoint_extraction
  src/narf_keypoint_extraction.cpp
)
##add_dependencies(narf_keypoint_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(narf_keypoint_extraction libvtkFiltering.so
  ${catkin_LIBRARIES}
)



##add_executable(correspondence_grouping
##  src/correspondence_grouping.cpp
##)
##add_dependencies(correspondence_grouping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
##target_link_libraries(correspondence_grouping libvtkFiltering.so
##  ${catkin_LIBRARIES}
##)
