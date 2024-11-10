cmake_minimum_required(VERSION 3.0.2)
project(sample_pcl_cropbox_filter_node)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)


catkin_package()


include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(cropbox_filter_node src/cropbox_filter_node.cpp)

target_link_libraries(cropbox_filter_node
  ${catkin_LIBRARIES}
)