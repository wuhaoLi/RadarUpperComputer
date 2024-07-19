cmake_minimum_required(VERSION 3.0.2)
project(ros_can)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  std_msgs
  message_generation
)


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_can
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions pcl_ros message_runtime
)

include_directories(
 include
  "/home/pc/radar/SR75/CANFD/src/ros_can/include"   ##自己的.h文件地址
  ${catkin_INCLUDE_DIRS}
)

 add_executable(roscan src/main.cpp)
 add_executable(object src/filter_v.cpp)
 add_executable(plane src/plane_dis.cpp)



 add_dependencies(roscan ${PROJECT_NAME}_generate_messages_cpp)
 add_dependencies(object ${PROJECT_NAME}_generate_messages_cpp)
 add_dependencies(plane ${PROJECT_NAME}_generate_messages_cpp)

 target_link_libraries(roscan
   ${catkin_LIBRARIES}
   libcontrolcanfd.so
 )
TARGET_LINK_LIBRARIES(roscan pthread)
TARGET_LINK_LIBRARIES(roscan dl)
 target_link_libraries(object
 ${catkin_LIBRARIES}
)
target_link_libraries(plane
${catkin_LIBRARIES}
)
