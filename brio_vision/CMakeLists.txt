cmake_minimum_required(VERSION 2.8.3)
project(brio_vision)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_ros pcl_conversions pcl_msgs cv_bridge
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  genmsg
)
set(VTK_DIR "/usr/include/vtk-5.8")
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
include(${VTK_USE_FILE})


add_message_files(
  FILES
  Data_Type.msg
  Container.msg
)
add_service_files(
	FILES
        Is_This_Easier.srv
)
generate_messages(   
  DEPENDENCIES
  std_msgs 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES brio_assembly_vision
#  CATKIN_DEPENDS eigen geometry_msgs message_generation pcl_conversions pcl_msgs pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
${opencv_INCLUDE_DIRS}
)


## Declare a cpp executable
# add_executable(opencv_test_kinect_node src/opencv_test_kinect_node.cpp)

add_executable(opencv_test src/opencv_test.cpp  )
target_link_libraries(opencv_test
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )

