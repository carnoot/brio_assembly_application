cmake_minimum_required(VERSION 2.8.3)
project(brio_assembly_vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x  -Wno-deprecated -DBT_USE_DOUBLE_PRECISION")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen
  geometry_msgs
  message_generation
  pcl_conversions
  #pcl_msgs
  #pcl_ros
  roscpp
  rospy
  std_msgs
)

set(VTK_DIR "/usr/include/vtk-5.8")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_service_files(
	FILES
        TrasformStamped.srv
	Finish_Movement.srv
)

generate_messages(   
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES brio_assembly_vision
#  CATKIN_DEPENDS eigen geometry_msgs message_generation pcl_conversions pcl_msgs pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/include/vtk-5.8)

#add_executable(Final src/Final.cpp)
#target_link_libraries(Final
#   ${catkin_LIBRARIES} ${VTK_LIBRARIES}
# )

