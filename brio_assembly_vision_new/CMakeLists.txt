cmake_minimum_required(VERSION 2.8.3)
project(brio_assembly_vision_new)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x  -Wno-deprecated -DBT_USE_DOUBLE_PRECISION")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  eigen
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs 
 cv_bridge
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
        TrasformStamped.srv
        Is_This_Easier.srv
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
CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
 ${opencv_INCLUDE_DIRS}
 ${brio_assembly_vision_new_SOURCE_DIR}/include
)


#############################################################################
 ## Hack for QT Creator to include all relevant header and descriptor files ##
 #############################################################################
 #find all header and descriptor files
 execute_process(COMMAND find ${PROJECT_SOURCE_DIR}  -path "${PROJECT_SOURCE_DIR}/include/*" -name "*.h*" -or -name "*.cpp" OUTPUT_VARIABLE RS_FILES_STRING)
 #cmake string to list; has to be done because otherwise add_custom_target would handle    RS_FILES_STRING as one single file, which leads to an error
 string(REPLACE "\n" ";" RS_FILES_LIST ${RS_FILES_STRING})
 add_custom_target(project_additional_files_brio_new
   SOURCES
   ${RS_FILES_LIST}
 )

add_executable(Final_Updated src/Final_Updated.cpp src/DepthImageProjection.cpp)
target_link_libraries(Final_Updated
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${VTK_LIBRARIES}
 )
