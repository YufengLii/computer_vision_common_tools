# Example CMakeLists.txt

cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
project( SLAM_FUSION_TOOLS )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.8 REQUIRED)

# Additional Include Directories
# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} )

# Preprocessor Definitions
# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )
add_definitions( -DPCL_NO_PRECOMPILE )

# Additional Library Directories
# [Linker]>[General]>[Additional Library Directories]
link_directories( ${PCL_LIBRARY_DIRS} )



# 可按下面的格式添加自己的源文件
add_executable( rgbd_sequence2pointclouds rgbd_sequence2pointclouds.cpp )
#add_custom_command( TARGET rgbd_sequence2pointclouds POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "rgbd_sequence2pointclouds" )
target_link_libraries( rgbd_sequence2pointclouds ${OpenCV_LIBS}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})



add_executable( alignclouds_use_camera_pose alignclouds_use_camera_pose.cpp )
#add_custom_command( TARGET alignclouds_use_camera_pose POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "alignclouds_use_camera_pose" )
target_link_libraries( alignclouds_use_camera_pose ${OpenCV_LIBS}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})




