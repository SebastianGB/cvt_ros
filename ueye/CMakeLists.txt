cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "/usr/share/cvt/cmake" )
FIND_PACKAGE( CVT REQUIRED )

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#For dynamic_reconfigure 
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_executable( ueye_cam
                         src/ueye_cam.cpp
                         src/UEyeNode.h )
target_link_libraries( ueye_cam ${CVT_LIBRARIES} )
	
rosbuild_add_executable( ueye_stereo src/ueye_stereo.cpp )
target_link_libraries( ueye_stereo ${CVT_LIBRARIES} )

rosbuild_add_library( ueye_stereo_nodelet src/nodelets/ueye_stereo.cpp )
target_link_libraries( ueye_stereo_nodelet ${CVT_LIBRARIES} )

rosbuild_add_library( rosbag_nodelet src/nodelets/bag_nodelet.cpp )
target_link_libraries( rosbag_nodelet ${CVT_LIBRARIES} )
rosbuild_link_boost( rosbag_nodelet date_time )
