cmake_minimum_required(VERSION 2.8.3)
project(cvt_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    cvt_ros_bridge 
    roscpp
    dynamic_reconfigure
    image_transport
    camera_info_manager
    tf2_ros
)

## System dependencies are found with CMake's conventions
find_package( Boost REQUIRED COMPONENTS thread signals )
find_package( CVT REQUIRED )

include_directories(
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${CVT_INCLUDE_DIRS}
)

# generate dynamic reconfigure configs
generate_dynamic_reconfigure_options(
    cfg/ChameleonSettings.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cvt_ros
   CATKIN_DEPENDS cvt_ros_bridge roscpp #dynamic_reconfigure
   DEPENDS CVT
)


SET( CVT_ROS_LIB_FILES
    src/lib/RGBDSubscriber.cpp
)
add_library( cvt_ros ${CVT_ROS_LIB_FILES} )
target_link_libraries( cvt_ros ${catkin_LIBRARIES} ${CVT_LIBRARIES} )

# node for synced point grey stereo camera
add_executable( chameleon_stereo
    src/ChameleonStereo.h
    src/ChameleonStereo.cpp
    src/chameleon_stereo.cpp
)
target_link_libraries( chameleon_stereo ${catkin_LIBRARIES} ${CVT_LIBRARIES} )
add_dependencies( chameleon_stereo ${PROJECT_NAME}_gencfg )

add_executable( direct_vo
    src/direct_vo/direct_vo.cpp
    src/direct_vo/RGBDVONode.cpp
    src/direct_vo/RGBDVONode.h
    src/direct_vo/DirectVisualOdometry.h
)
target_link_libraries( direct_vo ${catkin_LIBRARIES} ${CVT_LIBRARIES} ${PROJECT_NAME} )
add_dependencies( direct_vo ${PROJECT_NAME}_gencfg )

add_executable( tsdf_meshing src/tsdf_meshing.cpp )
target_link_libraries( tsdf_meshing ${catkin_LIBRARIES} ${CVT_LIBRARIES} ${PROJECT_NAME} )
add_dependencies( tsdf_meshing ${PROJECT_NAME}_gencfg )

# node for Video for Linux Stereo
IF( NOT APPLE )
    add_executable( v4l2_stereo
        src/V4L2Stereo.h
        src/V4L2Stereo.cpp
        src/v4l2_stereo.cpp
    )
    target_link_libraries( v4l2_stereo ${catkin_LIBRARIES} ${CVT_LIBRARIES} )
    add_dependencies( v4l2_stereo ${PROJECT_NAME}_gencfg )
ENDIF()

# if messages are present, the dependent executables should set this dependency
# add_dependencies(cvt_ros_node cvt_ros_generate_messages_cpp)

#############
## Install ##
#############

# TODO: install cvt_ros library
# TODO: install camera nodes

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS cvt_ros cvt_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

