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
add_definitions( -DBOOST_SIGNALS_NO_DEPRECATION_WARNING )
set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O2" )

find_package( CVT_DEBUG REQUIRED )

include_directories(
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${CVT_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# generate dynamic reconfigure configs
generate_dynamic_reconfigure_options(
    cfg/ChameleonSettings.cfg
    cfg/RGBDCamera.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
<<<<<<< Updated upstream
catkin_package( INCLUDE_DIRS 
    include
    LIBRARIES cvt_ros
    CATKIN_DEPENDS roscpp image_transport 
    DEPENDS CVT )
=======
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cvt_ros
   CATKIN_DEPENDS cvt_ros_bridge roscpp #dynamic_reconfigure
   DEPENDS CVT_DEBUG
)
>>>>>>> Stashed changes


SET( CVT_ROS_LIB_SRCS
    include/cvt_ros/ImageSubscriber.h
    include/cvt_ros/RGBDSubscriber.h
    include/cvt_ros/StereoSubscriber.h
    include/cvt_ros/ROSSpinner.h
    src/lib/ImageSubscriber.cpp
    src/lib/RGBDSubscriber.cpp
    src/lib/StereoSubscriber.cpp )
SET( CVT_ROS_LIB_HDRS
    include/cvt_ros/ImageSubscriber.h
    include/cvt_ros/RGBDSubscriber.h
    include/cvt_ros/StereoSubscriber.h
    include/cvt_ros/ROSSpinner.h )
add_library( cvt_ros ${CVT_ROS_LIB_SRCS} ${CVT_ROS_LIB_HDRS} )
target_link_libraries( cvt_ros ${catkin_LIBRARIES} ${CVT_LIBRARIES} )

# node for synced point grey stereo camera
IF( ${CVT_HAS_DC1394} )
    add_executable( chameleon_stereo
        src/ChameleonStereo.h
        src/ChameleonStereo.cpp
        src/chameleon_stereo.cpp
    )
    target_link_libraries( chameleon_stereo ${catkin_LIBRARIES} ${CVT_LIBRARIES} )
    add_dependencies( chameleon_stereo ${PROJECT_NAME}_gencfg )
ENDIF()

# node for synced point grey stereo camera
add_executable( stereo_view src/stereo_view.cpp )
target_link_libraries( stereo_view ${catkin_LIBRARIES} ${CVT_LIBRARIES} ${PROJECT_NAME} )

# node for viewing images
add_executable( image_view src/image_view.cpp )
target_link_libraries( image_view ${catkin_LIBRARIES} ${CVT_LIBRARIES} ${PROJECT_NAME} )

# node for viewing images
add_executable( rgbd_view 
                src/rgbd_view.cpp 
                src/PoseView.h 
                src/PoseView.cpp 
                src/RGBDGui.h 
                src/RGBDGui.cpp )
target_link_libraries( rgbd_view ${catkin_LIBRARIES} ${CVT_LIBRARIES} ${PROJECT_NAME} )

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

IF( ${CVT_HAS_OPENNI2} )
    add_executable( rgbd_camera
                    src/RGBDCamera.h
                    src/RGBDCamera.cpp
                    src/rgbd_camera.cpp )
                target_link_libraries( rgbd_camera ${catkin_LIBRARIES} ${CVT_LIBRARIES} ${Boost_LIBRARIES} )
    add_dependencies( rgbd_camera ${PROJECT_NAME}_gencfg )
ENDIF()

# if messages are present, the dependent executables should set this dependency
# add_dependencies(cvt_ros_node cvt_ros_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS cvt_ros
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} )

install(TARGETS stereo_view image_view rgbd_view tsdf_meshing
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

IF( ${CVT_HAS_DC1394} )
    install(TARGETS chameleon_stereo
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
ENDIF()

IF( NOT APPLE )
    install(TARGETS v4l2_stereo
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ENDIF()

IF( ${CVT_HAS_OPENNI2} )
    install(TARGETS rgbd_camera
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
ENDIF()


## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

