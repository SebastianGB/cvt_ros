#include <ros/ros.h>
#include <RGBDCamera.h>

#include <cvt/util/Exception.h>

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "rgbd_camera" );
    ros::NodeHandle nh( "~" );
    ROS_INFO( "Starting node: %s", nh.getNamespace().c_str() );

    try {
        cvt_ros::RGBDCamera cameraNode;
        cameraNode.spin();
    } catch ( const cvt::Exception& e ){
        ROS_ERROR_STREAM( e.what() );
    }

    return 0;
}
