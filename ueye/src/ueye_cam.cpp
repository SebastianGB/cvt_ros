#include <ros/ros.h>

#include "UEyeNode.h"

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "ueye_cam" );

    // camera
    size_t count = cvt::UEyeUsbCamera::count();
    if( count == 0 ){
        ROS_ERROR( "No UEye Camera foudn, please connect one and check the driver" );
        return 1;
    }

    UEyeNode ueyeNode;
    ros::spin();
    return 0;
}
