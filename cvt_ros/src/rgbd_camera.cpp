#include <ros/ros.h>

class RGBDNode {

};

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "rgbd_camera" );

    ros::NodeHandle nh( "~" );

    return 0;
}
