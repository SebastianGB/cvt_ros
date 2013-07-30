#include <ros/ros.h>
#include "V4L2Stereo.h"

using namespace cvt;
using namespace cvt_ros;

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "v4l2_stereo" );
	ros::NodeHandle nh( "~" );

    try {
        V4L2Stereo stereo;
        stereo.run( NULL );
        ros::spin();
    } catch( const cvt::Exception& e ){
        std::cout << e.what() << std::endl;
    }

	return 0;
}
