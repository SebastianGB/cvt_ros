#include <ros/ros.h>
#include "ChameleonStereo.h"

using namespace cvt_ros;

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "chameleon" );
	ros::NodeHandle nh( "~" );

    try {
        cvt_ros::ChameleonStereo stereo;
        stereo.run( NULL );
        ros::spin();
    } catch( const cvt::Exception& e ){
        std::cout << e.what() << std::endl;
    }

	return 0;
}
