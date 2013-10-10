#include <cvt_ros/ImageSubscriber.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

namespace cvt_ros {

    ImageSubscriber::ImageSubscriber() :
        _it( ros::NodeHandle() )
    {
        _sub = _it.subscribe( "image", 2, &ImageSubscriber::imageMessageCallback, this );
        ROS_INFO( "Subscribing to %s", _sub.getTopic().c_str() );
    }

    ImageSubscriber::~ImageSubscriber()
    {
    }

    void ImageSubscriber::imageMessageCallback( const sensor_msgs::ImageConstPtr& img )
    {
        _header = img->header;
        try {
            cvt_ros_bridge::msg2Image( *img, _cur );
            imageCallback( _cur );
        } catch( cvt::Exception& e ) {
            ROS_ERROR( "Could not covert incoming image messages to cvt image: %s", e.what() );
            return;
        }
    }
}
