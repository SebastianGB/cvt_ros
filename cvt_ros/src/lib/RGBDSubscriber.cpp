#include <cvt_ros/RGBDSubscriber.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

namespace cvt_ros {

    RGBDSubscriber::RGBDSubscriber() :
        _camSync( SyncPolicyType( 5 ) )
    {
        setupSubscribers();
        _camInfoSub.registerCallback( boost::bind( &RGBDSubscriber::camInfoCallback, this, _1 ) );
    }

    RGBDSubscriber::RGBDSubscriber( const cvt::Matrix3f& calib ) :
        _camSync( SyncPolicyType( 5 ) ),
        _intrinsics( calib )
    {
        setupSubscribers();
        // directly register callback for images
        _camSync.registerCallback( boost::bind( &RGBDSubscriber::imageMessageCallback, this, _1, _2 ) );
    }

    RGBDSubscriber::~RGBDSubscriber()
    {
    }

    void RGBDSubscriber::setupSubscribers()
    {
        ros::NodeHandle camNh( "/camera" );
        ros::NodeHandle nhDepth( camNh, "depth" );
        ros::NodeHandle nhRGB( camNh, "rgb" );
        _colorImageSub.subscribe( nhRGB, "image_raw", 2 );
        _depthImageSub.subscribe( nhDepth, "image_raw", 2 );
        _camInfoSub.subscribe( nhRGB, "camera_info", 1 );

        ROS_INFO( "Subscribing to %s", _colorImageSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _depthImageSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _camInfoSub.getTopic().c_str() );

        _camSync.connectInput( _colorImageSub, _depthImageSub );
    }

    void RGBDSubscriber::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camInfo )
    {
        ROS_INFO( "Camera Info received, now subscribing to images" );
        for( size_t r = 0; r < 3; ++r ){
            for( size_t c = 0; c < 3; ++c ){
                _intrinsics[ r ][ c ] = camInfo->K[ 3 * r + c ];
            }
        }

        _camSync.registerCallback( boost::bind( &RGBDSubscriber::imageMessageCallback, this, _1, _2 ) );
        _camInfoSub.unsubscribe();
    }

    void RGBDSubscriber::imageMessageCallback( const sensor_msgs::ImageConstPtr& colorImageMsg,
                                               const sensor_msgs::ImageConstPtr& depthImageMsg )
    {
        _rgbHeader = colorImageMsg->header;
        _depthHeader = depthImageMsg->header;

        try {
            cvt_ros_bridge::msg2Image( *colorImageMsg, _curRGB );
            cvt_ros_bridge::msg2Image( *depthImageMsg, _curDepth );
            imageCallback( _curRGB, _curDepth );
        } catch( cvt::Exception& e ) {
            ROS_ERROR( "Could not covert incoming image messages to cvt image: %s", e.what() );
            return;
        }
    }
}
