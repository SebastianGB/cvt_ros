#include <cvt_ros/StereoSubscriber.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

namespace cvt_ros {

    StereoSubscriber::StereoSubscriber() :
        _camSync( SyncPolicyType( 5 ) ),
        _camInfoSync( CInfoSyncPolicyType( 5 ) )
    {
        setupSubscribers();

        _camInfoSync.registerCallback( boost::bind( &StereoSubscriber::camInfoCallback, this, _1, _2 ) );
    }

//    StereoSubscriber::StereoSubscriber( const cvt::Matrix3f& calib ) :
//        _camSync( SyncPolicyType( 5 ) ),
//        _intrinsics( calib )
//    {
//        setupSubscribers();
//        // directly register callback for images
//        _camSync.registerCallback( boost::bind( &StereoSubscriber::imageMessageCallback, this, _1, _2 ) );
//    }

    StereoSubscriber::~StereoSubscriber()
    {
    }

    void StereoSubscriber::setupSubscribers()
    {
        ros::NodeHandle stereo( "/camera" );
        ros::NodeHandle nhLeft( stereo, "left" );
        ros::NodeHandle nhRight( stereo, "right" );

        _leftImageSub.subscribe( nhLeft, "image_raw", 2 );
        _rightImageSub.subscribe( nhRight, "image_raw", 2 );

        _leftCamInfoSub.subscribe( nhLeft, "camera_info", 1 );
        _rightCamInfoSub.subscribe( nhRight, "camera_info", 1 );

        ROS_INFO( "Subscribing to %s", _leftImageSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _rightImageSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _leftCamInfoSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _rightCamInfoSub.getTopic().c_str() );

        _camSync.connectInput( _leftImageSub, _rightImageSub );
        _camInfoSync.connectInput( _leftCamInfoSub, _rightCamInfoSub );
    }

    void StereoSubscriber::camInfoCallback( const sensor_msgs::CameraInfoConstPtr&,
                                            const sensor_msgs::CameraInfoConstPtr& )
    {
        ROS_INFO( "Camera Info received, now subscribing to images" );
//        for( size_t r = 0; r < 3; ++r ){
//            for( size_t c = 0; c < 3; ++c ){
//                _intrinsics[ r ][ c ] = camInfo->K[ 3 * r + c ];
//            }
//        }

        _camSync.registerCallback( boost::bind( &StereoSubscriber::imageMessageCallback, this, _1, _2 ) );

        _leftCamInfoSub.unsubscribe();
        _rightCamInfoSub.unsubscribe();
    }

    void StereoSubscriber::imageMessageCallback( const sensor_msgs::ImageConstPtr& leftImageMsg,
                                                 const sensor_msgs::ImageConstPtr& rightImageMsg )
    {
        _leftHeader = leftImageMsg->header;
        _rightHeader = rightImageMsg->header;

        try {
            cvt_ros_bridge::msg2Image( *leftImageMsg, _left );
            cvt_ros_bridge::msg2Image( *rightImageMsg, _right );
            imageCallback( _left, _right );
        } catch( cvt::Exception& e ) {
            ROS_ERROR( "Could not covert incoming image messages to cvt image: %s", e.what() );
            return;
        }
    }
}
