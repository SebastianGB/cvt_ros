#include <cvt_ros/StereoSubscriber.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

namespace cvt_ros {

    StereoSubscriber::StereoSubscriber() :
        _nh( "~" ),
        _camSync( SyncPolicyType( 5 ) ),
        _camInfoSync( CInfoSyncPolicyType( 5 ) )
    {
        setupSubscribers();

        _camInfoSync.registerCallback( boost::bind( &StereoSubscriber::camInfoCallback, this, _1, _2 ) );
    }

    StereoSubscriber::~StereoSubscriber()
    {
    }

    void StereoSubscriber::setupSubscribers()
    {
        std::string itopic( "image_raw" );
        _nh.param<std::string>( "image_raw", itopic, itopic );

        ros::NodeHandle stereo( "/camera" );
        ros::NodeHandle nhLeft( stereo, "left" );
        ros::NodeHandle nhRight( stereo, "right" );

        _leftImageSub.subscribe( nhLeft, itopic, 2 );
        _rightImageSub.subscribe( nhRight, itopic, 2 );

        _leftCamInfoSub.subscribe( nhLeft, "camera_info", 1 );
        _rightCamInfoSub.subscribe( nhRight, "camera_info", 1 );

        ROS_INFO( "Subscribing to %s", _leftImageSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _rightImageSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _leftCamInfoSub.getTopic().c_str() );
        ROS_INFO( "Subscribing to %s", _rightCamInfoSub.getTopic().c_str() );

        _camSync.connectInput( _leftImageSub, _rightImageSub );
        _camInfoSync.connectInput( _leftCamInfoSub, _rightCamInfoSub );
    }

    void StereoSubscriber::camInfoCallback( const sensor_msgs::CameraInfoConstPtr& infoLeft,
                                            const sensor_msgs::CameraInfoConstPtr& infoRight )
    {
        ROS_INFO( "Camera Info received, now subscribing to images" );
        ROS_INFO( "Left: %s", _leftImageSub.getTopic().c_str() );
        ROS_INFO( "Right: %s", _rightImageSub.getTopic().c_str() );
        try {
            bool isRectified;
            _nh.param<bool>( "is_rectified", isRectified, true );
            if( isRectified ){
                cvt_ros_bridge::camInfoRectToStereoCalib( _calib, infoLeft, infoRight );
            } else {
                cvt_ros_bridge::camInfoToStereoCalib( _calib, infoLeft, infoRight );
            }
        } catch ( std::exception& e ) {
            ROS_INFO( "Error Converting stereo calib: %s", e.what() );
        }

        _camSync.registerCallback( boost::bind( &StereoSubscriber::imageMessageCallback, this, _1, _2 ) );

        _leftCamInfoSub.unsubscribe();
        _rightCamInfoSub.unsubscribe();
    }

    void StereoSubscriber::imageMessageCallback( const sensor_msgs::ImageConstPtr& leftImageMsg,
                                                 const sensor_msgs::ImageConstPtr& rightImageMsg )
    {
        _leftHeader = leftImageMsg->header;
        _rightHeader= rightImageMsg->header;

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
