#include "V4L2Stereo.h"
#include <ros/ros.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

namespace cvt_ros {


    V4L2Stereo::V4L2Stereo() :
        _left( 0 ),
        _right( 0 ),
        _rate( 30 )
    {
        ros::NodeHandle nh( "~" );
        _cameraFrameId = nh.getNamespace();

		int leftIdx = 0;
		int rightIdx = 1;
		int fps = 30;
		int width = 640;
		int height = 480;
		nh.param<int>( "left_idx", leftIdx, leftIdx );
		nh.param<int>( "right_idx", rightIdx, rightIdx );
		nh.param<int>( "fps", fps, fps );
		nh.param<int>( "width", width, width );
		nh.param<int>( "height", height, height );

		std::cout << "Left Cam Idx:    " << leftIdx << std::endl;
		std::cout << "Right Cam Idx:   " << rightIdx << std::endl;
		std::cout << "FPS:   " << fps << std::endl;

		size_t nCams = cvt::V4L2Camera::count();
		if( nCams < 2 ){
			throw CVTException( "Not enough usb cams found" );
		}

		// create the camera topics:
		ros::NodeHandle leftNh( nh, "left" );
		ros::NodeHandle rightNh( nh, "right" );
		image_transport::ImageTransport itLeft( leftNh );
		image_transport::ImageTransport itRight( rightNh );
		_pubLeft	= itLeft.advertiseCamera( "image_raw", 1 );
		_pubRight	= itRight.advertiseCamera( "image_raw", 1 );

		// create the camera managers
		// TODO: useful names for the camera calibration
		cvt::String resourcePath( "package://cvt_ros/resources/" );
		cvt::String camInfoName, saveUrl;

		camInfoName.sprintf( "usb_%d", leftIdx );
		saveUrl.sprintf( "%s%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		_cInfoLeft = CameraInfoManPtr( new camera_info_manager::CameraInfoManager( leftNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _cInfoLeft->validateURL( saveUrl.c_str() ) ) {
			_cInfoLeft->loadCameraInfo( saveUrl.c_str() );
		}

		camInfoName.sprintf( "usb_%d", rightIdx );
		saveUrl.sprintf( "%s%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		_cInfoRight = CameraInfoManPtr( new camera_info_manager::CameraInfoManager( rightNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _cInfoRight->validateURL( saveUrl.c_str() ) ) {
			_cInfoRight->loadCameraInfo( saveUrl.c_str() );
		}

		cvt::CameraInfo leftCameraInfo, rightCameraInfo;
		cvt::V4L2Camera::cameraInfo( leftIdx, leftCameraInfo );
		cvt::CameraMode mode = leftCameraInfo.bestMatchingMode( cvt::IFormat::YUYV_UINT8, width, height, fps );
		_left = new cvt::V4L2Camera( leftIdx, mode );

		cvt::V4L2Camera::cameraInfo( rightIdx, rightCameraInfo );
		mode = rightCameraInfo.bestMatchingMode( cvt::IFormat::YUYV_UINT8, width, height, fps );
		_right = new cvt::V4L2Camera( rightIdx, mode );

		_left->startCapture();
		_right->startCapture();
	}

	V4L2Stereo::~V4L2Stereo()
	{
		if( _left )
			delete _left;
		if( _right )
			delete _right;
	}

	void V4L2Stereo::execute( void* )
	{
		_rate.reset();

        ROS_INFO( "CAMERA LOOP START" );		
        try {
            while( ros::ok() )
            {
                if( waitFrames( 10 ) ){
                    publishFrames();
                }
                _rate.sleep();
            }
        } catch( const cvt::Exception& e ){
            ROS_WARN( "Error: %s", e.what() );
        }
	}

	void V4L2Stereo::publishFrames()
	{
		// get the current frames
		const cvt::Image& leftImg = _left->frame();
		const cvt::Image& rightImg  = _right->frame();

		_iter++;

		sensor_msgs::ImagePtr leftMessage( new sensor_msgs::Image );
		sensor_msgs::ImagePtr rightMessage( new sensor_msgs::Image );

        // publish the frame
        fillMessage( *leftMessage, leftImg, _cameraFrameId );
        fillMessage( *rightMessage, rightImg, _cameraFrameId );

//      update caminfo stuff
        sensor_msgs::CameraInfoPtr leftInfo( new sensor_msgs::CameraInfo( _cInfoLeft->getCameraInfo() ) );
        sensor_msgs::CameraInfoPtr rightInfo( new sensor_msgs::CameraInfo( _cInfoRight->getCameraInfo() ) );

        leftInfo->header.frame_id = _cameraFrameId;
        leftInfo->header.stamp = _triggerTime;
        leftInfo->width = leftImg.width();
        leftInfo->height = leftImg.height();

        rightInfo->header.frame_id = _cameraFrameId;
        rightInfo->header.stamp = _triggerTime;
        rightInfo->width = rightImg.width();
        rightInfo->height = rightImg.height();

        _pubLeft.publish( leftMessage, leftInfo );
        _pubRight.publish( rightMessage, rightInfo );
    }

    void V4L2Stereo::fillMessage( sensor_msgs::Image& iMsg, const cvt::Image& image, const std::string& frameId ) const
    {        
        iMsg.header.seq = _iter;
        iMsg.header.frame_id = frameId;
        iMsg.header.stamp = _triggerTime;

        // image
        cvt_ros_bridge::image2RosMsg( image, iMsg );
    }

	bool V4L2Stereo::waitFrames( size_t val )
	{
		bool ret = true;
		ret &= _left->nextFrame( val );
        if( !ret ) // if we don't got a frame from master, we should not wait for slave
            return false;
        ret &= _right->nextFrame( val );

        double tL = _left->stamp();
        double tR = _right->stamp();
        std::cout << "DeltaT: " << tL - tR << std::endl;

		return ret;
	}

}
