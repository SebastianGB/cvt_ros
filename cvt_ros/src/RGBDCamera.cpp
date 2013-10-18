#include <RGBDCamera.h>
#include <dynamic_reconfigure/server.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

namespace cvt_ros
{

    RGBDCamera::RGBDCamera() :
        _cam( 0 ),
        _loopRate( 30.0 ),
        _frameRegistration( false ),
        _frameSync( false )
    {
        initCamera();

		// create the camera publishers
		ros::NodeHandle nh( "~" );
		ros::NodeHandle rgbNh( nh, "rgb" );
		ros::NodeHandle depthNh( nh, "depth" );
		image_transport::ImageTransport itRGB( rgbNh );
		image_transport::ImageTransport itDepth( depthNh );
		_pubRGB		= itRGB.advertiseCamera( "image_raw", 1 );
		_pubDepth	= itDepth.advertiseCamera( "image_raw", 1 );

		// create the camera managers
		cvt::String resourcePath( "package://cvt_ros/resources" );
		cvt::String camInfoName, saveUrl;
		camInfoName.sprintf( "rgb_%s", _cam->identifier().c_str() );
		saveUrl.sprintf( "%s/%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		ROS_INFO( "Creating RGB CameraInfoManager: \n\tInfo Name:%s\n\tUrl:%s\n", camInfoName.c_str(), saveUrl.c_str() );
		_rgbCamManager = CameraInfoManPtr( new camera_info_manager::CameraInfoManager( rgbNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _rgbCamManager->validateURL( saveUrl.c_str() ) ) {
			_rgbCamManager->loadCameraInfo( saveUrl.c_str() );
		}

		camInfoName.sprintf( "depth_%s", _cam->identifier().c_str() );
		saveUrl.sprintf( "%s/%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		ROS_INFO( "Creating Depth CameraInfoManager: \n\tInfo Name:%s\n\tUrl:%s\n", camInfoName.c_str(), saveUrl.c_str() );
		_depthCamManager = CameraInfoManPtr( new camera_info_manager::CameraInfoManager( depthNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _depthCamManager->validateURL( saveUrl.c_str() ) ) {
			_depthCamManager->loadCameraInfo( saveUrl.c_str() );
		}

		//For dynamic_reconfigure
		_reconfigureServer.setCallback( boost::bind( &RGBDCamera::configCb, this, _1, _2 ) );

		_rgbFrameId   = rgbNh.getNamespace();
		_depthFrameId = depthNh.getNamespace();
    }

    RGBDCamera::~RGBDCamera()
    {
        if( _cam ){
            _cam->stopCapture();
            delete _cam;
        }
    }

    void RGBDCamera::initCamera()
    {
        ros::NodeHandle nh( "~" );

        int camIdx, width, height;
        double fps;
        nh.param<int>( "cam_idx", camIdx, 0 );
        nh.param<int>( "width", width, 640 );
        nh.param<int>( "height", height, 480 );
        nh.param<double>( "fps", fps, 30.0 );

        size_t nCams = cvt::OpenNI2Camera::count();
        if( nCams < camIdx ){
            throw CVTException( "Camera Index out of bounds" );
        }

        cvt::CameraInfo cInfo;
        cvt::OpenNI2Camera::cameraInfo( camIdx, cInfo );
        cvt::CameraMode mode = cInfo.bestMatchingMode( cvt::IFormat::YUYV_UINT8, width, height, ( size_t )fps );
        _cam = new cvt::OpenNI2Camera( camIdx, mode );
        startCapture();

        ROS_INFO( "Created RGBD-Camera: [%zdx%zd] @ %zdHz", mode.width, mode.height, mode.fps );
        _loopRate = ros::Rate( fps );
    }

    void RGBDCamera::startCapture()
    {
        _cam->startCapture();
    }

    void RGBDCamera::stopCapture()
    {
        _cam->stopCapture();
    }

    void RGBDCamera::spin()
    {
        while( ros::ok() ){
            spinOnce();
            _loopRate.sleep();
        }
    }

    static void _fillMessage( sensor_msgs::Image& iMsg,
                              sensor_msgs::CameraInfo& info,
                              const cvt::Image& image,
                              const std::string& frameId,
                              const ros::Time& stamp )
    {
        iMsg.header.frame_id = frameId;
        iMsg.header.stamp = stamp;
        info.header = iMsg.header;
        cvt_ros_bridge::image2RosMsg( image, iMsg );
    }

    void RGBDCamera::spinOnce()
    {
        if( _cam->nextFrame( _loopRate.cycleTime().toSec() * 1000 ) ){
            const cvt::Image& rgb   = _cam->rgb();
            const cvt::Image& depth = _cam->depth();

            // TODO: use internal camera stamp
            ros::Time stamp = ros::Time::now();
            sensor_msgs::ImagePtr rgbMsg( new sensor_msgs::Image );
            sensor_msgs::ImagePtr depthMsg( new sensor_msgs::Image );
            sensor_msgs::CameraInfoPtr rgbInfo( new sensor_msgs::CameraInfo( _rgbCamManager->getCameraInfo() ) );
            sensor_msgs::CameraInfoPtr depthInfo( new sensor_msgs::CameraInfo( _depthCamManager->getCameraInfo() ) );

            _fillMessage( *rgbMsg, *rgbInfo, rgb, _rgbFrameId, stamp );
            _fillMessage( *depthMsg, *depthInfo, depth, _depthFrameId, stamp );

            _pubRGB.publish( rgbMsg, rgbInfo );
            _pubDepth.publish( depthMsg, depthInfo );
        }
        ros::spinOnce();
    }

    void RGBDCamera::configCb( cvt_ros::RGBDCameraConfig& config, uint32_t /*level*/ )
    {
        if( config.auto_exposure != _cam->autoExposure() ){
            _cam->setAutoExposure( config.auto_exposure );
            ROS_INFO( "Setting Auto-Exposure: %d", config.auto_exposure );
        }

        if( config.auto_white_balance != _cam->autoWhiteBalance() ){
            _cam->setAutoWhiteBalance( config.auto_white_balance );
            ROS_INFO( "Setting White-Balance: %d", config.auto_white_balance );
        }

        if( config.register_frames != _frameRegistration ){
            _frameRegistration = config.register_frames;
            _cam->setRegisterDepthToRGB( _frameRegistration );
            ROS_INFO( "Setting Frame Registration: %d", _frameRegistration );
        }

        if( config.sync_frames != _frameSync ){
            _frameSync = config.sync_frames;
            _cam->setSyncRGBDepth( _frameSync );
            ROS_INFO( "Setting Frame Sync: %d", _frameSync );
        }
    }

}
