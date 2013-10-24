#include "ChameleonStereo.h"
#include <ros/ros.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

#include <cvt/io/ChameleonStereo.h>

namespace cvt_ros {

static cvt::DC1394Camera::FeatureMode _valToMode( int val )
{
    cvt::DC1394Camera::FeatureMode mode;
    switch( val ){
    case ChameleonSettings_AUTO:
        mode = cvt::DC1394Camera::AUTO;
        break;
    case ChameleonSettings_MANUAL:
        mode = cvt::DC1394Camera::MANUAL;
        break;
    case ChameleonSettings_ONE_PUSH_AUTO:
        mode = cvt::DC1394Camera::ONE_SHOT;
        break;
    default:
        mode = cvt::DC1394Camera::AUTO;
        break;
    }
    return mode;
}


    ChameleonStereo::ChameleonStereo() :
        _stereo( 0 ),
        _rate( 30 )
    {
        ros::NodeHandle nh( "~" );
        std::string masterId( "49712223533866357" );
        std::string slaveId( "49712223533866360" );

        _cameraFrameId = nh.getNamespace();

		// default trigger pin: GPIO_0
		int triggerGPIO = 0;

		// default strobe pin: GPIO_1
		int strobeGPIO = 1;

		// user presets: 0 = factory; 1 = user_0; 2 = user_1
		int preset = 1;

        double fps = 30.0;
		nh.param<std::string>( "master_id", masterId, masterId );
		nh.param<std::string>( "slave_id", slaveId, slaveId );
		nh.param<int>( "trigger_pin", triggerGPIO, triggerGPIO );
		nh.param<int>( "strobe_pin", strobeGPIO, strobeGPIO );
        nh.param<double>( "fps", fps, fps );

		nh.param<int>( "user_presets", preset, preset );

		std::cout << "Master ID:   " << masterId << std::endl;
		std::cout << "Slave  ID:   " << slaveId << std::endl;
		std::cout << "Trigger Pin: " << triggerGPIO << std::endl;
		std::cout << "Strobe  Pin: " << strobeGPIO << std::endl;
		std::cout << "User Preset: " << preset << std::endl;
        std::cout << "Loop Rate: " << fps << "Hz" << std::endl;
        _rate = ros::Rate( fps );

        cvt::ChameleonStereo::Parameters params;
        params.leftId = cvt::String( masterId.c_str() );
        params.rightId = cvt::String( slaveId.c_str() );
        params.leftStrobePin = strobeGPIO;
        params.rightTriggerPin = triggerGPIO;

        _stereo = new cvt::ChameleonStereo( params );

		// create the camera topics:
		ros::NodeHandle masterNh( nh, "left" );
		ros::NodeHandle slaveNh( nh, "right" );
		image_transport::ImageTransport itMaster( masterNh );
		image_transport::ImageTransport itSlave( slaveNh );
		_pubMaster = itMaster.advertiseCamera( "image_raw", 1 );
		_pubSlave  = itSlave.advertiseCamera( "image_raw", 1 );

		// create the camera managers
		cvt::String resourcePath( "package://cvt_ros/resources/" );
		cvt::String camInfoName, saveUrl;

		camInfoName.sprintf( "chameleon_%s", masterId.c_str() );
		saveUrl.sprintf( "%s%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		ROS_INFO( "Creating Left CamerInfoManager: \n%s\n%s\n", camInfoName.c_str(), saveUrl.c_str() );
		_cInfoMaster = CameraInfoManPtr( new camera_info_manager::CameraInfoManager( masterNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _cInfoMaster->validateURL( saveUrl.c_str() ) ) {
			_cInfoMaster->loadCameraInfo( saveUrl.c_str() );
		}

		camInfoName.sprintf( "chameleon_%s", slaveId.c_str() );
		saveUrl.sprintf( "%s%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		ROS_INFO( "Creating Right CamerInfoManager: \n%s\n%s\n", camInfoName.c_str(), saveUrl.c_str() );
		_cInfoSlave = CameraInfoManPtr( new camera_info_manager::CameraInfoManager( slaveNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _cInfoSlave->validateURL( saveUrl.c_str() ) ) {
			_cInfoSlave->loadCameraInfo( saveUrl.c_str() );
		}

		//For dynamic_reconfigure
		_recCb = boost::bind( &ChameleonStereo::reconfigureCallback,this, _1, _2 );
		_server.setCallback( _recCb );
	}

	ChameleonStereo::~ChameleonStereo()
	{
		if( _stereo )
			delete _stereo;
	}

    void ChameleonStereo::run()
	{
		_rate.reset();

        ROS_INFO( "CAMERA LOOP START" );		
        try {
            while( ros::ok() ){
                triggerFrame();
                if( _stereo->nextFrame( 30 ) ){
                    publishFrames();
                }

                ros::spinOnce();
                _rate.sleep();
            }
        } catch( const cvt::Exception& e ){
            ROS_WARN( "Error: %s", e.what() );
        }
	}

	void ChameleonStereo::publishFrames()
	{
		// get the current frames
		const cvt::Image& masterImg = _stereo->left();
		const cvt::Image& slaveImg  = _stereo->right();

		_iter++;

		sensor_msgs::ImagePtr masterMessage( new sensor_msgs::Image );
		sensor_msgs::ImagePtr slaveMessage( new sensor_msgs::Image );

        // publish the frame
        fillMessage( *masterMessage, masterImg, _cameraFrameId );
        fillMessage( *slaveMessage, slaveImg, _cameraFrameId );

//      update caminfo stuff
        sensor_msgs::CameraInfoPtr masterInfo( new sensor_msgs::CameraInfo( _cInfoMaster->getCameraInfo() ) );
        sensor_msgs::CameraInfoPtr slaveInfo( new sensor_msgs::CameraInfo( _cInfoSlave->getCameraInfo() ) );

        masterInfo->header.frame_id = _cameraFrameId;
        masterInfo->header.stamp = _triggerTime;
        masterInfo->width = masterImg.width();
        masterInfo->height = masterImg.height();

        slaveInfo->header.frame_id = _cameraFrameId;
        slaveInfo->header.stamp = _triggerTime;
        slaveInfo->width = slaveImg.width();        
        slaveInfo->height = slaveImg.height();

        _pubMaster.publish( masterMessage, masterInfo );
        _pubSlave.publish( slaveMessage, slaveInfo );
    }

    void ChameleonStereo::fillMessage( sensor_msgs::Image& iMsg, const cvt::Image& image, const std::string& frameId ) const
    {        
        iMsg.header.seq = _iter;
        iMsg.header.frame_id = frameId;
        iMsg.header.stamp = _triggerTime;

        // image
        cvt_ros_bridge::image2RosMsg( image, iMsg );
    }

	void ChameleonStereo::triggerFrame()
	{		
		_stereo->trigger();
		_triggerTime = ros::Time::now();
	}

	void ChameleonStereo::setExposure( float value )
	{
		_stereo->setExposure( value );
	}

	void ChameleonStereo::setShutter( float value )
	{
        ROS_INFO( "Setting shutter value: %0.03fs", value / 1000.0f );
		_stereo->setShutter( value / 1000.0f );
	}

	void ChameleonStereo::setGain( float value )
	{
        ROS_INFO( "Setting gain value: %0.2fdb", value );
		_stereo->setGain( value );
	}    

	void ChameleonStereo::setAutoExposureMode( int val )
	{
        cvt::DC1394Camera::FeatureMode mode = _valToMode( val );
        bool v = false;
        if( mode == cvt::DC1394Camera::AUTO )
            v = true;
        _stereo->setAutoExposure( v );
        ROS_INFO( "Auto Exposure Mode: %d", v );
	}

	void ChameleonStereo::setAutoShutterMode( int val )
	{
        cvt::DC1394Camera::FeatureMode mode = _valToMode( val );
        bool v = false;
        if( mode == cvt::DC1394Camera::AUTO )
            v = true;
        _stereo->setAutoShutter( v );

        ROS_INFO( "Auto Shutter Mode: %d", v );
	}

	void ChameleonStereo::setAutoGainMode( int val )
	{
        cvt::DC1394Camera::FeatureMode mode = _valToMode( val );
        bool v = false;
        if( mode == cvt::DC1394Camera::AUTO )
            v = true;
        _stereo->setAutoGain( v );
        ROS_INFO( "Auto Shutter Mode: %d", v );
	}

    void ChameleonStereo::setWhiteBalanceMode( int val )
    {
        ROS_INFO( "TODO: setting White balance mode NOT IMPLEMENTED YET" );
        //_stereo->set
        //_slave->setWhiteBalanceMode( _valToMode( val ) );
    }

	void ChameleonStereo::reconfigureCallback( cvt_ros::ChameleonSettingsConfig& config, uint32_t /*level*/ )
    {      
        try {
            setShutter( config.shutter_value );
            setExposure( config.exposure_value );
            setGain( config.gain_value );
            _stereo->setWhiteBalance( config.white_balance_ub_value, config.white_balance_vr_value );
            setAutoGainMode( config.gain_mode );
            setAutoShutterMode( config.shutter_mode );
            setAutoExposureMode( config.exposure_mode );
            _stereo->enableAutoWhiteBalance( config.auto_wb );
            _stereo->enableAutoExposure( config.auto_exp );
            _stereo->enableAutoGain( config.auto_gain );
            _stereo->enableAutoShutter( config.auto_shutter );

// TODO: ROI feature needs more testing in CVT first
//            if( config.roi_pos_x  != _config.roi_pos_x ||
//                config.roi_pos_y  != _config.roi_pos_y ||
//                config.roi_width  != _config.roi_width ||
//                config.roi_height != _config.roi_height ){
//                cvt::Recti r( config.roi_pos_x, config.roi_pos_y, config.roi_width, config.roi_height );
//                //ROS_INFO( "IGNORING ROI CHANGE -> not implemented yet" );
//                //_stereo->setAreaOfInterest( r );
//            }

            if( config.left_packet_size != _config.left_packet_size ){
                _stereo->setPacketSize( config.left_packet_size, cvt::ChameleonStereo::LEFT );
                config.left_packet_size = _stereo->packetSize( cvt::ChameleonStereo::LEFT );
            }
            if( config.right_packet_size != _config.right_packet_size ){
                _stereo->setPacketSize( config.right_packet_size, cvt::ChameleonStereo::RIGHT );
                config.right_packet_size = _stereo->packetSize( cvt::ChameleonStereo::RIGHT );
            }


            if( config.trigger ){
                triggerFrame();
                config.trigger = false;
            }
            _config = config;
        } catch( const cvt::Exception& e ){
            ROS_WARN( "Exception in Reconfigure: %s", e.what() );
        }
    }

}
