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

//static int _modeToVal( cvt::DC1394Camera::FeatureMode mode )
//{
//    int val;

//    switch( mode ){
//    case cvt::DC1394Camera::AUTO:
//        val = ChameleonSettings_AUTO;
//        break;
//    case cvt::DC1394Camera::MANUAL:
//        val = ChameleonSettings_MANUAL;
//        break;
//    case cvt::DC1394Camera::ONE_SHOT:
//        val = ChameleonSettings_ONE_PUSH_AUTO;
//        break;
//    default:
//        val = ChameleonSettings_AUTO;
//        break;
//    }
//    return val;
//}

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

		nh.param<std::string>( "master_id", masterId, masterId );
		nh.param<std::string>( "slave_id", slaveId, slaveId );
		nh.param<int>( "trigger_pin", triggerGPIO, triggerGPIO );
		nh.param<int>( "strobe_pin", strobeGPIO, strobeGPIO );

		std::cout << "Master ID:   " << masterId << std::endl;
		std::cout << "Slave  ID:   " << slaveId << std::endl;
		std::cout << "Trigger Pin: " << triggerGPIO << std::endl;
		std::cout << "Strobe  Pin: " << strobeGPIO << std::endl;

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
		_cInfoMaster = CameraInfoManPtr( new camera_info_manager::CameraInfoManager(
										masterNh, camInfoName.c_str(), saveUrl.c_str() ) );
		if ( _cInfoMaster->validateURL( saveUrl.c_str() ) ) {
			_cInfoMaster->loadCameraInfo( saveUrl.c_str() );
		}

		camInfoName.sprintf( "chameleon_%s", slaveId.c_str() );
		saveUrl.sprintf( "%s%s.yaml", resourcePath.c_str(), camInfoName.c_str() );
		_cInfoSlave = CameraInfoManPtr( new camera_info_manager::CameraInfoManager(
										slaveNh, camInfoName.c_str(), saveUrl.c_str() ) );
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

	void ChameleonStereo::execute( void* )
	{
		_rate.reset();

        ROS_INFO( "CAMERA LOOP START" );		
        try {
            while( ros::ok() )
            {
                {
                    cvt::ScopeLock lock( &_reconfigureMutex );
                    triggerFrame();
                    if( _stereo->nextFrame( 30 ) ){
                        publishFrames();
                    }
                }
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
		_stereo->setShutter( value / 1000.0f );
	}

	void ChameleonStereo::setGain( float value )
	{
		_stereo->setGain( value );
	}

    void ChameleonStereo::setWhiteBalance( uint32_t ubValue, uint32_t vrValue )
    {
        ROS_INFO( "TODO: NOT IMPLEMENTED YET" );
        //_stereo->setWhiteBalance( ubValue, vrValue );
    }

	void ChameleonStereo::setAutoExposureMode( int val )
	{
		_stereo->setAutoExposure( _valToMode( val ) );
	}

	void ChameleonStereo::setAutoShutterMode( int val )
	{
		_stereo->setAutoShutter( _valToMode( val ) );
	}

	void ChameleonStereo::setAutoGainMode( int val )
	{
		_stereo->setAutoGain( _valToMode( val ) );
	}

    void ChameleonStereo::setWhiteBalanceMode( int val )
    {
        ROS_INFO( "TODO: NOT IMPLEMENTED YET" );
        //_master->setWhiteBalanceMode( _valToMode( val ) );
        //_slave->setWhiteBalanceMode( _valToMode( val ) );
    }

	void ChameleonStereo::reconfigureCallback( cvt_ros::ChameleonSettingsConfig& config, uint32_t /*level*/ )
    {
        cvt::ScopeLock lock( &_reconfigureMutex );
        ROS_INFO( "Got reconfigure callback" );        

        try {
            if( ( config.shutter_value != _config.shutter_value ) ){
                setShutter( config.shutter_value );
            }
            if( ( config.exposure_value != _config.exposure_value ) ){                                    
                setExposure( config.exposure_value );
            }
            if( config.gain_value != _config.gain_value ) {                
                setGain( config.gain_value );
            }

            if( config.white_balance_ub_value != _config.white_balance_ub_value ||
                config.white_balance_vr_value != _config.white_balance_vr_value ) {                
                setWhiteBalance( config.white_balance_ub_value, config.white_balance_vr_value );
            }

            if( config.gain_mode != _config.gain_mode ){
                ROS_INFO( "Gain Mode change" );
                setAutoGainMode( config.gain_mode );
            }

            if( config.shutter_mode != _config.shutter_mode ){
                ROS_INFO( "Changing Shutter Mode" );
                setAutoShutterMode( config.shutter_mode );
            }

            if( config.exposure_mode != _config.exposure_mode ){
                ROS_INFO( "Changing Exposure Mode" );
                setAutoExposureMode( config.exposure_mode );
            }

            if( config.white_balance_mode != _config.white_balance_mode ){
                ROS_INFO( "Changing WB Mode" );
                setWhiteBalanceMode( config.white_balance_mode );
            }

//            if( _config.fps != config.fps ){
//                setFPS( config.fps );
//                //config.fps = fps();
//                ROS_INFO( "Setting FPS: %f", config.fps );
//                _rate = ros::Rate( config.fps + 5.0 /* backup */ );
//            }

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
