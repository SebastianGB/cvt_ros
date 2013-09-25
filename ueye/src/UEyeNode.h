#include <ros/ros.h>

#include <cvt/io/UEyeUsbCamera.h>
#include <cvt/util/Time.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/fill_image.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <ueye/paramsConfig.h>

#include <cvt/io/UEyeUsbCamera.h>

class UEyeNode
{
	public:
        UEyeNode() :
            _imageNode( "~" ),			
            _it( _imageNode ),
            _pub( _it.advertiseCamera( "image_raw", 2 ) ),
            _cam( 0 ),
            _cInfo( 0 )
        {
            cvt::String resourcePath( "package://ueye/resources/" );

            int cam_num = 0;
            double fps = 60;

            _imageNode.param<int>( "cam_num", cam_num, cam_num );
            _imageNode.param<double>( "fps", fps, fps );

            size_t nCams = cvt::UEyeUsbCamera::count();
            cam_num = cvt::Math::min<int>( cam_num, nCams-1 );

            cvt::CameraInfo cInfo;
            cvt::UEyeUsbCamera::cameraInfo( cam_num, cInfo );
            cvt::CameraMode mode = cInfo.bestMatchingMode( cvt::IFormat::BAYER_RGGB_UINT8, 640, 480, fps );
            _cam = new cvt::UEyeUsbCamera( cInfo.index(), mode );
            _cam->startCapture();

            cvt::String saveUrl;
            cvt::String name;
            name.sprintf( "ueye_%s.yaml", _cam->identifier().c_str() );
            saveUrl = resourcePath + name;
            _cInfo = new camera_info_manager::CameraInfoManager( _imageNode, name.c_str(), saveUrl.c_str() );

            if( _cInfo->validateURL( saveUrl.c_str() ) ){
                _cInfo->loadCameraInfo( saveUrl.c_str() );
			}

			// advertise image topic
            _msg.header.frame_id = "/ueye_cam";
            _camInfoMsg.header.frame_id = _msg.header.frame_id;

            updateCamInfo();

			//For dynamic_reconfigure
            f = boost::bind(&UEyeNode::reconfigureCallback,this, _1, _2);
		 	server.setCallback(f);

            // create the Timer:
            _timer = _imageNode.createTimer( ros::Duration( 1.0 / fps ), &UEyeNode::timer_callback, this );
		}

        ~UEyeNode()
		{
            if( _cInfo )
                delete _cInfo;
            if( _cam ){
                _cam->stopCapture();
                delete _cam;
            }
		}

        void updateCamInfo()
		{
            if( _cam->width() != _camInfoMsg.width ||
                _cam->height() != _camInfoMsg.height ){
                _camInfoMsg.width = _cam->width();
                _camInfoMsg.height = _cam->height();
			}
		}

		//Work in progress
		void reconfigureCallback(ueye::paramsConfig &config, uint32_t level) {

            if( config.hor_mirror != _config.hor_mirror ){
                _cam->setHorizontalMirror( config.hor_mirror );
            }

            if( config.ver_mirror != _config.ver_mirror ){
                _cam->setVerticalMirror( config.ver_mirror );
            }

            if( config.pixel_clock != _config.pixel_clock ){
                _cam->setPixelClock( config.pixel_clock );
            }

            if( config.auto_sensor_shutter != _config.auto_sensor_shutter ){
                _cam->setAutoSensorShutter( config.auto_sensor_shutter );
            }

            if( config.auto_shutter != _config.auto_shutter ){
                _cam->setAutoShutter( config.auto_shutter );
            }

            if( config.framerate != _config.framerate ){
                _cam->setFramerate( config.framerate );
            }

            if( config.exposure_time != _config.exposure_time ){
                _cam->setExposureTime( config.exposure_time );
            }

            _config = config;
		}

	private:
        cvt::UEyeUsbCamera*                 _cam;
		ros::NodeHandle 					_imageNode;

        image_transport::ImageTransport		_it;
        image_transport::CameraPublisher	_pub;
        sensor_msgs::Image _msg;
        sensor_msgs::CameraInfo _camInfoMsg;

        camera_info_manager::CameraInfoManager* _cInfo;

        ueye::paramsConfig              _config;

		//For dynamic_reconfigure
        dynamic_reconfigure::Server<ueye::paramsConfig> server;
        dynamic_reconfigure::Server<ueye::paramsConfig>::CallbackType f;

        ros::Timer                      _timer;

        void timer_callback( const ros::TimerEvent& )
        {
            if( _cam->nextFrame() ){
                    const cvt::Image& img = _cam->frame();
                    // publish the frame
                    _msg.header.stamp = ros::Time::now();

                    // update caminfo stuff
                    _camInfoMsg = _cInfo->getCameraInfo();
                    _camInfoMsg.header= _msg.header;

                    cvt_ros_bridge::image2RosMsg( img, _msg );
                    _pub.publish( _msg, _camInfoMsg );
            }
        }
};
