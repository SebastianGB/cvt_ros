#include <ros/ros.h>

#include <cvt/io/UEyeUsbCamera.h>
#include <cvt/util/Time.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/fill_image.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <ueye/paramsConfig.h>

#include <cvt/io/UEyeStereo.h>

using namespace cvt;

class CvtCamera
{
	public:
        CvtCamera( const String& master, const String& slave ) :
            _stereo( master, slave ),
            _imageNode( "~" ),
			_nh0( _imageNode, "left" ),
			_nh1( _imageNode, "right" ),
			_it0( _nh0 ),
			_it1( _nh1 ),
			_pub0( _it0.advertiseCamera( "image_raw", 2 ) ),
			_pub1( _it1.advertiseCamera( "image_raw", 2 ) ),
			_cInfo0( 0 ),
			_cInfo1( 0 )
        {
			String resourcePath( "package://ueye/resources/" );
			String name0 = "ueye_";
            name0 += master;
			String name1 = "ueye_";
            name1 += slave;

			String saveUrl0 = resourcePath;
			saveUrl0 += name0;
			saveUrl0 += ".yaml";
			String saveUrl1 = resourcePath;
			saveUrl1 += name1;
			saveUrl1 += ".yaml";
		
			_cInfo0 = new camera_info_manager::CameraInfoManager( _nh0, name0.c_str(), saveUrl0.c_str() );
			_cInfo1 = new camera_info_manager::CameraInfoManager( _nh1, name1.c_str(), saveUrl1.c_str() );

			String cFile( resourcePath );
			cFile += name0; cFile += ".yaml";
			if( _cInfo0->validateURL( cFile.c_str() ) ){
				_cInfo0->loadCameraInfo( cFile.c_str() );
			}
			cFile = resourcePath;
			cFile += name1; cFile += ".yaml";
			if( _cInfo1->validateURL( cFile.c_str() ) ){
				_cInfo1->loadCameraInfo( cFile.c_str() );
			}


			// advertise image topic
			_msg0.header.frame_id = "/ueye_stereo";
			_camInfoMsg0.header.frame_id = _msg0.header.frame_id;
			_msg1.header.frame_id = _msg0.header.frame_id;
			_camInfoMsg1.header.frame_id = _msg1.header.frame_id;

            updateCamInfo( _stereo.master(), _camInfoMsg0 );
            updateCamInfo( _stereo.slave(),  _camInfoMsg1);

			//For dynamic_reconfigure
			f = boost::bind(&CvtCamera::reconfigureCallback,this, _1, _2);
		 	server.setCallback(f);
		}

		~CvtCamera()
		{
			delete _cInfo0;
			delete _cInfo1;
		}

        void updateCamInfo( const UEyeUsbCamera& cam, sensor_msgs::CameraInfo & camInfo )
		{
            if( cam.width() != camInfo.width ||
                cam.height() != camInfo.height ){
                camInfo.width = cam.width();
                camInfo.height = cam.height();
			}
		}

		void run()
		{
			//String path;
			size_t iter = 0;
			
			ros::Rate rate( 100 );
			Time timer;

			//Main loop
			while( ros::ok() ){
                if( _stereo.nextFrame() ){
                    const cvt::Image& i0 = _stereo.masterFrame();
                    const cvt::Image& i1 = _stereo.slaveFrame();
                    iter++;

                    // publish the frame
                    _msg0.header.seq = iter;
                    _msg1.header.seq = iter;
                    _msg0.header.stamp = ros::Time::now();
                    _msg1.header.stamp = _msg0.header.stamp;

                    // update caminfo stuff
                    _camInfoMsg0 = _cInfo0->getCameraInfo();
                    _camInfoMsg1 = _cInfo1->getCameraInfo();
                    _camInfoMsg0.header= _msg0.header;
                    _camInfoMsg1.header= _msg1.header;

                    cvt_ros_bridge::image2RosMsg( i0, _msg0 );
                    cvt_ros_bridge::image2RosMsg( i1, _msg1 );
                    _pub0.publish( _msg0, _camInfoMsg0 );
                    _pub1.publish( _msg1, _camInfoMsg1 );

                    //Displays the FPS
                    if( iter % 100 == 0 ){
                        std::cout << "FPS: " << iter / timer.elapsedSeconds() << std::endl;
                    }
				
                    ros::spinOnce();
                }
				rate.sleep();
			}
		}

		//Work in progress
		void reconfigureCallback(ueye::paramsConfig &config, uint32_t level) {

            if( config.hor_mirror != _config.hor_mirror ){
                _stereo.setHorizontalMirror( config.hor_mirror );
            }

            if( config.ver_mirror != _config.ver_mirror ){
                _stereo.setVerticalMirror( config.ver_mirror );
            }

            if( config.pixel_clock != _config.pixel_clock ){
                _stereo.setPixelClock( config.pixel_clock );
            }

            if( config.auto_sensor_shutter != _config.auto_sensor_shutter ){
                _stereo.setAutoSensorShutter( config.auto_sensor_shutter );
            }

            if( config.auto_shutter != _config.auto_shutter ){
                _stereo.setAutoShutter( config.auto_shutter );
            }

            if( config.framerate != _config.framerate ){
                _stereo.setFramerate( config.framerate );
            }

            if( config.exposure_time != _config.exposure_time ){
                _stereo.setExposureTime( config.exposure_time );
            }

/* NOT IMPLEMENTED YET
			//Image Dimensions
			_cam0->setImageWidth(  config.image_width ) ;
			_cam1->setImageWidth(  config.image_width ) ;
			_cam0->setImageHeight(  config.image_height ) ;
			_cam1->setImageHeight(  config.image_height ) ;
			//Image Offsets
			_cam0->setImageTopOffset(  config.image_top_C1 ) ;
			_cam1->setImageTopOffset(  config.image_top_C2 ) ;
			_cam0->setImageLeftOffset(  config.image_left_C1 ) ;
			_cam1->setImageLeftOffset(  config.image_left_C2 ) ;
*/
            _config = config;
		}

	private:
        UEyeStereo                          _stereo;
		ros::NodeHandle 					_imageNode;
		ros::NodeHandle 					_nh0;
		ros::NodeHandle 					_nh1;
		image_transport::ImageTransport		_it0;
		image_transport::ImageTransport		_it1;
		image_transport::CameraPublisher	_pub0; 
		image_transport::CameraPublisher	_pub1; 


		sensor_msgs::Image _msg0;
		sensor_msgs::Image _msg1;
		sensor_msgs::CameraInfo _camInfoMsg0;
		sensor_msgs::CameraInfo _camInfoMsg1;

		camera_info_manager::CameraInfoManager* _cInfo0;
		camera_info_manager::CameraInfoManager* _cInfo1;

        ueye::paramsConfig              _config;

		//For dynamic_reconfigure
        dynamic_reconfigure::Server<ueye::paramsConfig> server;
        dynamic_reconfigure::Server<ueye::paramsConfig>::CallbackType f;
};


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "ueye_stereo" );
    ros::NodeHandle nh( "~" );

    std::string leftId;
    std::string rightId;
    nh.param<std::string>( "left_id",  leftId,  "4002738790" );
    nh.param<std::string>( "right_id", rightId, "4002738788" );

    CvtCamera ccam( leftId.c_str(), rightId.c_str() );

	std::cout << "Starting runloop" << std::endl;
    ccam.run();
        
	return 0;
}
