#ifndef CHAMELEON_CAM_CHAMELEON_STEREO_H
#define CHAMELEON_CAM_CHAMELEON_STEREO_H

#include <cvt/io/ChameleonStereo.h>
#include <cvt/util/Thread.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cvt_ros/ChameleonSettingsConfig.h>

#include <cvt/util/Mutex.h>


namespace cvt_ros {

    class ChameleonStereo
	{
		public:
			ChameleonStereo();
			~ChameleonStereo();

            void run();

			void triggerFrame();
			bool waitFrames( size_t val );

		private:
			typedef boost::shared_ptr< camera_info_manager::CameraInfoManager > CameraInfoManPtr;
			typedef dynamic_reconfigure::Server<cvt_ros::ChameleonSettingsConfig> ServerType;

			cvt::ChameleonStereo*		_stereo;
			ros::Rate					_rate;
			size_t						_iter;
			ros::Time					_triggerTime;
			std::string					_cameraFrameId;

			CameraInfoManPtr			_cInfoMaster;
			CameraInfoManPtr			_cInfoSlave;

			// publishers
			image_transport::CameraPublisher	_pubMaster;
			image_transport::CameraPublisher	_pubSlave;

			//For dynamic_reconfigure
			ChameleonSettingsConfig		_config;
			ServerType					_server;
			ServerType::CallbackType	_recCb;

			void publishFrames();
			void fillMessage( sensor_msgs::Image& iMsg, const cvt::Image& image, const std::string& frameId ) const;

			cvt::DC1394Camera* createCameraFromId( const std::string& id ) const;
			void configureMaster( int strobePin );
			void configureSlave( int triggerPin );
			void configureStrobe( int pin );

			void setAutoExposureMode( int val );
			void setAutoShutterMode( int val );
			void setAutoGainMode( int val );
            void setWhiteBalanceMode( int val );

			int autoExposureMode() const;
			int autoShutterMode() const;
			int autoGainMode() const;
            int autoWhiteBalanceMode() const;

			void setExposure( float value );
			void setShutter( float value );
			void setGain( float value );            

			uint32_t exposure() const;
			uint32_t shutter() const;
			uint32_t gain() const;

			void setFPS( float fps );
			float fps() const;

			bool canTrigger() const;			
			void reconfigureCallback( cvt_ros::ChameleonSettingsConfig& config, uint32_t level );
	};

}

#endif
