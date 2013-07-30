#ifndef CHAMELEON_CAM_CHAMELEON_STEREO_H
#define CHAMELEON_CAM_CHAMELEON_STEREO_H

#include <cvt/io/V4L2Camera.h>
#include <cvt/util/Thread.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cvt_ros/ChameleonSettingsConfig.h>

#include <cvt/util/Mutex.h>


namespace cvt_ros {

	class V4L2Stereo : public cvt::Thread<void>
	{
		public:
			V4L2Stereo();
			~V4L2Stereo();

			void execute( void* );			
			bool waitFrames( size_t val );

		private:
			typedef boost::shared_ptr< camera_info_manager::CameraInfoManager > CameraInfoManPtr;

			cvt::V4L2Camera*			_left;
			cvt::V4L2Camera*			_right;

			ros::Rate					_rate;
			size_t						_iter;
			ros::Time					_triggerTime;
			std::string					_cameraFrameId;

			CameraInfoManPtr			_cInfoLeft;
			CameraInfoManPtr			_cInfoRight;

			// publishers
			image_transport::CameraPublisher	_pubLeft;
			image_transport::CameraPublisher	_pubRight;

			//For dynamic_reconfigure
			cvt::Mutex					_reconfigureMutex;

			void publishFrames();
			void fillMessage( sensor_msgs::Image& iMsg, const cvt::Image& image, const std::string& frameId ) const;
	};

}

#endif
