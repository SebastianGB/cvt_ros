#include <ros/node_handle.h>
#include <cvt_ros/RGBDCameraConfig.h>
#include <cvt/io/OpenNI2Camera.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

namespace cvt_ros
{

    class RGBDCamera
    {
        public:
            RGBDCamera();
            ~RGBDCamera();

            void spin();
            void spinOnce();

        private:
            typedef boost::shared_ptr< camera_info_manager::CameraInfoManager > CameraInfoManPtr;
            typedef dynamic_reconfigure::Server<cvt_ros::RGBDCameraConfig> ServerType;

            cvt::OpenNI2Camera* _cam;
            ros::Rate           _loopRate;
            ServerType          _reconfigureServer;
            RGBDCameraConfig    _config;

			// publishers
			image_transport::CameraPublisher	_pubRGB;
			image_transport::CameraPublisher	_pubDepth;
			CameraInfoManPtr					_rgbCamManager;
			CameraInfoManPtr					_depthCamManager;

			std::string			_rgbFrameId;
			std::string			_depthFrameId;
			bool				_frameRegistration;
			bool				_frameSync;

            /**
             * @brief initialize OpenNI Camera according to parameters
             *        stored in ROS param server
             */
            void initCamera();

            /**
             * @brief start capturing frames
             */
            void startCapture();

            /**
             * @brief stop capturing frames
             */
            void stopCapture();

            void configCb( cvt_ros::RGBDCameraConfig& config, uint32_t level );
    };
}
