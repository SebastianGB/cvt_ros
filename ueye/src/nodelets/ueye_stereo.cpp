// -- ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ueye/paramsConfig.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// -- CVT includes
#include <cvt/io/UEyeUsbCamera.h>
#include <cvt/io/UEyeStereo.h>
#include <cvt/util/Time.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>
#include <cvt/vision/StereoCameraCalibration.h>

using namespace cvt;

namespace ueye_cam
{

    class UEyeStereoCamNodelet : public nodelet::Nodelet
    {
        public:
            typedef boost::shared_ptr< camera_info_manager::CameraInfoManager >
            CameraInfoManPtr;

            typedef boost::shared_ptr< UEyeStereo > UEyeStereoPtr;
            typedef boost::shared_ptr<dynamic_reconfigure::Server<ueye::paramsConfig> > DynamicReconfServerPtr;


            UEyeStereoCamNodelet() {}
            ~UEyeStereoCamNodelet(); // dtor is virtual in nodelet::Nodelet

            virtual void onInit();

        private:
            UEyeStereoPtr                       _stereo;
            image_transport::CameraPublisher	_pub0;
            image_transport::CameraPublisher	_pub1;

            CameraInfoManPtr                    _cInfo0;
            CameraInfoManPtr                    _cInfo1;

            sensor_msgs::Image                  _msg0;
            sensor_msgs::Image                  _msg1;
            sensor_msgs::CameraInfo             _camInfoMsg0;
            sensor_msgs::CameraInfo             _camInfoMsg1;
            ueye::paramsConfig                  _config;
            static const std::string            leftCamID;
            static const std::string            rightCamID;

            //For dynamic_reconfigure
            DynamicReconfServerPtr server;
            dynamic_reconfigure::Server<ueye::paramsConfig>::CallbackType f;

            ros::Timer                          _timer;

            void publishCallback( const ros::TimerEvent& event );
            void reconfigureCallback(ueye::paramsConfig &config,
                                     uint32_t level);
            void updateCamInfo( const UEyeUsbCamera& cam,
                                sensor_msgs::CameraInfo & camInfo );
    };
    
    const std::string UEyeStereoCamNodelet::leftCamID =     "4002738790";
    const std::string UEyeStereoCamNodelet::rightCamID =    "4002738788";

    PLUGINLIB_DECLARE_CLASS( ueye_cam, UEyeStereoCamNodelet, ueye_cam::UEyeStereoCamNodelet, nodelet::Nodelet )

    void UEyeStereoCamNodelet::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");

        const ros::NodeHandle& nh(getPrivateNodeHandle());
        ros::NodeHandle _nh0(nh, "left");
        ros::NodeHandle _nh1(nh, "right");

        image_transport::ImageTransport _it0( _nh0 );
        image_transport::ImageTransport _it1( _nh1 );
        _pub0   = _it0.advertiseCamera( "image_raw", 2 );
        _pub1   = _it1.advertiseCamera( "image_raw", 2 );

        {   // init stereo camera
            std::string leftId;
            std::string rightId;

            // get values from parameter server and init camera
            nh.param<std::string>( "left_id",  leftId,  leftCamID.c_str() );
            nh.param<std::string>( "right_id", rightId, rightCamID.c_str() );
            _stereo = UEyeStereoPtr(new UEyeStereo( leftId.c_str(), rightId.c_str()));

            // get camera calibration
            String name_left_cam( String( "ueye_" ) + leftId.c_str() );
            String name_right_cam( String( "ueye_" ) + rightId.c_str() );

            String resourcePath( "package://ueye/resources/" );
            String saveURL0( resourcePath + name_left_cam + ".yaml" );
            String saveURL1( resourcePath + name_right_cam + ".yaml" );

            _cInfo0 = CameraInfoManPtr( new camera_info_manager::CameraInfoManager(
                                            _nh0, name_left_cam.c_str(), saveURL0.c_str() ) );

            _cInfo1 = CameraInfoManPtr( new camera_info_manager::CameraInfoManager(
                                            _nh1, name_right_cam.c_str(), saveURL1.c_str() ) );

            if ( _cInfo0->validateURL( saveURL0.c_str() ) ) {
                _cInfo0->loadCameraInfo( saveURL0.c_str() );
            }
            if ( _cInfo1->validateURL( saveURL1.c_str() ) ) {
                _cInfo1->loadCameraInfo( saveURL1.c_str() );
            }
        }

        {   // update camera info?
            _msg0.header.frame_id = "/ueye_stereo";
            _camInfoMsg0.header.frame_id = _msg0.header.frame_id;
            _msg1.header.frame_id = _msg0.header.frame_id;
            _camInfoMsg1.header.frame_id = _msg1.header.frame_id;

            updateCamInfo( _stereo->master(), _camInfoMsg0 );
            updateCamInfo( _stereo->slave(),  _camInfoMsg1 );
        }

        //For dynamic_reconfigure
        f = boost::bind( &UEyeStereoCamNodelet::reconfigureCallback, this, _1, _2 );
        server = DynamicReconfServerPtr(
                     new dynamic_reconfigure::Server<ueye::paramsConfig>());
        server->setCallback( f );

        {   // timer for the camera images
            int fps;
            nh.param( "fps", fps, 30 );
            _timer = nh.createTimer(
                         ros::Duration( 1.0f / static_cast<float>( fps ) ),
                         &UEyeStereoCamNodelet::publishCallback, this );
            _timer.start();
        }
    }

    UEyeStereoCamNodelet::~UEyeStereoCamNodelet()
    {
        _timer.stop();
    }

    void UEyeStereoCamNodelet::updateCamInfo( const UEyeUsbCamera& cam,
                                              sensor_msgs::CameraInfo & camInfo )
    {
        if( cam.width() != camInfo.width ||
            cam.height() != camInfo.height ){
            camInfo.width = cam.width();
            camInfo.height = cam.height();
        }
    }


    void UEyeStereoCamNodelet::publishCallback( const ros::TimerEvent& )
    {
        size_t iter = 0;

        if ( _stereo->nextFrame() ) {
            const cvt::Image& i0 = _stereo->masterFrame();
            const cvt::Image& i1 = _stereo->slaveFrame();
            ++iter;

            // publish the frame
            _msg0.header.seq = iter;
            _msg1.header.seq = iter;
            _msg0.header.stamp = ros::Time::now();
            _msg1.header.stamp = _msg0.header.stamp;

            // maybe not so smart after all, this can be done with ros::image_proc
            // would be useful to publish a stereo-rectified version here as well:
            //        cvt::StereoCameraCalibration stereoCC();
            //        stereoCC.load(cvt::String("ueye_stereo_cc.xml"));
            //        stereoCC.undistortRectify();



            // update caminfo stuff
            _camInfoMsg0 = _cInfo0->getCameraInfo();
            _camInfoMsg1 = _cInfo1->getCameraInfo();
            _camInfoMsg0.header= _msg0.header;
            _camInfoMsg1.header= _msg1.header;

            cvt_ros_bridge::image2RosMsg( i0, _msg0 );
            cvt_ros_bridge::image2RosMsg( i1, _msg1 );
            _pub0.publish( _msg0, _camInfoMsg0 );
            _pub1.publish( _msg1, _camInfoMsg1 );
        }
    }

    //Work in progress
    void UEyeStereoCamNodelet::reconfigureCallback(
            ueye::paramsConfig &config, uint32_t /*level*/ ) {

        if( config.hor_mirror != _config.hor_mirror ){
            _stereo->setHorizontalMirror( config.hor_mirror );
        }

        if( config.ver_mirror != _config.ver_mirror ){
            _stereo->setVerticalMirror( config.ver_mirror );
        }

        if( config.pixel_clock != _config.pixel_clock ){
            _stereo->setPixelClock( config.pixel_clock );
        }

        if( config.auto_sensor_shutter != _config.auto_sensor_shutter ){
            _stereo->setAutoSensorShutter( config.auto_sensor_shutter );
        }

        if( config.auto_shutter != _config.auto_shutter ){
            _stereo->setAutoShutter( config.auto_shutter );
        }

        if( config.framerate != _config.framerate ){
            _stereo->setFramerate( config.framerate );
            _timer.setPeriod( ros::Duration( 1.0f / ( config.framerate + 5 ) ) );
        }

        if( config.exposure_time != _config.exposure_time ){
            _stereo->setExposureTime( config.exposure_time );
        }

        if( config.framerate != _config.framerate ){
            _timer.setPeriod( ros::Duration( 1.0f / config.framerate) );
        }

        if( config.auto_gain != _config.auto_gain ){
            _stereo->setAutoGain( config.auto_gain );
        }

        if( config.gain_boost != _config.gain_boost ){
            _stereo->setGainBoost( config.gain_boost );
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
    // -------------

}
