#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <cvt_ros/ImageSubscriber.h>
#include <camera_calibration_parsers/parse.h>

#include <cvt/io/FileSystem.h>

class CalibRepublisher : public cvt_ros::ImageSubscriber
{
    public:
        CalibRepublisher() :
            _nh( "~" )
        {
            std::string calibFile;
            _nh.param<std::string>( "calib_url", calibFile, "calib.yaml" );

            ROS_INFO( "Loading calibration data: %s", calibFile.c_str() );
            if( !cvt::FileSystem::exists( cvt::String( calibFile.c_str() ) ) ){
                throw cvt::Exception( "Calibration file does not exists!" );
            }

            std::string camName;
            camera_calibration_parsers::readCalibration( calibFile, camName, _camInfo );

            cvt::String topic( this->_sub.getTopic().c_str() );
            cvt::String camInfoTopic = topic.substring( 0, topic.rfind( '/' ) + 1 ) + "camera_info";
            ROS_INFO( "CAMINFO TOPIC: %s", camInfoTopic.c_str() );

            _pub = _nh.advertise<sensor_msgs::CameraInfo>( camInfoTopic.c_str(), 4 );
        }

        void imageCallback( const cvt::Image& ){}

        void imageMessageCallback( const sensor_msgs::ImageConstPtr &imageMsg )
        {
            _camInfo.header = imageMsg->header;
            _pub.publish( _camInfo );
        }

    private:
        ros::NodeHandle         _nh;
        ros::Publisher          _pub;
        sensor_msgs::CameraInfo _camInfo;
};

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "synced_calib_publisher" );
    ros::NodeHandle nh( "~" );

    try {
        CalibRepublisher repub;
        ros::spin();
    } catch( const cvt::Exception& e ){
        ROS_ERROR( "Exception: %s", e.what() );
    }

    return 0;
}
