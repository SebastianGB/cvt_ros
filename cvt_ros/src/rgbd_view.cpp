#include <ros/ros.h>

#include <RGBDGui.h>
#include <cvt_ros/RGBDSubscriber.h>
#include <cvt_ros/ROSSpinner.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace cvt;

namespace cvt_ros {
    class RGBDViewNode : public RGBDSubscriber {
        public:
            RGBDViewNode() :
                RGBDSubscriber(),
                _tfListener( _tfBuffer )
            {
                ros::NodeHandle nh( "~" );
                nh.param<std::string>( "parent_frame", _parentFrame, "world" );
                nh.param<std::string>( "child_frame", _childFrame, "camera" );
                nh.param<bool>( "use_tf", _useTFFrame, false );
            }

            ~RGBDViewNode()
            {
                if( _gui ){
                    delete _gui;
                }
            }

            void imageCallback( const Image &rgb, const Image &depth )
            {
                if( _gui ){
                    _gui->setCurrentRGB( rgb );
                    _gui->setCurrentDepth( depth );

                    if( _useTFFrame ){
                        trySetPose();
                    }
                    _gui->updateScenePoints( rgb, depth );
                }
            }

        private:
            RGBDGui*                    _gui;

            bool                        _useTFFrame;
            tf2_ros::Buffer             _tfBuffer;
            tf2_ros::TransformListener  _tfListener;

            std::string                 _parentFrame;
            std::string                 _childFrame;

            void camInfoCallback( const sensor_msgs::CameraInfoConstPtr& camInfo )
            {
                RGBDSubscriber::camInfoCallback( camInfo );
                _gui = new RGBDGui( _intrinsics, 1000.0f );
            }

            void trySetPose()
            {
                try {
                    geometry_msgs::TransformStamped t = _tfBuffer.lookupTransform( _childFrame,
                                                                                   _parentFrame,
                                                                                   _rgbHeader.stamp,
                                                                                   ros::Duration( 0.05 ) );
                    cvt::Matrix4f mat = cvt_ros_bridge::toCVTMatrix<float>( t );
                    _gui->setPose( mat );
                } catch( tf2::TransformException& ){
                    // pose timeout
                }
            }

    };
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "rgbd_view" );
    ros::NodeHandle nh( "~" );

    cvt_ros::ROSSpinner spinner( 10 );
    cvt_ros::RGBDViewNode guiNode;

    cvt::Application::run();

    return 0;
}
