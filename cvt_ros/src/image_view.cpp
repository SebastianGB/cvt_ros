#include <ros/ros.h>
#include <cvt/gui/Window.h>
#include <cvt/gui/Application.h>
#include <cvt/gui/ImageView.h>
#include <cvt/gui/Moveable.h>
#include <cvt/gui/TimeoutHandler.h>
#include <cvt/gui/Button.h>
#include <cvt/gui/WidgetLayout.h>
#include <cvt/util/Time.h>
#include <cvt/util/Delegate.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>
#include <cvt_ros/ImageSubscriber.h>
#include <cvt_ros/ROSSpinner.h>

namespace cvt_ros {
    class Window : public ImageSubscriber
    {
        public:
            Window() :
                ImageSubscriber(),
                _window( "Image View" ),
                _saveButton( "save" ),
                _numFrames( 0 ),
                _saveNext( false ),
                _saveIter( 0 )
            {
                cvt::WidgetLayout wl;
                wl.setAnchoredBottom( 10, 20 );
                wl.setAnchoredRight( 10, 100 );
                _window.addWidget( &_saveButton, wl );

                wl.setRelativeLeftRight( 0.01f, 0.99f );
                wl.setRelativeTopBottom( 0.01f, 0.9f );
                _window.addWidget( &_view, wl );

                _window.setSize( 600, 400 );
                _window.update();
                _window.setVisible( true );

                cvt::Delegate<void ()> d( this, &Window::buttonPressed );
                _saveButton.clicked.add( d );
            }

            ~Window()
            {
                ros::shutdown();
                _window.removeWidget( &_view );
                _window.removeWidget( &_saveButton );
                _window.raise();
            }

            void imageCallback( const cvt::Image& img )
            {
                try {
                    _view.setImage( img );
                } catch( cvt::Exception& e ){
                    std::cout << e.what() << std::endl;
                }
                _numFrames++;

                if( _elapsedTime.elapsedSeconds() > 5 ){
                    cvt::String str;

                    str.sprintf( "ImageView: %0.1f", _numFrames / ( _elapsedTime.elapsedSeconds() ) );
                    _numFrames = 0;
                    _elapsedTime.reset();
                    _window.setTitle( str );
                }

                if( _saveNext ){
                    _saveNext = false;
                    cvt::String str;
                    str.sprintf( "image_%03d.png", _saveIter );
                    img.save( str );
                    ROS_INFO( "SAVING IMAGE" );
                    _saveIter++;
                }
            }

        private:
            cvt::Window		_window;
            cvt::ImageView	_view;

            cvt::Button		_saveButton;

            cvt::Time       _elapsedTime;
            size_t          _numFrames;
            bool            _saveNext;
            size_t          _saveIter;

            void buttonPressed()
            {
                _saveNext = true;
            }

    };
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "image_view", ros::init_options::NoSigintHandler );
    ros::NodeHandle nh( "~" );

    cvt_ros::Window win;

    /* let ROS spin with 10ms interval */
    cvt_ros::ROSSpinner app( 10 );

    cvt::Application::run();
    return 0;
}
