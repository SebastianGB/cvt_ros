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
#include <cvt_ros/StereoSubscriber.h>
#include <cvt_ros/ROSSpinner.h>

namespace cvt_ros {
	class StereoWindow : public StereoSubscriber
	{
		public:
			StereoWindow() :
                StereoSubscriber(),
                _window( "Stereo View" ),
                _saveButton( "save" ),
				_numFrames( 0 ),
				_saveIter( 0 )
			{
				cvt::WidgetLayout wl;
				wl.setAnchoredBottom( 10, 20 );
				wl.setAnchoredRight( 10, 100 );
				_window.addWidget( &_saveButton, wl );

				wl.setRelativeLeftRight( 0.01f, 0.495f );
				wl.setRelativeTopBottom( 0.01f, 0.9f );
				_window.addWidget( &_view0, wl );
				wl.setRelativeLeftRight( 0.505f, 0.99f );
				_window.addWidget( &_view1, wl );

				_window.setSize( 600, 400 );
				_window.update();
				_window.setVisible( true );

				cvt::Delegate<void ()> d( this, &StereoWindow::buttonPressed );
				_saveButton.clicked.add( d );
			}

			~StereoWindow()
			{
                _window.removeWidget( &_view0 );
                _window.removeWidget( &_view1 );
                _window.removeWidget( &_saveButton );
			}

			void imageCallback( const cvt::Image& left, const cvt::Image& right )
			{
				try {
					_view0.setImage( _left );
					_view1.setImage( _right );
				} catch( cvt::Exception& e ){
					std::cout << e.what() << std::endl;
				}
				_numFrames++;
				
                if( _elapsedTime.elapsedSeconds() > 5 ){
                    cvt::String str;

                    str.sprintf( "StereoView: %0.1f", _numFrames / ( _elapsedTime.elapsedSeconds() ) );
					_numFrames = 0;
					_elapsedTime.reset();
					_window.setTitle( str );
				}
			}

		private:
            cvt::Window		_window;
			cvt::ImageView	_view0;
			cvt::ImageView	_view1;
            cvt::Button		_saveButton;

			cvt::Time	_elapsedTime;
			size_t		_numFrames;
			size_t		_saveIter;

			void buttonPressed()
			{
				cvt::String str;
				str.sprintf( "image_left_%03d.png", _saveIter );
				_left.save( str );
				str.sprintf( "image_right_%03d.png", _saveIter );
				_right.save( str );
				ROS_INFO( "SAVING IMAGES" );
				++_saveIter;
			}
	};

}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "iafc_stereo_view", ros::init_options::NoSigintHandler );

    cvt_ros::StereoWindow stereoWin;
    cvt_ros::ROSSpinner spinner( 10 );
    cvt::Application::run();
	return 0;
}
