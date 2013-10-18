#ifndef CVT_ROS_RGBDVIEW_H
#define CVT_ROS_RGBDVIEW_H

#include <cvt/gui/Window.h>
#include <cvt/gui/ImageView.h>
#include <cvt/gui/Moveable.h>
#include <cvt/gui/Label.h>
#include <cvt/gui/Button.h>
#include <cvt/gui/ToggleButton.h>
#include <cvt/gui/TimeoutHandler.h>
#include <cvt/util/Mutex.h>
#include <cvt/vision/CameraCalibration.h>
#include <PoseView.h>

namespace cvt_ros
{
    class RGBDGui
    {
        public:
            RGBDGui( const cvt::Matrix3f& intrinsics, float depthScale );
            ~RGBDGui();

            void setCurrentRGB( const cvt::Image& rgb );
            void setCurrentDepth( const cvt::Image& depth );
            void setPose( const cvt::Matrix4f &data );
            void setOffsetPose( const cvt::Matrix4f& pose );
            void updateScenePoints( const cvt::Image& rgb, const cvt::Image& depth );

        private:
            cvt::Window             _mainWindow;
            cvt::ImageView          _currentImage;
            cvt::ImageView          _depthView;
            cvt::ToggleButton       _drawGrid;
            PoseView                _poseView;


            cvt::CameraCalibration  _camCalib;
            float                   _depthFactor;

            void setupGui();
            void drawGridToggled( cvt::ToggleButton* button );
    };

}

#endif
