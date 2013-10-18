#include <RGBDGui.h>
#include <cvt/gui/Application.h>
#include <cvt/geom/scene/ScenePoints.h>
#include <cvt/vision/Vision.h>

namespace cvt_ros {

    RGBDGui::RGBDGui( const cvt::Matrix3f &intrinsics, float depthScale ) :
        _mainWindow( "RGBD-View" ),
        _depthFactor( depthScale ),
        _drawGrid( "Draw Grid" )
    {
        _camCalib.setIntrinsics( intrinsics );
        setupGui();
    }

    RGBDGui::~RGBDGui()
    {
    }

    void RGBDGui::setupGui()
    {
        _mainWindow.setSize( 1024, 768 );

        cvt::WidgetLayout wl;
        wl.setRelativeLeftRight( 0.0f, 0.75f );
        wl.setRelativeTopBottom( 0.0f, 1.0f );
        _mainWindow.addWidget( &_poseView, wl );

        wl.setRelativeTopBottom( 0.0f, 0.45f );
        wl.setRelativeLeftRight( 0.76f, 1.0f );
        _mainWindow.addWidget( &_currentImage, wl );
        wl.setRelativeTopBottom( 0.46f, 0.9f );
        _mainWindow.addWidget( &_depthView, wl );

        wl.setRelativeTopBottom( 0.9f, 1.0f );
        _mainWindow.addWidget( &_drawGrid, wl );

        cvt::Delegate<void (cvt::ToggleButton*)> delegate( this, &RGBDGui::drawGridToggled );
        _drawGrid.toggled.add( delegate );
        _poseView.setDrawGrid( _drawGrid.state() );

        _mainWindow.setVisible( true );
    }

    void RGBDGui::setOffsetPose( const cvt::Matrix4f& pose )
    {
        _poseView.setOffsetPose( pose );
    }

    void RGBDGui::setCurrentRGB( const cvt::Image& rgb )
    {
        _currentImage.setImage( rgb );
    }

    void RGBDGui::setCurrentDepth( const cvt::Image& depth )
    {
        cvt::Image scaled( depth );
        scaled.mul( 6.0f );
        _depthView.setImage( scaled );
    }

    void RGBDGui::setPose( const cvt::Matrix4f& pose )
    {
        _poseView.setCamPose( pose );
        _camCalib.setExtrinsics( pose.inverse() );
    }

    void RGBDGui::drawGridToggled( cvt::ToggleButton* button )
    {
        _poseView.setDrawGrid( button->state() );
    }

    void RGBDGui::updateScenePoints(const cvt::Image &rgb, const cvt::Image &depth )
    {
        cvt::ScenePoints curPts( "curTmp" );
        cvt::Image rgbf, df;
        rgb.convert( rgbf, cvt::IFormat::RGBA_FLOAT );
        depth.convert( df, cvt::IFormat::GRAY_FLOAT );
        cvt::Vision::unprojectToScenePoints( curPts, rgbf, df, _camCalib, ( float ) ( 0xffff ) / _depthFactor );
        _poseView.setScenePoints( curPts );
    }

}
