#ifndef CVTROS_POSEVIEW_H
#define CVTROS_POSEVIEW_H

#include <cvt/geom/ArcBall.h>
#include <cvt/gui/GLView.h>
#include <cvt/gl/progs/GLBasicProg.h>
#include <cvt/gl/GLLines.h>
#include <cvt/geom/scene/ScenePoints.h>
#include <cvt/gl/GLPoints.h>

namespace cvt_ros
{
    class PoseView : public cvt::GLView
    {
        public:
            PoseView();

            /**
             *  \brief set the current estimated pose
             *  \param m    pose from World TO Camera
             */
            void setCamPose( const cvt::Matrix4f & m );

            /**
             *  \brief will reset the virtual view slightly behind the current camera pose
             */
            void resetCameraView();

            void setOffsetPose( const cvt::Matrix4f& pose );
            void setScenePoints( const cvt::ScenePoints& pts );

            void setDrawGrid( bool val ){ _drawGrid = val; }

        protected:
            void paintGLEvent( cvt::PaintEvent& );
            void mousePressEvent( cvt::MousePressEvent& e );
            void mouseReleaseEvent( cvt::MouseReleaseEvent& e );
            void mouseMoveEvent( cvt::MouseMoveEvent& e );
            void resizeEvent( cvt::ResizeEvent& e );

        private:
            // view transform (virtual cam)
            cvt::Matrix4f    _offset;
            cvt::Matrix4f	_rot;
            cvt::Vector3f	_trans;
            float		_near;
            float		_far;

            bool        _drawGrid;

            // Current Camera pose
            cvt::Matrix4f	_cam;

            cvt::ArcBall	_arcball;
            cvt::Vector2i	_press;
            cvt::Vector2i	_panPress;

            cvt::GLBasicProg _basicProg;

            // base level grid
            cvt::GLVertexArray	_grid;
            cvt::GLBuffer       _gridLines;
            size_t              _numLines;

            cvt::GLVertexArray	 _axes;
            cvt::GLBuffer        _axesBuf;
            cvt::GLBuffer        _axesColBuf;

            cvt::GLPoints        _scenePoints;

            void createGrid( ssize_t halfRes );
            void createAxes();
    };
}

#endif
