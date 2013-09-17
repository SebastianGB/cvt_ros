#include <ros/ros.h>
#include <ros/package.h>
#include <cvt_ros/RGBDSubscriber.h>
#include <tf2_ros/transform_listener.h>

#include <cvt/cl/OpenCL.h>
#include <cvt/cl/CLPlatform.h>
#include <cvt/vision/TSDFVolume.h>
#include <cvt/vision/CameraCalibration.h>

using namespace cvt_ros;

class TSDFMeshing : public RGBDSubscriber
{
    public:
        TSDFMeshing() :
            RGBDSubscriber(),
            _tfListener( _tfBuffer ),
            _volume( 0 ),
            _factorDepthToMeter( (float)(0xFFFF) / 1000.0f ),
            _nMaps( 0 )
        {
            init();
        }

        TSDFMeshing( const cvt::Matrix3f& intrinsics ) :
            RGBDSubscriber( intrinsics ),
            _tfListener( _tfBuffer ),
            _volume( 0 ),
            _factorDepthToMeter( (float)(0xFFFF) / 1000.0f ),
            _nMaps( 0 )
        {
            init();
        }

    private:
        tf2::Buffer             _tfBuffer;
        tf2::TransformListener  _tfListener;
        std::string             _base, _moving;
        ros::Duration           _timeout;

        cvt::Matrix4f           _gridToWorld;
        cvt::TSDFVolume*        _volume;
        float                   _factorDepthToMeter;
        uint32_t                _nMaps;
        std::vector<cvt::CLPlatform> _platforms;

        void init()
        {
            ros::NodeHandle nh( "~" );
            _moving = "direct_vo";
            _base = "world";
            double dt = 0.4;

            nh.param<std::string>( "moving_frame", _moving, _moving );
            nh.param<std::string>( "base_frame", _base, _base );

            ROS_INFO( "BASE: %s", _base.c_str() );
            ROS_INFO( "MOVING: %s", _moving.c_str() );

            nh.param<double>( "time_out", dt, dt );
            _timeout.fromSec( dt );

            cvt::CLPlatform::get( _platforms );
            cvt::CL::setDefaultDevice( _platforms[ 0 ].defaultDevice() );

            createTSDF( 2, 2, 2, 0.004f );
            _volume->clear();
        }


        void imageCallback( const cvt::Image& rgb, const cvt::Image& depth )
        {            
            cvt::Image depthCL;
            depth.convert( depthCL, cvt::IFormat::GRAY_UINT16, cvt::IALLOCATOR_CL );

            try {
                geometry_msgs::TransformStamped transform = _tfBuffer.lookupTransform( _moving, _base, _rgbHeader.stamp, _timeout );
                cvt::Quaternionf q;
                q.x = transform.transform.rotation.x;
                q.y = transform.transform.rotation.y;
                q.z = transform.transform.rotation.z;
                q.w = transform.transform.rotation.w;

                cvt::Matrix4f pose( q.toMatrix3() );
                pose[ 0 ][ 3 ] = transform.transform.translation.x;
                pose[ 1 ][ 3 ] = transform.transform.translation.y;
                pose[ 2 ][ 3 ] = transform.transform.translation.z;

                // Add to the TSDF
                std::cout << _intrinsics << std::endl;
                _volume->addDepthMap( _intrinsics, pose, depthCL, _factorDepthToMeter );
                _nMaps++;
                ROS_INFO( "Volume contains %d depthmaps", _nMaps );
            } catch ( const tf2::TransformException& e ){
                ROS_WARN( "Error in transform lookup: %s", e.what() );
                return;
            }

            // TODO: publish / display current mesh -> not in each step
            if( _nMaps % 200 == 0 ){
                cvt::SceneMesh mesh( "TSDF_MESH" );
                _volume->toSceneMesh( mesh );

                mesh.transform( _gridToWorld );

                meshToOBJ( "tsdf.obj", mesh );
            }
        }

        void createTSDF( uint32_t size_x, uint32_t size_y, uint32_t size_z, float resolution )
        {
            // compute the number of needed "Voxels"
            int nx = size_x / resolution;
            if( nx > 512 ){
                nx = 512;
                resolution = ( float )size_x / ( float )nx;
            }

            int ny = size_y / resolution;
            if( ny > 512 ){
                ny = 512;
                resolution = ( float )size_y / ( float )ny;
                nx = size_x / resolution;
            }

            int nz = size_z / resolution;
            if( nz > 512 ){
                nz = 512;
                resolution = ( float )size_z / ( float )nz;
                nx = size_x / resolution;
                ny = size_y / resolution;
            }

            ROS_INFO( "%d, %d, %d", nx, ny, nz );

            _gridToWorld = cvt::Matrix4f( resolution, 0.0f, 0.0f, -( float )size_x / 2.0f,
                                          0.0f, resolution, 0.0f, -( float )size_y / 2.0f,
                                          0.0f, 0.0f, resolution,  ( float )0.0f,
                                          0.0f, 0.0f, 0.0f, 1.0f );
            std::cout << "GRID2WORLD: " <<_gridToWorld << std::endl;


//            _gridToWorld = cvt::Matrix4f( 2.0f / ( float )( 512 ), 0.0f, 0.0f,  -0.25f,
//                                  0.0f, 2.0f / ( float )( 512 ), 0.0f, -1.5f,
//                                  0.0f, 0.0f, 2.0f / ( float ) ( 512 ), -0.5f,
//                                  0.0f, 0.0f, 0.0f, 0.25 );
//            _gridToWorld *= 4.0f;

            if( _volume != 0 ){
                delete _volume;
            }
            _volume = new cvt::TSDFVolume( _gridToWorld, nx, ny, nz, 15.0f * resolution );
            //_volume = new cvt::TSDFVolume( _gridToWorld, 512, 512, 512, 0.07f );
        }

        void meshToOBJ( const cvt::String& file, const cvt::SceneMesh& mesh  ) const
        {
            FILE* f = fopen( file.c_str(), "wb" );

            ROS_INFO( "MESH: vertices -> %ul, normals -> %ul, faces -> %ul", mesh.vertexSize(), mesh.normalSize(), mesh.faceSize() );

            for( size_t idx = 0; idx < mesh.vertexSize(); idx++ ) {
                cvt::Vector3f vtx = mesh.vertex( idx );
                fprintf( f, "v %f %f %f\n", vtx.x, vtx.y, vtx.z );
            }

            for( size_t idx = 0; idx < mesh.normalSize(); idx++ ) {
                cvt::Vector3f vtx = mesh.normal( idx );
                fprintf( f, "vn %f %f %f\n", vtx.x, vtx.y, vtx.z );
            }

            const unsigned int* faces = mesh.faces();
            for( size_t idx = 0; idx < mesh.faceSize(); idx++ ) {
                fprintf( f, "f %d//%d %d//%d %d//%d\n", *( faces) + 1, *( faces) + 1,
                                                        *( faces + 1 ) + 1, *( faces + 1 ) + 1,
                                                        *( faces + 2 ) + 1, *( faces + 2 ) + 1 );
                faces += 3;
            }

            fclose( f );
        }
};

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "tsdf_meshing");

    ros::NodeHandle nh( "~" );

    bool useCalibFile = false;
    std::string filename = "xtion_rgb.xml";

    nh.param<std::string>( "calib_file", filename, filename );
    nh.param<bool>( "use_calib_file", useCalibFile, useCalibFile );
    ROS_INFO( "Using Calibration file: %d", useCalibFile );

    if( useCalibFile ){
        try {
            // load calibration
            cvt::String resourcePath;
            resourcePath.sprintf( "%s/resources/%s", ros::package::getPath( "cvt_ros" ).c_str(), filename.c_str() );
            ROS_INFO( "Calibration file: %s", resourcePath.c_str() );
            cvt::CameraCalibration calib;
            calib.load( resourcePath.c_str() );

            TSDFMeshing mesher( calib.intrinsics() );
            ros::spin();

        } catch( cvt::Exception& e ){
            ROS_ERROR( "%s", e.what() );
            return 1;
        }
    } else {
        TSDFMeshing mesher;
        ros::spin();
    }

    return 0;
}
