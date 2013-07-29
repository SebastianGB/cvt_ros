#include <cvt/vision/StereoCameraCalibration.h>
#include <cvt_ros_bridge/cvt_ros_bridge.h>

using namespace cvt;

void dump( const cvt::String& camName,
		   const CameraCalibration& calib,
		   const Matrix3f& Rrect,
		   const Matrix4f& projection,
		   size_t w, size_t h )
{
	std::ofstream out;
	String fName;
	fName.sprintf( "%s.yaml", camName.c_str() );

	out.open( fName.c_str() );

	const Matrix3f& K = calib.intrinsics();
	const Vector3f& rad  = calib.radialDistortion();
	const Vector2f& tang = calib.tangentialDistortion();

	out << "image_width: " << w << "\n";
	out << "image_height: " << h << "\n";
	out << "camera_name: " << camName << "\n";
	out << "camera_matrix:\n";
	out << "  rows: 3\n";
	out << "  cols: 3\n";
	out << "  data: [";
	out << K[ 0 ][ 0 ] << ", " << K[ 0 ][ 1 ] << ", " << K[ 0 ][ 2 ] << ", "
		<< K[ 1 ][ 0 ] << ", " << K[ 1 ][ 1 ] << ", " << K[ 1 ][ 2 ] << ", "
		<< K[ 2 ][ 0 ] << ", " << K[ 2 ][ 1 ] << ", " << K[ 2 ][ 2 ] << "]\n";
	out << "distortion_model: plumb_bob\n";
	out << "distortion_coefficients:\n";
	out << "  rows: 1\n";
	out << "  cols: 5\n";
	out << "  cols: 5\n";
	out << "  data: [";
	out << rad[ 0 ] << ", "
		<< rad[ 1 ] << ", "
		<< tang[ 0 ] << ", "
		<< tang[ 1 ] << ", "
		<< rad[ 2 ] << "]\n";

	out << "rectification_matrix:\n";
	out << "  rows: 3\n";
	out << "  cols: 3\n";
	out << "  data: [";
	out << Rrect[ 0 ][ 0 ] << ", " << Rrect[ 0 ][ 1 ] << ", " << Rrect[ 0 ][ 2 ] << ", "
		<< Rrect[ 1 ][ 0 ] << ", " << Rrect[ 1 ][ 1 ] << ", " << Rrect[ 1 ][ 2 ] << ", "
		<< Rrect[ 2 ][ 0 ] << ", " << Rrect[ 2 ][ 1 ] << ", " << Rrect[ 2 ][ 2 ] << "]\n";

	out << "projection_matrix:\n";
	out << "  rows: 3\n";
	out << "  cols: 4\n";
	out << "  data: [";
	out << projection[ 0 ][ 0 ] << ", " << projection[ 0 ][ 1 ] << ", " << projection[ 0 ][ 2 ] << ", " << projection[ 0 ][ 3 ] << ", "
		<< projection[ 1 ][ 0 ] << ", " << projection[ 1 ][ 1 ] << ", " << projection[ 1 ][ 2 ] << ", " << projection[ 1 ][ 3 ] << ", "
		<< projection[ 2 ][ 0 ] << ", " << projection[ 2 ][ 1 ] << ", " << projection[ 2 ][ 2 ] << ", " << projection[ 2 ][ 3 ] << "]\n";

	out.close();
}

int main( int argc, char* argv[] ){
	try {
		if( argc < 5 ){
			std::cout << "usage: " << argv[ 0 ] << " <calibLeft> <calibRight> <width> <height>" << std::endl;
			return 0;
		}

		String file0( argv[ 1 ] );
		String file1( argv[ 2 ] );

		String s;
		s = argv[ 3 ];
		size_t w = s.toInteger();
		s = argv[ 4 ];
		size_t h = s.toInteger();

		CameraCalibration c0, c1;
		c0.load( file0 );
		c1.load( file1 );

		StereoCameraCalibration sCalib( c0, c1 ), tmp;

		Image il( w, h, IFormat::GRAYALPHA_FLOAT ), ir( w, h, IFormat::GRAYALPHA_FLOAT );
		sCalib.undistortRectify( tmp, il, ir, w, h, false );

		Matrix3f rl, rr;
		sCalib.rectificationMatrices( rl, rr );

		file0 = file0.substring( 0, file0.rfind( '.' ) );
		std::cout << "Writing file: " << file0 << std::endl;
		dump( file0, c0, rl, tmp.firstCamera().projectionMatrix(), w, h );

		file1 = file1.substring( 0, file1.rfind( '.' ) );
		std::cout << "Writing file: " << file1 << std::endl;
		dump( file1, c1, rr, tmp.secondCamera().projectionMatrix(), w, h );

	} catch ( Exception& e ){
		std::cout << e.what() << std::endl;
	}

	return 0;


}
