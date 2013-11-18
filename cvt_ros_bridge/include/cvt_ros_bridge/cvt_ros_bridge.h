#ifndef CVT_ROS_IMAGE_BRIDGE_H
#define CVT_ROS_IMAGE_BRIDGE_H

#include <cvt/gfx/Image.h>
#include <cvt/util/Exception.h>
#include <cvt/math/Matrix.h>
#include <cvt/vision/CameraCalibration.h>
#include <cvt/vision/StereoCameraCalibration.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/common.h>

/* generic helpers to convert between ros and iafc/cvt stuff */
namespace cvt_ros_bridge
{
	static inline const cvt::IFormat & imageFormatForMsg( const sensor_msgs::Image & imgMsg )
	{
		if( imgMsg.encoding == sensor_msgs::image_encodings::BAYER_RGGB8 )
			return cvt::IFormat::BAYER_RGGB_UINT8;
		
		if( imgMsg.encoding == sensor_msgs::image_encodings::BAYER_GRBG8 )
			return cvt::IFormat::BAYER_GRBG_UINT8;
		
		if( imgMsg.encoding == sensor_msgs::image_encodings::BAYER_GBRG8 )
			return cvt::IFormat::BAYER_GBRG_UINT8;

		if( imgMsg.encoding == sensor_msgs::image_encodings::RGBA8 ||
			imgMsg.encoding == sensor_msgs::image_encodings::RGB8 )
			return cvt::IFormat::RGBA_UINT8;

		if( imgMsg.encoding == sensor_msgs::image_encodings::BGRA8 ||
			imgMsg.encoding == sensor_msgs::image_encodings::BGR8 )
			return cvt::IFormat::BGRA_UINT8;

		if( imgMsg.encoding == sensor_msgs::image_encodings::MONO8 )
			return cvt::IFormat::GRAY_UINT8;

		if( imgMsg.encoding == sensor_msgs::image_encodings::MONO16 )
			return cvt::IFormat::GRAY_UINT16;

		if( imgMsg.encoding == sensor_msgs::image_encodings::YUV422 )
			return cvt::IFormat::UYVY_UINT8;
		
        if( imgMsg.encoding == sensor_msgs::image_encodings::TYPE_16UC1 )
			return cvt::IFormat::GRAY_UINT16;

		if( imgMsg.encoding == sensor_msgs::image_encodings::TYPE_32FC1 )
			return cvt::IFormat::GRAY_FLOAT;

		if( imgMsg.encoding == sensor_msgs::image_encodings::TYPE_32FC2 )
			return cvt::IFormat::GRAYALPHA_FLOAT;
        cvt::String msg;
        msg.sprintf( "No equivalent IFormat for ROS format %s", imgMsg.encoding.c_str() );
		throw CVTException( msg.c_str() );
	}

	static inline const std::string & msgEncodingForFormat( const cvt::IFormat & format )
	{
		switch ( format.formatID ){
			case cvt::IFORMAT_GRAY_UINT8:
				return sensor_msgs::image_encodings::MONO8;
			case cvt::IFORMAT_GRAY_UINT16:
				return sensor_msgs::image_encodings::MONO16;
			case cvt::IFORMAT_BAYER_RGGB_UINT8:
				return sensor_msgs::image_encodings::BAYER_RGGB8;
			case cvt::IFORMAT_BAYER_GRBG_UINT8:
				return sensor_msgs::image_encodings::BAYER_GRBG8;
			case cvt::IFORMAT_BAYER_GBRG_UINT8:
				return sensor_msgs::image_encodings::BAYER_GBRG8;
			case cvt::IFORMAT_RGBA_UINT8:
				return sensor_msgs::image_encodings::RGBA8;
			case cvt::IFORMAT_BGRA_UINT8:
				return sensor_msgs::image_encodings::BGRA8;
			case cvt::IFORMAT_UYVY_UINT8:
				return sensor_msgs::image_encodings::YUV422;
			case cvt::IFORMAT_YUYV_UINT8:
				return sensor_msgs::image_encodings::YUV422;
			case cvt::IFORMAT_GRAY_FLOAT:
				return sensor_msgs::image_encodings::TYPE_32FC1;
			case cvt::IFORMAT_GRAYALPHA_FLOAT:
				return sensor_msgs::image_encodings::TYPE_32FC2;
			default:
				throw CVTException( "No equivalent ROS format for cvt:Image Format" );
		}
	}

	static inline void image2RosMsg( const cvt::Image & img, sensor_msgs::Image & imgMsg )
	{
		size_t stride;
		const uint8_t * ptr = img.map( &stride );

		sensor_msgs::fillImage( imgMsg,
								msgEncodingForFormat( img.format() ),
								img.height(),
								img.width(),
								stride,
								ptr );
		img.unmap( ptr );
	}

	static inline void msg2Image( const sensor_msgs::Image & imgMsg, cvt::Image & img )
	{
		const cvt::IFormat & format = imageFormatForMsg( imgMsg );
		img.reallocate( imgMsg.width, imgMsg.height, format );

		size_t imgStride;
		uint8_t * p = img.map( &imgStride );

		cvt::SIMD * simd = cvt::SIMD::instance();
		if( imgMsg.encoding == sensor_msgs::image_encodings::BGR8 ||
			imgMsg.encoding == sensor_msgs::image_encodings::RGB8 ){
			size_t h = imgMsg.height;
			uint8_t * dst = p;
			const uint8_t * src = &imgMsg.data[ 0 ];
			while( h-- ){
				simd->Memcpy( dst, src, imgMsg.step );
				simd->Conv_XXXu8_to_XXXAu8( dst, src, imgMsg.width * 3 );
				dst += imgStride;
				src += imgMsg.step;
			}
		} else {
			if( imgStride == imgMsg.step ){
				simd->Memcpy( p, &imgMsg.data[ 0 ], imgStride * imgMsg.height );
			} else {
				size_t h = imgMsg.height;
				uint8_t * dst = p;
				const uint8_t * src = &imgMsg.data[ 0 ];
				while( h-- )
				{
					simd->Memcpy( dst, src, imgMsg.step );
					dst += imgStride;
					src += imgMsg.step;
				}
			}
		}

		img.unmap( p );
	}

	template <class T>
	static inline cvt::Matrix4<T> toCVTMatrix( const geometry_msgs::TransformStamped& t )
	{
		cvt::Quaternion<T> q( ( T )t.transform.rotation.x,
							  ( T )t.transform.rotation.y,
							  ( T )t.transform.rotation.z,
							  ( T )t.transform.rotation.w );
		cvt::Matrix4<T> m( q.toMatrix4() );
		m[ 0 ][ 3 ] = ( T )t.transform.translation.x;
		m[ 1 ][ 3 ] = ( T )t.transform.translation.y;
		m[ 2 ][ 3 ] = ( T )t.transform.translation.z;
		return m;
	}

    /**
     * @brief   Turn a ROS Camera Calibration message into a CVT camera calibration object.
     * @param calib  CVT camera calibration output
     * @param info   Camera info as received by ROS.
     */
    static inline void camInfoToCalib( cvt::CameraCalibration& calib, const sensor_msgs::CameraInfoConstPtr& info )
    {
        calib.setIntrinsics( ( float )info->K[ 0 ], ( float )info->K[ 4 ],
                             ( float )info->K[ 2 ], ( float )info->K[ 5 ] );

        // TODO: this is actually only for the rectified case ... 
        // "real" extrinsics are usually within the TF tree

        cvt::Matrix4f extrinsics;
        extrinsics.setIdentity();
        extrinsics[ 0 ][ 3 ] = ( float )( info->P[ 3 ] / info->K[ 0 ] ); // Tx
        calib.setExtrinsics( extrinsics );
        
        calib.setHeight( info->height );
        calib.setWidth( info->width );

        try {
            cvt::Vector3f radial;
            cvt::Vector2f tangential;
            radial.setZero();
            tangential.setZero();

            size_t distSize = info->D.size();
            for( size_t i = 0; i < 2 && i < distSize; i++ ){
                radial[ i ] = info->D[ i ];
            }
            for( size_t i = 2; i < 4 && i < distSize; i++ ){
                tangential[ i - 2 ] = info->D[ i ];
            }
            if( distSize == 5 )
                radial[ 4 ] = info->D[ 4 ];

            calib.setDistortion( radial, tangential );
        } catch ( const std::out_of_range& e ) {
            ROS_WARN( "Distortion coefficients not provided." );
        }
    }

    static inline void camInfoToStereoCalib( cvt::StereoCameraCalibration& calib, const sensor_msgs::CameraInfoConstPtr& infoL, const sensor_msgs::CameraInfoConstPtr& infoR )
    {
        cvt::CameraCalibration l, r;
        camInfoToCalib( l, infoL );
        camInfoToCalib( r, infoR );
        calib = cvt::StereoCameraCalibration( l, r );
    }

}

#endif
