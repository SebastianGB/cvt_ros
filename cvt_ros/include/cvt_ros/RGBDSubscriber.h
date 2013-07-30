#ifndef CVT_ROS_RGBDSUBSCRIBER_H
#define CVT_ROS_RGBDSUBSCRIBER_H

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cvt/gfx/Image.h>

namespace cvt_ros {

    class RGBDSubscriber {
        public:
            RGBDSubscriber();
            RGBDSubscriber( const cvt::Matrix3f& calib );
            virtual ~RGBDSubscriber();

        private:
            typedef message_filters::Subscriber<sensor_msgs::Image> FilteredImageSubType;
            typedef message_filters::Subscriber<sensor_msgs::CameraInfo> FilteredCamInfoSubType;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyType;
            typedef message_filters::Synchronizer<SyncPolicyType> CamSyncType;

            CamSyncType                     _camSync;
            FilteredImageSubType            _colorImageSub;
            FilteredImageSubType            _depthImageSub;
            FilteredCamInfoSubType          _camInfoSub;

            void setupSubscribers();

        protected:
            std_msgs::Header                _rgbHeader;
            std_msgs::Header                _depthHeader;

            cvt::Matrix3f                   _intrinsics;
            cvt::Image                      _curRGB;
            cvt::Image                      _curDepth;

            virtual void camInfoCallback( const sensor_msgs::CameraInfoConstPtr& camInfo );
            virtual void imageMessageCallback( const sensor_msgs::ImageConstPtr& colorImageMsg,
                                               const sensor_msgs::ImageConstPtr& depthImageMsg );
            virtual void imageCallback( const cvt::Image& rgb, const cvt::Image& depth ) = 0;

    };
}

#endif
