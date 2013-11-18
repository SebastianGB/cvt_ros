#ifndef CVT_ROS_STEREOSUBSCRIBER_H
#define CVT_ROS_STEREOSUBSCRIBER_H

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cvt/gfx/Image.h>
#include <cvt/vision/StereoCameraCalibration.h>

namespace cvt_ros {

    class StereoSubscriber {
        public:
            StereoSubscriber();
            virtual ~StereoSubscriber();

        private:
            typedef message_filters::Subscriber<sensor_msgs::Image> FilteredImageSubType;
            typedef message_filters::Subscriber<sensor_msgs::CameraInfo> FilteredCamInfoSubType;
            typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyType;
            typedef message_filters::sync_policies::ExactTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> CInfoSyncPolicyType;
            typedef message_filters::Synchronizer<SyncPolicyType>       CamSyncType;
            typedef message_filters::Synchronizer<CInfoSyncPolicyType>  CamInfoSyncType;

            CamSyncType                     _camSync;
            CamInfoSyncType                 _camInfoSync;
            FilteredImageSubType            _leftImageSub;
            FilteredImageSubType            _rightImageSub;
            FilteredCamInfoSubType          _leftCamInfoSub;
            FilteredCamInfoSubType          _rightCamInfoSub;

            void setupSubscribers();

        protected:
            std_msgs::Header                _leftHeader;
            std_msgs::Header                _rightHeader;
            sensor_msgs::ImageConstPtr      _leftMsg;
            sensor_msgs::ImageConstPtr      _rightMsg;

            sensor_msgs::CameraInfoConstPtr _leftCamInfo;
            sensor_msgs::CameraInfoConstPtr _rightCamInfo;

            cvt::StereoCameraCalibration    _calib;

            cvt::Image                      _left;
            cvt::Image                      _right;

            virtual void camInfoCallback( const sensor_msgs::CameraInfoConstPtr& lCamInfo,
                                          const sensor_msgs::CameraInfoConstPtr& rCamInfo );


            virtual void imageMessageCallback( const sensor_msgs::ImageConstPtr& leftImageMsg,
                                               const sensor_msgs::ImageConstPtr& rightImageMsg );

            virtual void imageCallback( const cvt::Image& left, const cvt::Image& right ) = 0;
            virtual void calibCallback( const cvt::StereoCameraCalibration calib ) = 0;
    };
}

#endif
