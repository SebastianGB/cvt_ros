#ifndef CVT_ROS_RGBDSUBSCRIBER_H
#define CVT_ROS_RGBDSUBSCRIBER_H

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <cvt/gfx/Image.h>
#include <ros/ros.h>

namespace cvt_ros {

    class ImageSubscriber {
        public:
            ImageSubscriber();
            virtual ~ImageSubscriber();

        private:
            image_transport::ImageTransport _it;

        protected:
            image_transport::Subscriber     _sub;
            std_msgs::Header                _header;
            cvt::Image                      _cur;

            virtual void imageMessageCallback( const sensor_msgs::ImageConstPtr& imageMsg );
            virtual void imageCallback( const cvt::Image& img ) = 0;

    };
}

#endif
