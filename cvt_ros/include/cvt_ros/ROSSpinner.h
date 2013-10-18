#ifndef CVTROS_ROSSPINNER_H
#define CVTROS_ROSSPINNER_H

#include <cvt/gui/TimeoutHandler.h>
#include <cvt/gui/Application.h>

namespace cvt_ros
{
    class ROSSpinner : public cvt::TimeoutHandler
    {
        public:
            /**
             * @brief ROSSpinner
             * @param interval - callback interval in milliseconds
             */
            ROSSpinner( size_t interval ) :
                _timerId( cvt::Application::registerTimer( interval, this ) )
            {
            }

            ~ROSSpinner()
            {
                cvt::Application::unregisterTimer( _timerId );
            }

            void onTimeout()
            {
                if( ros::ok() ){
                    ros::spinOnce();
                } else {
                    cvt::Application::exit();
                }
            }

        private:
            uint32_t _timerId;
    };
}

#endif
