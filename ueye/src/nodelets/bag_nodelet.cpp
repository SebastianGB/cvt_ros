// -- ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <rosbag/bag.h>
#include <ros/topic.h>
#include <ros/time.h>

#include <boost/foreach.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread.hpp>

// messages that can be logged
#include <tf/tfMessage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iafc_msgs/FSDFusedData.h>
#include <iafc_msgs/HighlevelCommand.h>
#include <iafc_msgs/HighLevelData1000.h>
#include <iafc_msgs/HighLevelData333.h>
#include <iafc_msgs/HighLevelData50.h>
#include <iafc_msgs/Keyframe.h>
#include <iafc_msgs/PositionCommand.h>
#include <iafc_msgs/VisionMeasurement.h>

namespace ueye_cam {

template <class T>
class SubscribeFunctor {
public:

    /** @brief  Subscriber callback as parametrized functor.
      * The name of the topic to subscribe to is "shrinkwrapped" with the
      * functor, because ROS only allows one parameter for the "operator()".
      */
    SubscribeFunctor(const std::string& topicName,
                     rosbag::Bag& bag): topicName_(topicName), bag_(bag){}

    void operator()(const T& message) {
        bag_.write(topicName_, ros::Time::now(), message);
    }

private:
    const std::string topicName_;
    rosbag::Bag& bag_;
};


/** @brief Predicate used for std::find algorithm, to search the iterable for
  *     the string specified in the ctor.
  */
class Equals {
public:
    Equals(std::string compareAgainstThis) :
        compareAgainstThis_(compareAgainstThis) {}

    bool operator()(const ros::master::TopicInfo compareMe) {
        return (compareMe.name == compareAgainstThis_);
    }

private:
    const std::string& compareAgainstThis_;
};


/** @brief Logging nodelet that avoids data copying by residing in the same
  * process and thus sharing the same memory.
  * The idea is that you can pass arbitrary command line parameters to this
  * and it will log all the topics specified there.
  */
class ROSBagLogNodelet : public nodelet::Nodelet {
friend class OnInitThreadCallable;
public:
    ROSBagLogNodelet() {}
    virtual ~ROSBagLogNodelet() { bag.close(); onInitThread.join(); }
    virtual void onInit();

private:
    rosbag::Bag                 bag;
    static const std::string    bag_file_name;

    template <typename T>
    ros::Subscriber subscribehelper(    const std::string& topicname);
    ros::Subscriber subscribe_to_topic( const std::string& topicname,
                                        const std::string& data_type);
    std::vector<ros::Subscriber>    subscribers_;
    boost::thread                   onInitThread;
};

const std::string ROSBagLogNodelet::bag_file_name(
        to_simple_string(ros::Time::now().toBoost()) + "_log.bag");



PLUGINLIB_DECLARE_CLASS( ueye_cam, ROSBagLogNodelet,
                         ueye_cam::ROSBagLogNodelet,
                         nodelet::Nodelet )

/** @brief  Subscribe to topics. Type switching is done here (see below)
  *
  *     **************************************
  *     * Add the loggable data types here ! *
  *     **************************************
  *
  *     As C++ is very picky about types, and ROS messages are not sharing
  *     a common base-class (polymorphy is no option hence) there has to be
  *     a hard-coded type switching place somewhere, and it is here ;)
  *
  * @param      topicname   Name of the topic to subscribe to
  * @param      topic_type  Data type of the topic
  * @returns    The created subscriber (might come in handy)
  */
ros::Subscriber ROSBagLogNodelet::subscribe_to_topic(
        const std::string& topicname, const std::string& topic_type)
{
    if (topic_type == "sensor_msgs/Image") {
        // doesnt work to pass a std_msgs::StringConstPtr here!
        // the subscribehelper does that for you though.
        return subscribehelper<sensor_msgs::Image>(topicname);
    }
    else if (topic_type == "tf/tfMessage") {
        return subscribehelper<tf::tfMessage>(topicname);
    }
    else if (topic_type == "sensor_msgs/CameraInfo") {
        return subscribehelper<sensor_msgs::CameraInfo>(topicname);
    }
    else if (topic_type == "iafc_msgs/FSDFusedData") {
        return subscribehelper<iafc_msgs::FSDFusedData>(topicname);
    }
    else if (topic_type == "iafc_msgs/HighlevelCommand") {
        return subscribehelper<iafc_msgs::HighlevelCommand>(topicname);
    }
    else if (topic_type == "iafc_msgs/HighLevelData1000") {
        return subscribehelper<iafc_msgs::HighLevelData1000>(topicname);
    }
    else if (topic_type == "iafc_msgs/HighLevelData333") {
        return subscribehelper<iafc_msgs::HighLevelData333>(topicname);
    }
    else if (topic_type == "iafc_msgs/HighLevelData50") {
        return subscribehelper<iafc_msgs::HighLevelData50>(topicname);
    }
    else if (topic_type == "iafc_msgs/Keyframe") {
        return subscribehelper<iafc_msgs::Keyframe>(topicname);
    }
    else if (topic_type == "iafc_msgs/PositionCommand") {
        return subscribehelper<iafc_msgs::PositionCommand>(topicname);
    }
    else if (topic_type == "iafc_msgs/VisionMeasurement") {
        return subscribehelper<iafc_msgs::VisionMeasurement>(topicname);
    }

    else {
        std::cerr << "Unknown data type encountered: " << topic_type <<
                     std::endl;
        return ros::Subscriber();
    }
}

// small subscribe helper to keep the interface more uniform and the type
// switching routine more concise.
template <typename T>
ros::Subscriber ROSBagLogNodelet::subscribehelper(
        const std::string& topicname)
{
    ros::NodeHandle& nh(getPrivateNodeHandle());

    return nh.subscribe<T>(topicname, 1,
        SubscribeFunctor<boost::shared_ptr<T const> >(topicname, bag));
}


class OnInitThreadCallable {
public:
    OnInitThreadCallable(ROSBagLogNodelet& rbn) : rbn_(rbn) {    }

    /** @brief  Init the nodelet. The following steps are performed:
      * - Open a bag file for writing
      * - List all topics with their respective types from the ROS Master
      * - Search in this obtained list with all given command line arguments
      *     passed to the nodelet. For each command line parameter the type if
      *     looked up in the obtained list, and then a matching callback is hope-
      *     fully existing.
      * - Create a subscriber for the respective topic
      *
      * @note   Because usually this nodelet is started among others in a launch
      *         file, and because launch files do not guarantee sequential start
      *         waiting for topics had to be implement explicitly.
      */
    void operator ()() {
        std::cout << "Waiting for all topics to be advertised..." << std::endl;

        // search in the topic list for all specified topics on the command line
        // and subscribe to the found ones. Naive 0(n^2) implementation, but too
        // lazy to implement more efficient search for those few topics.
        std::vector<std::string>::const_iterator it(NULL);

        // prevent double printing, see below
        std::map<std::string, std::string> found_topics;

        for (it = rbn_.getMyArgv().begin(); it != rbn_.getMyArgv().end(); ++it){

            bool found(false);
            for (int i(0); i != 10; ++i) {

                const std::string& needle(*it);
                // query the master server for all topics to get the topic types
                ros::master::V_TopicInfo topicInfos;
                ros::master::V_TopicInfo::const_iterator cit;
                if (!ros::master::getTopics(topicInfos)) {
                    std::cerr << "Failed getting topic info from master." <<
                                 std::endl;
                    exit(-1);
                }

                cit = std::find_if(topicInfos.begin(), topicInfos.end(),
                                   Equals(needle));
                if (cit != topicInfos.end()) {

                    if (found_topics.find(needle) != found_topics.end()) {
                        // topic was found before, do nothing
                    } else {
                        std::cout << "Topic " << needle << " found." <<
                                     std::endl;
                        found_topics[cit->name] = cit->datatype;
                    }
                    found = true;
                    break; // found the topic among active topics
                } else {
                    std::cout << "Could not subscribe to topic " << needle <<
                                 ": topic not found, retrying..." << std::endl;
                    boost::this_thread::sleep(
                                boost::posix_time::milliseconds(1000));
                } // found topic
            } // retry finding 10 times, 1 sec pause
            if (!found) {
                std::cout << "Re-tried 10 times, giving up and terminating:(" <<
                             std::endl;
                exit(-2);
            }
        } // for all topics to be found (specified on cmdline)

        std::cout << "Successfully waited for topics, subscribing..." <<
                     std::endl;

        for (it = rbn_.getMyArgv().begin(); it != rbn_.getMyArgv().end(); ++it){

            const std::string& needle(*it); // search for this one

            // found a hit, extract topic type
            std::cout << "Callback found for topic " << needle << " (" <<
                         found_topics[needle] << ")" << std::endl;

            rbn_.subscribers_.push_back(
                        rbn_.subscribe_to_topic(needle, found_topics[needle]));
        } // for all cmd line args
        std::cout << "Subscribed to all topics that could be found." <<
                     std::endl;

        // open bag file for writing
        try {
            rbn_.bag.open(rbn_.bag_file_name, rosbag::bagmode::Write);
        } catch (const rosbag::BagException& e) {
            std::cerr << "Error when opening bagfile" << rbn_.bag_file_name <<
                         " :" << e.what();
            return;
        }
    } // operator()()

private:
    ROSBagLogNodelet& rbn_;
};

/** @brief  Init the nodelet. Code has been move to
  * @see OnInitThreadCallable::operator() so it runs as an own thread.
  * This has been done because onInit should not block.
  */
void ROSBagLogNodelet::onInit()
{
    // onInit() must return, and should not block or do significant work!
    onInitThread = boost::thread(OnInitThreadCallable(*this));
}

} // namespace
