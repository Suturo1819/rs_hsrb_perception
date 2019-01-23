//
// Publishes the perceived data as a message to a specified ROS topic
// @author Fenja Kollasch
//

#include <ros/ros.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>

using namespace suturo_perception_msgs;

class MsgPublisher {
private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
public:
    MsgPublisher();
    void publish(ObjectDetectionData msg);
};


