//
// Created by Fenja on 26.11.18.
//

#ifndef PROJECT_ROSPUBLISHER_H
#define PROJECT_ROSPUBLISHER_H


#include "std_msgs/String.h"
#include "../../../../../../opt/ros/kinetic/include/ros/ros.h"

class RosPublisher {

private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    std::string message;
public:
    RosPublisher(std::string topic);
    ~RosPublisher();
    void publish(const std::string message);
};


#endif //PROJECT_ROSPUBLISHER_H
