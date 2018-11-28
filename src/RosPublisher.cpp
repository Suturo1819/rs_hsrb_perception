//
// Created by suturo on 26.11.18.
//
#include <rs/utils/output.h>
#include "std_msgs/String.h"
#include "../include/RosPublisher.h"
#include "../../../../../../opt/ros/kinetic/include/ros/ros.h"

RosPublisher::RosPublisher(const std::string topic) {
    this->publisher = nh.advertise<std_msgs::String>(topic, 1000);
}

void RosPublisher::publish(const std::string message) {
    std::stringstream ss;
    ss << "Respectfully report: " << message;
    std_msgs::String msg;
    msg.data = ss.str();
    publisher.publish(msg);
    outInfo(msg.data.c_str());
    ros::spinOnce();
}