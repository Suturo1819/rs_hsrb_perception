//
// See MsgPublisher.h for reference
// @author Fenja Kollasch
//

#include <rs/utils/output.h>
#include "../include/rs_hsrb_perception/MsgPublisher.h"

MsgPublisher::MsgPublisher() : nh("~") {
    this->publisher = this->nh.advertise<ObjectDetectionData>("/suturo_perception/object_detection", 1000);
}

void MsgPublisher::publish(const ObjectDetectionData msg) {
    this->publisher.publish(msg);
    outInfo("I totally published the message " << msg.name);
}