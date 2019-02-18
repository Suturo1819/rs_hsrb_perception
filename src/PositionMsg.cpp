#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <geometry_msgs/PoseStamped.h>
#include <rs_hsrb_perception/types/all_types.h>
#include "../include/rs_hsrb_perception/MsgPublisher.h"

#include <tf/transform_listener.h>

using namespace uima;
using namespace suturo_perception_msgs;
using namespace geometry_msgs;

class PositionMsg : public Annotator {

private:
    MsgPublisher *msgPublisher;
    tf::TransformListener tl;
public:

    TyErrorId initialize(AnnotatorContext &ctx) {
        outInfo("initialize");
        msgPublisher = new MsgPublisher();
        return UIMA_ERR_NONE;
    }

    TyErrorId destroy() {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec) {
        rs::SceneCas cas(tcas);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cas.get(VIEW_CLOUD, *cloud_ptr);

        rs::Scene scene = cas.getScene();
        std::vector<rs::ObjectHypothesis> clusters;
        scene.identifiables.filter(clusters);
        for (auto &cluster : clusters) {
            std::vector<rs::Shape> shapes;
            get_annotations<rs::Shape>(cluster, shapes);
            std::vector<rs::Geometry> geometry;
            get_annotations<rs::Geometry>(cluster, geometry);
            if (!shapes.empty() && !geometry.empty()) {
                outInfo("I saw some Shapes and BoundingBoxes. Leave it to me!");
                std::vector<rs::PoseAnnotation> poses;
                cluster.annotations.filter(poses);
                for (auto &pose : poses) {
                    outInfo("Creating ROS msg for recognized object...");
                    ObjectDetectionData odd;
                    PoseStamped poseStamped;
                    rsPoseToGeoPose(pose.world.get(), poseStamped);
                    makeObjectDetectionData(poseStamped, geometry[0], shapes[0], odd);
                    msgPublisher->publish(odd);

                    tf::Stamped<tf::Pose> tfPose;
                    rsPoseToTfPose(pose.world.get(), tfPose);
                    tf::Stamped<tf::Pose> odom;
                    tl.transformPose("odom", tfPose, odom);
                    PoseStamped odomPose;
                    tfPoseToGeoPose(odom, odomPose);
                    ObjectDetectionData odomData;
                    makeObjectDetectionData(odomPose, geometry[0], shapes[0], odomData);
                    msgPublisher->publish(odomData);
                }
            } else  {
                outInfo("No shapes were recognized");
            }
        }
        return UIMA_ERR_NONE;
    }

    template<class T> void get_annotations(rs::ObjectHypothesis cluster, std::vector<T> &annotations) {
        cluster.annotations.filter(annotations);
    }

    u_int shape_map(std::string shape) {
        if (shape == "round" || shape == "cylinder") {
            return ObjectDetectionData::CYLINDER;
        } else if (shape == "box") {
            return ObjectDetectionData::BOX;
        }
        return ObjectDetectionData::MISC;
    }

    void makeObjectDetectionData(PoseStamped pose, rs::Geometry geometry, rs::Shape shape, ObjectDetectionData &odd) {
        odd.shape = shape_map(shape.shape());
        odd.pose = pose;
        auto boundingBox = geometry.boundingBox();
        odd.width = boundingBox.width();
        odd.height = boundingBox.height();
        odd.depth = boundingBox.depth();
        odd.name = "Object (" + pose.header.frame_id + ")";
    }

    // A bunch of pose type transformation functions
    // Todo: Think of a smarter way to transform pose types
    void rsPoseToGeoPose(rs::StampedPose pose, PoseStamped &geoPose) {
        auto translation = pose.translation.get();
        auto rotation = pose.rotation.get();

        // Pose infos
        geoPose.pose.position.x = translation[0];
        geoPose.pose.position.y = translation[1];
        geoPose.pose.position.z = translation[2];
        geoPose.pose.orientation.x = rotation[0];
        geoPose.pose.orientation.y = rotation[1];
        geoPose.pose.orientation.z = rotation[2];
        geoPose.pose.orientation.w = rotation[3];

        // Header infos
        geoPose.header.frame_id = pose.frame.get();
        geoPose.header.stamp.sec = pose.timestamp.get()/1000000000;
        geoPose.header.stamp.nsec = pose.timestamp.get();
    }

    void rsPoseToTfPose(rs::StampedPose pose, tf::Stamped<tf::Pose> &tfPose) {
        auto translation = pose.translation.get();
        auto rotation = pose.rotation.get();

        // Pose infos
        tfPose.getOrigin()[0] = translation[0];
        tfPose.getOrigin()[1] = translation[1];
        tfPose.getOrigin()[2] = translation[2];
        tfPose.getRotation().setX(rotation[0]);
        tfPose.getRotation().setY(rotation[1]);
        tfPose.getRotation().setZ(rotation[2]);
        tfPose.getRotation().setW(rotation[3]);

        // Header infos
        tfPose.frame_id_ = pose.frame.get();
        tfPose.stamp_.sec = pose.timestamp.get()/1000000000;
        tfPose.stamp_.nsec = pose.timestamp.get();
    }

    void tfPoseToGeoPose(tf::Stamped<tf::Pose> tfPose, PoseStamped &geoPose) {

        geoPose.pose.position.x = tfPose.getOrigin()[0];
        geoPose.pose.position.y = tfPose.getOrigin()[1];
        geoPose.pose.position.z = tfPose.getOrigin()[2];

        tfPose.getRotation().setX(tfPose.getRotation().x());
        tfPose.getRotation().setX(tfPose.getRotation().y());
        tfPose.getRotation().setX(tfPose.getRotation().z());
        tfPose.getRotation().setX(tfPose.getRotation().w());

        geoPose.header.frame_id = tfPose.frame_id_;
        geoPose.header.stamp.sec = tfPose.stamp_.sec;
        geoPose.header.stamp.nsec = tfPose.stamp_.nsec;

    }

    void tfPoseToRsPose(tf::Stamped<tf::Pose> tfPose, rs::StampedPose &rsPose) {

        rsPose.translation.get()[0] = tfPose.getOrigin()[0];
        rsPose.translation.get()[1] = tfPose.getOrigin()[1];
        rsPose.translation.get()[2] = tfPose.getOrigin()[2];

        rsPose.rotation.get()[0] =  tfPose.getRotation().x();
        rsPose.rotation.get()[1] =  tfPose.getRotation().y();
        rsPose.rotation.get()[2] =  tfPose.getRotation().z();
        rsPose.rotation.get()[3] =  tfPose.getRotation().w();

        rsPose.frame.set(tfPose.frame_id_);
        rsPose.timestamp.set(tfPose.stamp_.nsec);

    }


};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PositionMsg)