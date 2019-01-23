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

using namespace uima;
using namespace suturo_perception_msgs;
using namespace geometry_msgs;

class PositionMsg : public Annotator {

private:
    MsgPublisher *msgPublisher;
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
        std::vector<rs::Cluster> clusters;
        scene.identifiables.filter(clusters);
        for (auto &cluster : clusters) {
            auto shapes = get_annotations<rs::Shape>(cluster);
            auto bbs = get_annotations<rs_hsrb_perception::BoundingBox>(cluster);
            if (!shapes.empty() && !bbs.empty()) {
                std::vector<rs::PoseAnnotation> poses;
                cluster.annotations.filter(poses);
                for (auto &pose : poses) {
                    outInfo("Creating ROS msg for recognized object...");
                    ObjectDetectionData odd = ObjectDetectionData();
                    odd.name = "Object";
                    odd.shape = shape_map(shapes[0].shape());
                    PoseStamped poseStamped = PoseStamped();
                    rsPoseToGeoPose(pose.world.get(), poseStamped);
                    odd.pose = poseStamped;
                    odd.width = bbs[0].width();
                    odd.height = bbs[0].height.get();
                    odd.depth = bbs[0].depth.get();
                    msgPublisher->publish(odd);
                }
            }
        }
        return UIMA_ERR_NONE;
    }

    template<class T> std::vector<T> get_annotations(rs::Cluster cluster) {
        std::vector<T> annotations;
        cluster.annotations.filter(annotations);
        return annotations;
    }

    u_int shape_map(std::string shape) {
        if (shape == "round" || shape == "cylinder") {
            return ObjectDetectionData::CYLINDER;
        } else if (shape == "box") {
            return ObjectDetectionData::BOX;
        }
        return ObjectDetectionData::MISC;
    }

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
        geoPose.header.stamp.sec = pose.timestamp.get();
    }


};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PositionMsg)