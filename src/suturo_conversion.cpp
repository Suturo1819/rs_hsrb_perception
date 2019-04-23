/**
 * Some conversation functions
 * Similiar to the rs::conversion paradigm
 * @author: Fenja Kollasch
 */
// ROS
#include <rs_hsrb_perception/suturo_conversion.h>
#include "../../../../../../../opt/ros/kinetic/include/std_msgs/ColorRGBA.h"
#include "../../../../../../../opt/ros/kinetic/include/geometry_msgs/PoseStamped.h"

namespace rs_hsrb_perception {
    namespace conversion {

        template<typename T>
        void from(const uima::FeatureStructure &fs, T &output)
        {
            std::string msg = std::string("no conversion for type '") + typeid(T).name() + "' defined";
            outError(msg);
            throw rs::conversion::ConversionException(msg);
        }

        template<typename T>
        uima::FeatureStructure to(uima::CAS &cas, const T &input)
        {
            std::string msg = std::string("no conversion for type '") + typeid(T).name() + "' defined";
            outError(msg);
            throw rs::conversion::ConversionException(msg);
        }

        template<>
        void from(const uima::FeatureStructure &fs, geometry_msgs::PoseStamped &output)
        {
            rs::StampedPose pose(fs);
            const std::vector<double> &rotation = pose.rotation.get();
            const std::vector<double> &translation = pose.translation.get();

            //rot matrix or quaternion (for backwards compatibility with old logs)
            assert((rotation.size() == 9 || rotation.size() == 4) && translation.size() == 3);

            // Pose infos
            output.pose.position.x = translation[0];
            output.pose.position.y = translation[1];
            output.pose.position.z = translation[2];
            output.pose.orientation.x = rotation[0];
            output.pose.orientation.y = rotation[1];
            output.pose.orientation.z = rotation[2];
            output.pose.orientation.w = rotation[3];

            // Header infos
            output.header.frame_id = pose.frame.get();
            output.header.stamp.sec = pose.timestamp.get()/1000000000;
            output.header.stamp.nsec = pose.timestamp.get();

        }

        template<>
        uima::FeatureStructure to(uima::CAS &cas, const geometry_msgs::PoseStamped &input)
        {
            rs::Pose pose = rs::create<rs::Pose>(cas);

            std::vector<double> quatVec(4), transVec(3);

            auto &rot = input.pose.orientation;
            auto &trans = input.pose.position;

            quatVec[0] = rot.x;
            quatVec[1] = rot.y;
            quatVec[2] = rot.z;
            quatVec[3] = rot.w;

            transVec[0] = trans.x;
            transVec[1] = trans.y;
            transVec[2] = trans.z;

            pose.rotation.set(quatVec);
            pose.translation.set(transVec);

            return pose;
        }

        u_int decode_shape(std::vector<rs::Shape> shapes) {
            bool box = false;
            bool round = false;
            bool flat = false;
            for(auto shape : shapes) {
                box = shape.shape() == "box" ? true : box;
                round = shape.shape() == "round" ? true : round;
                flat = shape.shape() == "flat" ? true : flat;
            }

            if(box) {
                if(round) {
                    // Cylinder
                    return 2;
                }
                // Box
                return 1;
            }
            if(round) {
               // Sphere
               return 3;
            }
            if(flat) {
                // Plane
                return 5;
            }
            return 0;
        }

        void makeObjectDetectionData(geometry_msgs::PoseStamped &pose, rs::Geometry &geometry, u_int shape, std::string region,
                std::string &objClass, float confidence, std_msgs::ColorRGBA &c,  suturo_perception_msgs::ObjectDetectionData &odd) {

//            std_msgs::ColorRGBA c;
//            c.r = r;
//            c.g = g;
//            c.b = b;
//            c.a = 1;


            odd.pose = pose;
            auto boundingBox = geometry.boundingBox();
            odd.width = boundingBox.width();
            odd.depth= boundingBox.height();
            odd.height = boundingBox.depth();
            odd.shape = shape;
            odd.region = region;
            odd.name = "Object (" + objClass + ")";
            odd.obj_class = objClass;
            odd.confidence = confidence;
            odd.color = c;


        }



    }
}

