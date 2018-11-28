#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <RosPublisher.h>

using namespace uima;


class NaiveObjectRecognition : public Annotator
{

public:

    TyErrorId initialize(AnnotatorContext &ctx)
    {
      outInfo("Hi there. I'm going to evaluate the view of our dearest HSR robot.");
      return UIMA_ERR_NONE;
    }

    TyErrorId destroy()
    {
      outInfo("destroy");
      return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
    {
      rs::SceneCas cas(tcas);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
      cas.get(VIEW_CLOUD,*cloud_ptr);

      rs::Scene scene = cas.getScene();
      std::vector<rs::Cluster> clusters;
      scene.identifiables.filter(clusters);
      RosPublisher *p = new RosPublisher("rs_hrsb_perception/naive_object_recognition");
      for(auto &cluster : clusters) {
          std::string shape = get_shape(cluster).shape.get();
        if(shape == "round") {
            p->publish("orange;");
        } else if(shape == "box") {
            p->publish("cereals");
        }
      }
      return UIMA_ERR_NONE;
    }


    rs::Shape get_shape(rs::Cluster cluster) {
        std::vector<rs::Shape> shapeAnnots;
        cluster.annotations.filter(shapeAnnots);
        return shapeAnnots[0];
    }


};


// This macro exports an entry point that is used to create the annotator.
MAKE_AE(NaiveObjectRecognition)