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
          std::string shape = get_shape(cluster);
          std::ostringstream output;
          if(shape == "round") {
              output<<"I saw the apple or pair!";
              p->publish(output.str());
          } else if(shape == "cylinder") {
              output<<"I saw the Pringles can!";
              p->publish(output.str());
          }
      }
      return UIMA_ERR_NONE;
    }


    //TODO: This would be so much prettier with a lambda
    std::string get_shape(rs::Cluster cluster) {
        std::vector<rs::Shape> shapeAnnots;
        cluster.annotations.filter(shapeAnnots);
        if(!shapeAnnots.empty())
            return shapeAnnots[0].shape.get();
        return "";
    }

    std::vector<float> get_color(rs::Cluster cluster) {
        std::vector<rs::ColorHistogram> shapeAnnots;
        cluster.annotations.filter(shapeAnnots);
        std::vector<float> colors = std::vector<float>();
        if(!shapeAnnots.empty()) {
            rs::Mat color = shapeAnnots[0].hist.get();
            colors.push_back(color.data.get()[color.rows.get()/2]);
        }
        return colors;
    }




};


// This macro exports an entry point that is used to create the annotator.
MAKE_AE(NaiveObjectRecognition)