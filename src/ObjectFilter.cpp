#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

using namespace uima;


class ObjectFilter : public Annotator
{
private:
  float min_width, min_height, min_depth;
  float max_width, max_height, max_depth;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("min_width")) {
      ctx.extractValue("min_width", min_width);
    } else {
      min_width = 0;
    }
    if(ctx.isParameterDefined("min_height")) {
      ctx.extractValue("min_height", min_height);
    } else {
      min_height = 0;
    }
    if(ctx.isParameterDefined("min_depth")) {
      ctx.extractValue("min_depth", min_depth);
    } else {
      min_depth = 0;
    }
    if(ctx.isParameterDefined("max_width")) {
        ctx.extractValue("max_width", max_width);
    } else {
        max_width = 0;
    }
    if(ctx.isParameterDefined("max_height")) {
        ctx.extractValue("max_height", max_height);
    } else {
        max_height = 0;
    }
    if(ctx.isParameterDefined("max_depth")) {
        ctx.extractValue("max_depth", max_depth);
    } else {
        max_depth = 0;
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*cloud_ptr);
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    outInfo(clusters.size());
    for(auto &cluster : clusters) {
    	outInfo("Process clusters");
        std::vector<rs::Geometry> geo;
        cluster.annotations.filter(geo);
        if(geo.empty()) {
            continue;
        }
        rs::BoundingBox3D boundingBox = geo[0].boundingBox();
        bool min = (min_width <= 0 || boundingBox.width() >= min_width) &&
                   (min_height <= 0 || boundingBox.height() >= min_height) &&
                   (min_depth <= 0 || boundingBox.depth() >= min_depth);
        bool max = (max_width <= 0 || boundingBox.width() <= max_width) &&
                   (max_height <= 0 || boundingBox.height() <= max_height) &&
                   (max_depth <= 0 || boundingBox.depth() <= max_depth);
        outInfo("Min: " << min);
        outInfo("Max: " << max);
        cluster.publish.set(min && max);
    }

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ObjectFilter)
