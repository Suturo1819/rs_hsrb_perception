#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

using namespace uima;


class RegionAnnotator : public Annotator
{

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
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
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    for (auto &cluster : clusters) {
      std::vector<rs::PoseAnnotation> poses;
      cluster.annotations.filter(poses);
      if(!poses.empty()) {
        rs::StampedPose pose = poses[0].world();
        std::vector<rs::Region> regions;
        scene.annotations.filter(regions);
        for(rs::Region region : regions) {
          auto regionPos = region.transform().translation();
          auto clusterPos = pose.translation();
          double x = std::abs(regionPos[0] - clusterPos[0]);
          double y = std::abs(regionPos[1] - clusterPos[1]);
          double z = std::abs(regionPos[2] - clusterPos[2]);
          outInfo(region.name() << " Origin: " << regionPos[0]<<","<<regionPos[1]<<","<<regionPos[2]);
          outInfo("Cluster Origin: " << clusterPos[0]<<","<<clusterPos[1]<<","<<clusterPos[2]);
          outInfo("Point: " << x << "," << y << "," << z);
          outInfo("Region Frustum: " << region.height()/2 <<","<< region.width()/2 <<","<< region.depth()/2);
          if(x < region.height()/2 && y < region.width()/2 && z < region.depth()/2) {
            cluster.region.set(region.name());
          }
        }
      }
    }
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RegionAnnotator)