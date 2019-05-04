/*
 * Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@tzi.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "uima/api.hpp"

#include <rs/scene_cas.h>
#include <rs/utils/exception.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

#include <ros/package.h>

#include <termios.h>
#include <signal.h>
#include <sys/stat.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>

using namespace uima;



class SaveClusterCloudsAndImages: public Annotator
{
private:
    std::string objectName;
    std::string fullPath;
    int angle;
    int padding;
    int idx;


    cv::Mat color, depth;
public:

    TyErrorId initialize(AnnotatorContext &ctx)
    {
        outInfo("initialize");
        if(ctx.isParameterDefined("objectName"))
        {
            ctx.extractValue("objectName", objectName);
        }
        else
        {
            outInfo("YOU DID NOT DEFINE A NAME FOR THE CLUSTER! SHUTTING DOWN");
            ros::shutdown();
        }
        if(ctx.isParameterDefined("angle"))
        {
            ctx.extractValue("angle", angle);
        }
        if(ctx.isParameterDefined("padding"))
        {
            outInfo("padding wurde gewaelt >>>>>>>>>>>>>>>>>>.");
            ctx.extractValue("padding", padding);
        }

        fullPath = ros::package::getPath("rs_hsrb_perception") + "/data/" + objectName;
        outInfo("FULLPATH WURDE GESETTET>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>..");
        struct stat fileStat;

        if(stat(fullPath.c_str(), &fileStat))
        {
            mkdir(fullPath.c_str(), 9999);
        }

        outInfo("saving images to: "<<fullPath);
        idx = 0;
        return (TyErrorId) UIMA_ERR_NONE;
    }

    TyErrorId typeSystemInit(TypeSystem const &type_system)
    {
        return (TyErrorId) UIMA_ERR_NONE;
    }

    TyErrorId destroy()
    {

        return (TyErrorId) UIMA_ERR_NONE;
    }


    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
    {
        // declare variables for kinect data
        outInfo("process start");
        rs::StopWatch clock;
        rs::SceneCas cas(tcas);
        rs::Scene scene = cas.getScene();
        std::vector<rs::ObjectHypothesis> clusters;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cas.get(VIEW_COLOR_IMAGE_HD, color);
        cas.get(VIEW_DEPTH_IMAGE_HD, depth);
        cas.get(VIEW_CLOUD, *cloudPtr);

        scene.identifiables.filter(clusters);
        if(clusters.size() == 1)
        {
            rs::ImageROI image_rois = clusters[0].rois.get();

            cv::Mat mask;
            cv::Rect roi;
            rs::conversion::from(image_rois.roi_hires(), roi);
            outInfo(roi.x);
            outInfo(roi.y);
            outInfo(roi.height);
            outInfo(roi.width);
            outInfo(padding);
            roi.x -= 0.1*padding;
            roi.y -=  0.1*padding;
            roi.height +=  5* padding;
            roi.width +=  5* padding;
            mask = cv::Mat::zeros(roi.height,roi.width,CV_8UC1);


            rs::conversion::from(image_rois.mask_hires(), mask);

            std::stringstream ss_rgb;
            std::stringstream ss_depth;
            std::stringstream ss_mask, ss_location;
            std::stringstream ss_pcd;
            std::fstream filestream;

            ss_rgb << fullPath << "/" << objectName << "_" << angle << "_" << idx << "_crop.png";
            ss_depth << fullPath << "/" << objectName << "_" << angle << "_" << idx << "_depthcrop.png";
            ss_mask << fullPath << "/" << objectName << "_" << angle << "_" << idx << "_mask.png";
            ss_location << fullPath << "/" << objectName << "_" << angle << "_" << idx << "_loc.txt";
            ss_pcd << fullPath << "/" << objectName << "_" << angle << "_" << idx << ".pcd";
            outInfo(ss_rgb.str());
            cv::imwrite(ss_rgb.str(), cv::Mat(color, roi));
            cv::imwrite(ss_depth.str(), cv::Mat(depth, roi));
            //todo: make mask the same size as color and depth

            cv::Scalar value(0,0,0);
            cv::Mat paddedMask;
            cv::copyMakeBorder(mask,paddedMask,padding,padding,padding,padding,cv::BORDER_CONSTANT,value);
            cv::imwrite(ss_mask.str(), paddedMask);




            filestream.open(ss_location.str(), std::ios::out);
            filestream << roi.x << "," << roi.y;
            filestream.flush();
            filestream.close();

            //save the point cloud
            rs::ObjectHypothesis &cluster = clusters[0];
            if(cluster.points.has())
            {
                outInfo("save the point cloud");
                pcl::PointIndicesPtr indices(new pcl::PointIndices());
                rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

                pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
                ei.setInputCloud(cloudPtr);
                ei.setIndices(indices);
                ei.filter(*cluster_cloud);
                //pcl::io::savePCDFileBinaryCompressed(ss_pcd.str(), *cluster_cloud);
            }
        }
        idx++;
        return (TyErrorId) UIMA_ERR_NONE;
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SaveClusterCloudsAndImages)