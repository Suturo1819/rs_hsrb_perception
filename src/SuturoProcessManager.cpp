/**
 * Modified RoboSherlock Process Manager
 * Optimized for the Suturo Perception Pipelines
 * @author Fenja Kollasch
 */
#include <rs_hsrb_perception/SuturoProcessManager.h>

SuturoProcessManager::SuturoProcessManager(ros::NodeHandle n, const std::string savePath)
:   RSProcessManager(true, false, RSProcessManager::KnowledgeEngineType::SWI_PROLOG, n, savePath),
    savePath(savePath)
{
    outInfo("A RoboSherlock process manager optimized for the Suturo perception was created.");
}



void SuturoProcessManager::init(std::string &xmlFile) {
    RSProcessManager::init(xmlFile, ros::package::getPath("robosherlock") + "/config/config.yaml", false, false);
}

void SuturoProcessManager::run(bool visualize) {
    if(!visualize) {
        visualizer_.stop();
    }
    signal(SIGINT, RSProcessManager::signalHandler);
    {
        std::lock_guard<std::mutex> lock(processing_mutex_);
        if(waitForServiceCall_)
        {
            usleep(100000);
        }
        else
        {
            std::vector<std::string> objDescriptions;
            engine_.process(objDescriptions, "");
            robosherlock_msgs::RSObjectDescriptions objDescr;
            objDescr.obj_descriptions = objDescriptions;
            result_pub.publish(objDescr);
        }
    }
}