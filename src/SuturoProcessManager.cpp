/**
 * Modified RoboSherlock Process Manager
 * Optimized for the Suturo Perception Pipelines
 * @author Fenja Kollasch
 */
#include "../include/rs_hsrb_perception/SuturoProcessManager.h"

SuturoProcessManager::SuturoProcessManager(RSProcessManager::KnowledgeEngineType keType, ros::NodeHandle n)
: RSProcessManager(false, false, keType, n, ""){
    outInfo("A RoboSherlock process manager optimized for the Suturo perception was created.");
}

SuturoProcessManager::~SuturoProcessManager() {
    uima::ResourceManager::deleteInstance();
    outInfo("Suturo Process Manager stopped.");
}

void SuturoProcessManager::makeVisualizer(const std::string &savePath) {
    visualizer_.stop();
    suturo_vis = new rs::Visualizer(savePath, false);
    suturo_vis->start();
}

void SuturoProcessManager::init(std::string &xmlFile, bool pervasive, bool parallel) {
    RSProcessManager::init(xmlFile, ros::package::getPath("robosherlock") + "/config/config.yaml", pervasive, parallel);
}

