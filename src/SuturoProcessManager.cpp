/**
 * Modified RoboSherlock Process Manager
 * Optimized for the Suturo Perception Pipelines
 * @author Fenja Kollasch
 */
#include <rs_hsrb_perception/SuturoProcessManager.h>

SuturoProcessManager::SuturoProcessManager(ros::NodeHandle n, const std::string savePath) :
    savePath(savePath),
    nh(n)
{
    outInfo("A RoboSherlock process manager optimized for the Suturo perception was created.");
    signal(SIGINT, RSProcessManager::signalHandler);
    outInfo("Creating resource manager");
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

    switch(OUT_LEVEL)
    {
        case OUT_LEVEL_NOOUT:
        case OUT_LEVEL_ERROR:
            resourceManager.setLoggingLevel(uima::LogStream::EnError);
            break;
        case OUT_LEVEL_INFO:
            resourceManager.setLoggingLevel(uima::LogStream::EnWarning);
            break;
        case OUT_LEVEL_DEBUG:
            resourceManager.setLoggingLevel(uima::LogStream::EnMessage);
            break;
    }

    visService = nh.advertiseService("vis_command", &SuturoProcessManager::visControlCallback, this);
}



void SuturoProcessManager::init(std::string &pipeline) {
    outInfo("Initializing Engine...");
    signal(SIGINT, RSProcessManager::signalHandler);

    std::string pipelinePath;
    rs::common::getAEPaths(pipeline, pipelinePath);
    engine.init(pipelinePath, false);

    uima::ErrorInfo errorInfo;
    mongo::client::GlobalInstance instance;
}

void SuturoProcessManager::run(bool visualize, std::vector<ObjectDetectionData>& detectionData) {
    outInfo("Running the Suturo Process Manager");
    visualizer = new rs::Visualizer(savePath, !visualize);
    visualizer->start();
    outInfo("Analysis engine starts processing");
    engine.process();
    uima::CAS* tcas = engine.getCas();
    rs::SceneCas cas(*tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    for (auto &cluster : clusters) {
        getClusterFeatures(cluster, detectionData);
    }
}

bool SuturoProcessManager::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
                                          robosherlock_msgs::RSVisControl::Response &res)
{

    std::string command = req.command;
    bool result = true;
    if(visualizer != nullptr) {
        std::string activeAnnotator = "";
        if(command == "next")
        {
            activeAnnotator = visualizer->nextAnnotator();

        }
        else if(command == "previous")
        {
            activeAnnotator = visualizer->prevAnnotator();
        }
        else if(command.empty())
        {
            activeAnnotator = visualizer->selectAnnotator(command);
        }
        if(activeAnnotator.empty())
            result = false;

        res.success = result;

        res.active_annotator = activeAnnotator;
    }
    return result;
}


void makeObjectDetectionData(geometry_msgs::PoseStamped pose, rs::Geometry geometry, u_int shape, std::string objClass, float confidence, std::string knownObjClass, float knownObjConfidence, ObjectDetectionData &odd) {
    odd.pose = pose;
    auto boundingBox = geometry.boundingBox();
    odd.width = boundingBox.width();
    odd.height = boundingBox.height();
    odd.depth = boundingBox.depth();
    odd.name = "Object (" + objClass + ")";
    odd.shape = shape;

    odd.obj_class = objClass;
    odd.confidence = confidence;

    odd.known_obj_class = knownObjClass;
    odd.known_obj_confidence = knownObjConfidence;
}


// Todo: Think of a smarter way to transform pose types
void rsPoseToGeoPose(rs::StampedPose pose, geometry_msgs::PoseStamped &geoPose) {
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

void SuturoProcessManager::getClusterFeatures(rs::ObjectHypothesis cluster, std::vector<ObjectDetectionData> &data) {

    std::vector<rs::Geometry> geometry;
    cluster.annotations.filter(geometry);

    if(!geometry.empty()) {
        std::vector<rs::PoseAnnotation> poses;
        cluster.annotations.filter(poses);

        std::vector<rs::Classification> classification;
        cluster.annotations.filter(classification);

        std::vector<rs::ClassConfidence> confi;
        cluster.annotations.filter(confi);

        std::vector<rs::Shape> shapes;
        cluster.annotations.filter(shapes);

        geometry_msgs::PoseStamped poseStamped;
        std::string objClass;
        std::string knownObjClass;
        u_int shape = 0;
        float confidence = 0;
        float knownObjConfidence = 0;
        ObjectDetectionData odd;
        if(!poses.empty()) {
            rsPoseToGeoPose(poses[0].world.get(), poseStamped);
        } else {
            ROS_WARN("Warning: No pose information was perceived");
        }
        if(!classification.empty()){
            objClass = classification[0].classname.get();
            knownObjClass = classification[1].classname.get();
            outInfo("OBJCLASSNAME >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: " << classification[0].classname.get());
            outInfo("OBJCLASSNAME >>>>>>>>>KNOWN>>>>>>>KNOWN>>>>>>>: " << classification[1].classname.get());
        } else {
            ROS_WARN("Warning: No object class was perceived");
        }
        if(!confi.empty()){
            confidence = confi[0].score.get();
            knownObjConfidence = confi[1].score.get();
            outInfo("OBJCLASSCONFI <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<: " << confi[0].score.get());
            outInfo("OBJCLASSCONFI <<<<<<<<<KNOWN<<<<<<<KNOWN<<<<<<: " << confi[1].score.get());
        } else {
            ROS_WARN("Warning: No confidence was perceived");
        }

        makeObjectDetectionData(poseStamped, geometry[0], shape, objClass, confidence, knownObjClass, knownObjConfidence, odd);
        data.push_back(odd);


    } else {
        ROS_WARN("Object Feature detection was unsuccessful. No geometries were recognized for this object.");
    }

}
