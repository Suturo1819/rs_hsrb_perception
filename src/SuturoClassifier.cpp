#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//OpenCV/DL
#include <fstream>
#include <sstream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace uima;
using namespace cv;
using namespace dnn;

/**
 * An object classifier for Suturo 18/19
 * Based on the OpenCV image classfication tutorial
 * @author Fenja Kollasch
 */
class SuturoClassifier : public Annotator
{
private:

    // Path to a binary file of model contains trained weights.
    // This version is using a .caffemodel
    std::string trainedModel;

    // Path to a text file of model contains network configuration.
    // For Caffe, this is a .prototxt
    std::string modelConfig;

    // Preprocess input image by multiplying on a scale factor
    float scaleFactor;

    // Preprocess input image by subtracting mean values.
    // Mean values should be in BGR order and delimited by spaces.
    std::vector<float> mean;

    // Indicate that model works with RGB input images instead BGR ones.
    bool swapRB;

    // Preprocess input image by resizing to a specific width.
    int inpWidth;

    // Preprocess input image by resizing to a specific height.
    int inpHeight;

    // The training network initialized by the .caffemodel and .prototxt config file
    Net net;

    // Video capture data types
    VideoCapture capture;
    Mat processedFrame;
    Mat imgBlob;

public:

    //TODO: Agree with knowledge on different object classes
    const std::vector<std::string> classes;

    TyErrorId initialize(AnnotatorContext &ctx) {
        outInfo("initializing parameters for SuturoClassifier");
        ctx.extractValue("trainedModel", trainedModel);
        ctx.extractValue("modelConfig", modelConfig);
        ctx.extractValue("scaleFactor", scaleFactor);
        ctx.extractValue("swapRB", swapRB);
        ctx.extractValue("inpWidth", inpWidth);
        ctx.extractValue("inpHeight", inpHeight);

        //TODO: Replace hardcoded stuff
        mean = std::vector<float>();
        mean.push_back(104);
        mean.push_back(117);
        mean.push_back(123);
        net = readNetFromCaffe(modelConfig, trainedModel);
        capture.open(0);
        return UIMA_ERR_NONE;
    }

    TyErrorId destroy() {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec) {
        outInfo("process start");
        rs::SceneCas cas(tcas);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

        capture >> processedFrame;
        if (processedFrame.empty()) {
            waitKey();
            return UIMA_ERR_NONE;
        }

        imgBlob = blobFromImage(processedFrame, scaleFactor, Size(inpWidth, inpHeight), Scalar(mean[0], mean[1], mean[2]), swapRB, false);
        net.setInput(imgBlob);
        Mat prob = net.forward();
        Point classIdPoint;
        double confidence;
        minMaxLoc(prob.reshape(1, 1), 0, &confidence, 0, &classIdPoint);
        int classId = classIdPoint.x;
        // Put efficiency information.
        std::vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        std::string label = format("Inference time: %.2f ms", t);
        putText(processedFrame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
        outInfo(label);
        // Print predicted class.
        label = format("%s: %.4f", (classes.empty() ? format("Class #%d", classId).c_str() :
                                    classes[classId].c_str()),
                       confidence);
        putText(processedFrame, label, Point(0, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
        outInfo(label);

        return UIMA_ERR_NONE;
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SuturoClassifier)