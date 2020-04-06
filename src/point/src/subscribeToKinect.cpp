#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/opencv.cpp> //do we need this?
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp> //Openpose dependencies
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "point/TFBroadcastPR.h"


#define ROOT_TRANSFORM "camera_rgb_optical_frame"

class SubscribeToKinect {
private:

    //struct to store camera parameters
    struct {
        float fx = 0;
        float fy = 0;
        float cx = 0; 
        float cy = 0;
    } camCal;

    TFBroadcastPR broadPRNavFix; 
    cv::Mat depth_image;
    cv::Mat color_image;
    //sensor_msgs::Image image;
    op::Wrapper& opWrapper;

public:
    ros::Publisher marker_pub;
    SubscribeToKinect(op::Wrapper& wrapper) : opWrapper(wrapper), broadPRNavFix(ROOT_TRANSFORM, "offset_navfixed") { }

    void masterCallback(const sensor_msgs::Image::ConstPtr &color, const sensor_msgs::Image::ConstPtr &depth) {
        ROS_INFO("OMG LOL");
        // depth image stuff
        depth_image = cv_bridge::toCvCopy(depth, depth->encoding)->image;
        color_image = cv_bridge::toCvCopy(color, color->encoding)->image;

        //cv::imshow("1", depth_image);
        //cv::imshow("2", color_image);
        //cv::waitKey(1);
        //now we run openpose on it
        getKeyPoints(color_image);
    } 

    //adjust a x, y, depth point to reflect the camera calibration
    geometry_msgs::Point transformPoint(int x, int y, float depth_val){
        geometry_msgs::Point point;
        point.x = (x + 0.5 - camCal.cx) * camCal.fx * depth_val;
        point.y = (y + 0.5 - camCal.cy) * camCal.fy * depth_val;
        point.z = depth_val;

        return point;
    }
    
    //get the camera calibration depth
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        camCal.fx = 1/msg->K[0]; // we need to adjust
        camCal.fy = 1/msg->K[4];
        camCal.cx = msg->K[2];
        camCal.cy = msg->K[5];
    }

    // Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
    // For getting key points from an image
    int getKeyPoints(const cv::Mat cvImageToProcess) {
        // opLog is like print?
        // op::opLog("Printing key points: ", op::Priority::High);
        // Instead of doing this, we need to pass in  the cv Mats we get in the subscriber
        //const cv::Mat cvImageToProcess = cv::imread(FLAGS_image_path);
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
        // Error checking datum processed
        if (datumProcessed != nullptr) {
            printKeypoints(datumProcessed);
        } else {
            op::opLog("Image could not be processed.", op::Priority::High);
        }
    }

    void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumPtr) {
        try {
            if (datumPtr != nullptr && !datumPtr->empty()) {
                // op::opLog("Body keypoints: " + datumPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
                showBodyKeypoints(datumPtr->at(0)->poseKeypoints);
            }
        } catch (const std::exception& e) {
            //op::error(e.what(), ___LINE__, __FUNCTION__, __FILE__);
            op::error(e.what());
        }
    }


    void showBodyKeypoints(const op::Array<float>& keyPoints) {
        visualization_msgs::Marker points;
        points.header.frame_id = ROOT_TRANSFORM;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.id = 0;
        points.color.r = 1.0f;
        points.color.a = 1.0;


        // auto keyPoints = datumPtr->at(0)->poseKeypoints;
        // std::cout << "keypoints  is: " << typeid(keyPoints).name();
        // key
        
        std::vector<int> sizes = keyPoints.getSize();

        if (keyPoints.getSize(0))
            for (int i = 3; i < 5; i++) {
                std::vector<int> xIndex{0, i, 0};
                std::vector<int> yIndex{0, i, 1};
                float rawX = keyPoints.at(xIndex);
                float rawY = keyPoints.at(yIndex); 
                float rawZ = depth_image.at<float>(rawY, rawX);
                if (!(std::isnan(rawZ) || rawZ <= 0.001)) {
                    geometry_msgs::Point p = transformPoint(rawX, rawY, rawZ);
                    points.points.push_back(p);
                }
            }
        marker_pub.publish(points);
    }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_node");
    ros::NodeHandle node;
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

    try {
        // Use single thread for debugging
        if (FLAGS_disable_multi_thread) {
            opWrapper.disableMultiThreading();
        }
        

        op::WrapperStructPose wrapperStructPose{};
        wrapperStructPose.modelFolder = op::String("openpose/models");
        
        opWrapper.configure(wrapperStructPose);
        opWrapper.start();
        op::PoseModel& poseModel = wrapperStructPose.poseModel;
        const auto& pbpmBody25 = getPoseBodyPartMapping(poseModel);

        // std::cout << "get 3 (relbow?) " << pbpmBody25.at(3) << std::endl; 
        // std::cout << "get 4 (rwrist?) " << pbpmBody25.at(4) << std::endl;
        // std::cout << "pose model: " << static_cast<int>(poseModel) << std::endl;
    } catch (const std::exception& e) {
        return -1;
    }


    SubscribeToKinect kSub(opWrapper);

    message_filters::Subscriber<sensor_msgs::Image> depth_image_test(node, "/camera/depth/image", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_test(node, "/camera/rgb/image_color", 10);

    //message_filters::Subscriber ptCloud(node, "camera/depth/points", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_test, depth_image_test);
    sync.registerCallback(boost::bind(&SubscribeToKinect::masterCallback, &kSub, _1, _2));

    ros::Subscriber cam_intrinsic_params = node.subscribe("/camera/depth_registered/sw_registered/camera_info",
        10, &SubscribeToKinect::cameraInfoCallback, &kSub);
    //ros::Subscriber ptCloud = node.subscribe("camera/depth/points",
     //   10, &SubscribeToKinect::printPtCloud, &kSub);
    kSub.marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    std::cout<<"running"<<std::endl;
    ros::spin();
    std::cout<<"running"<<std::endl;

    // Run openposes on the cv mats

    return 0;
}
