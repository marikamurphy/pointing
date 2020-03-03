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

    sensor_msgs::Image image;
    cv::Mat depth_image;
    bool depthhere = false;

public:
    ros::Publisher marker_pub;
    SubscribeToKinect() : broadPRNavFix(ROOT_TRANSFORM, "offset_navfixed") {}

    // Call back to get rgb image
    void imageCb(const sensor_msgs::Image::ConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr = 
            cv_bridge::toCvCopy(msg, msg->encoding);
        cv::Mat cvImageToProcess = cv_ptr->image; //const or not?
        //now we run openpose on it
        getKeyPoints(cvImageToProcess);
    } 

    // Call back to get depth image... depth map easier, faster, better
    //figure out frustrum calculation... we need it to get the 3D point
    void depth_cb(const sensor_msgs::Image::ConstPtr &img) {
        depth_image = cv_bridge::toCvCopy(img, img->encoding)->image;
        depthhere = true;
        //now we need to adjust points for camera calibration

        //before we do adjust: we gotta make sure this is not a bad point/ impossible depth
        //we need to figure out if this is a bad point... how ?  
        
        //float depth_val = cv_depth_image.at<float>(y, x);
        //if (isnan(depth_val) || depth_val <= 0.001)
        //{
            //depth_val = prevDepthVal;
        //}
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
        try {
            // opLog is like print?
            // op::opLog("Printing key points: ", op::Priority::High);

            op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
            // Use single thread for debugging
            if (FLAGS_disable_multi_thread) {
                opWrapper.disableMultiThreading();
            }
            

            op::WrapperStructPose wrapperStructPose{};
            wrapperStructPose.modelFolder = op::String("openpose/models");
           
            opWrapper.configure(wrapperStructPose);
            opWrapper.start();
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
            return 0;
        } catch (const std::exception& e) {
            return -1;
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
        if (!depthhere)
            return;
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

        for (int i = 0; i < keyPoints.getSize(1); i++) {
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
    ros::init(argc, argv, "mapping_marker");
    ros::NodeHandle node;
    SubscribeToKinect kSub;
    ros::Subscriber depth_image_test = node.subscribe("/camera/depth/image"
        , 10, &SubscribeToKinect::depth_cb, &kSub);
    ros::Subscriber image_test = node.subscribe("/camera/rgb/image_color"
       , 10, &SubscribeToKinect::imageCb, &kSub);
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
