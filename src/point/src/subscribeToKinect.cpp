#include "point/Client.h"
#include "point/TFBroadcastPR.h"
#include "tf2_ros/message_filter.h"//do we need?
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <openpose/headers.hpp> //Openpose dependencies
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
//#include <opencv2/opencv.cpp> //do we need this?
//#include <openpose/flags.hpp>

#define ROOT_TRANSFORM "camera_rgb_optical_frame"
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

class SubscribeToKinect {
private:

    /* Struct to store camera parameters */
    Matrix3f camCal;
    struct {
        float fx = 0;
        float fy = 0;
        float cx = 0; 
        float cy = 0;
    } camCal;
    cv::Mat depth_image;
    cv::Mat color_image;
    op::Wrapper& opWrapper;

public:
    ros::Publisher marker_pub;
    /* Constructor for Subscribe to Kinect.  Initializes opWrapper. */
    SubscribeToKinect(op::Wrapper& wrapper) : opWrapper(wrapper)) { }

    void masterCallback(const sensor_msgs::Image::ConstPtr &color, const sensor_msgs::Image::ConstPtr &depth) {
        ROS_INFO("OMG LOL");
        /* Save the image and depth map to class variables. */
        depth_image = cv_bridge::toCvCopy(depth, depth->encoding)->image;
        color_image = cv_bridge::toCvCopy(color, color->encoding)->image;

        cv::imshow("1", depth_image);
        cv::imshow("2", color_image);
        cv::waitKey(1);
        /* Now we run openpose on it. */
        getKeyPoints();
    } 

    /* Get camera calibration values.  Adjust them and put into camera calibration matrix. */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        // TODO Do we need to do the 1/msg thing?
        float fx = 1/msg->K[0];
        float fy = 1/msg->K[4];
        float cx = msg->K[2];
        float cy = msg->K[5];
        camCal << fx,  0, cx,
                   0, fy,  0,
                   0,  0,  1;
        return camCal;
    }    

    // Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
    /* Get the DatumProcessed with all of the information on body key points. */
    auto getKeyPoints() {
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(color_image);
        auto datum = opWrapper.emplaceAndPop(imageToProcess);
        /* Error checking datum processed. */
        if (datum != nullptr) {
            //printKeypoints(datum);
            return datum;
        } else {
            op::opLog("Image could not be processed.", op::Priority::High);
            return Null;
        }
    }

    //extract points from datumPtr
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

    //this can be used to publish all of the openpose keypoints in keypoints
    void showBodyKeypoints(const op::Array<float>& keyPoints) {
        visualization_msgs::Marker points;
        points.header.frame_id = ROOT_TRANSFORM;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;     
        points.color.a = 1.0;

        for (const geometry_msgs::Point& p : findEndPoints(keyPoints))
            points.points.push_back(p);

        visualization_msgs::Marker line_list(points);

        points.color.g = 1.0;
        line_list.color.r = 1.0;

        points.scale.x = 0.2;
        points.scale.y = 0.2;
        line_list.scale.x = 0.1;

        points.id = 0;
        line_list.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // auto keyPoints = datumPtr->at(0)->poseKeypoints;
        // std::cout << "keypoints  is: " << typeid(keyPoints).name();
        // key
        

        marker_pub.publish(points);
        marker_pub.publish(line_list);
    }

    // Locate limb positions 
    std::vector<geometry_msgs::Point> findEndPoints(const op::Array<float>& keyPoints) {
        visualization_msgs::Marker points;
        std::vector<geometry_msgs::Point> ret;
        if (keyPoints.getSize(0)){
            for (int i = 3; i < 5; i++) {
                std::vector<int> xIndex{0, i, 0};
                std::vector<int> yIndex{0, i, 1};
                std::cout << "Set up indicies" << std::endl;
                float rawX = keyPoints.at({0, 3, 0});
                std::/out << "Read in rawX." << std::endl;
                float rawY = keyPoints.at(yIndex);
                std::cout << "Read in rawY." << std::endl;
                if (std::isnan(rawX) || std::isnan(rawY)) {
                    return ret;
                } 
                float rawZ = depth_image.at<float>(rawY, rawX);
                if (!(std::isnan(rawZ) || rawZ <= 0.001)) {
                    geometry_msgs::Point p = transformPoint(rawX, rawY, rawZ);
                    std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
                    ret.push_back(p);
                    
                }
            }
        }
        if (ret.size() >= 2) {
            // TODO Heuristic to determine how much to scale by?
            extendPoint(ret, 5);
        }
        
        return ret;
    }

    // Find what's being pointed at out in space
    std::vector<geometry_msgs::Point> extendPoint (std::vector<geometry_msgs::Point> origin, int scalar) {
        geometry_msgs::Point vec;
        // Create vector
        vec.x = origin[1].x - origin[0].x;
        vec.y = origin[1].y - origin[0].y;
        vec.z = origin[1].z - origin[0].z;
        // Extend vector
        geometry_msgs::Point end_vec;
        end_vec.x = vec.x * scalar;
        end_vec.y = vec.y * scalar;
        end_vec.z = vec.z * scalar;
        std::vector<geometry_msgs::Point> ret;
        // Create end point
        geometry_msgs::Point end_pt;
        end_pt.x = origin[0].x + end_vec.x;
        end_pt.y = origin[0].y + end_vec.y;
        end_pt.z = origin[0].z + end_vec.z;
        ret.push_back(origin[0]);
        ret.push_back(end_pt);
        printPoints(ret);
        return ret;
    }
    
    /* Show points on vector in rviz. Creates a new geometry_msg::Point each time it is called. */
    void printPoints (std::vector<geometry_msgs::Point> pts) {
        /* Create visualization msg and set parameters. */
        visualization_msgs::Marker points;
        points.header.frame_id = ROOT_TRANSFORM;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;     
        points.color.a = 1.0;
        points.color.g = 1.0;
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        /* Put points onto msg. */
        for (geometry_msgs::Point pt : pts) {
            points.points.push_back(pt);
        }
        /* Publish points to show on screen. */
        marker_pub.publish(points);
    }

    /* Initialize camera intrisic parameters. */
    Eigen::Matrix3f setCamCal (float fx, float fy, float cx, float cy) {
       camCal << fx,  0, cx,
                   0, fy,  0,
                   0,  0,  1;
        return camCal;
    }

};


int main(int argc, char **argv) {
 ros::init(argc, argv, "point_node");
    ros::NodeHandle node;
    /* Set up opWrapper and point to the right model folder. */
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    try {
        op::WrapperStructPose wrapperStructPose{};
        wrapperStructPose.modelFolder = op::String("openpose/models");
        opWrapper.configure(wrapperStructPose);
        opWrapper.start();
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
