#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/opencv.cpp> //do we need this?
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <openpose/flags.hpp>
#include <openpose/headers.hpp> //Openpose dependencies
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

#include "tf2_ros/message_filter.h"//do we need?
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "point/TFBroadcastPR.h"
#include "point/Client.h"

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
                std::cout << "Read in rawX." << std::endl;
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
    
    // Show the two points on the point vector
    void printPoints (std::vector<geometry_msgs::Point> pts) {
        visualization_msgs::Marker points;
        points.header.frame_id = ROOT_TRANSFORM;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;     
        points.color.a = 1.0;

        points.points.push_back(pts[0]);
        points.points.push_back(pts[1]);


        points.color.g = 1.0;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.id = 0;

        points.type = visualization_msgs::Marker::POINTS;
        marker_pub.publish(points);
    }

    // check if pixels w/in certain area from the line intersect
    // with the bounding box of an object
    void searchLine(std::vector<geometry_msgs::Point> line) {
      // Go along line
      // Find size of image and make sure we don't go outside it
      // Iterate through the grid_line returned from bresenham
      // Show extened line on the imshow viewer
      // Think of ways to detect ghosting
      // Find the depth of a x, y cordinate
      // Search around the bresenham line
      // -we could do that in the bresenham method (add to grid_line)
      // -or create a separate method?
      std::vector<geometry_msgs::Point> grid_line;
      grid_line = bresenham(line);
      int i;
      int search_area = 20; //num pixels above/ below line to search
      for(int i = 0; i < grid_line.size(); i++){
          float x = grid_line[i].x;
          float y = grid_line[i].y;
          //check above the line
          if(withinImage(x, y+search_area)) 
            objectIntersection(x, y+search_area);
          //check below the line
          if(withinImage(x, y-search_area))
            objectIntersection(x, y-search_area);
      }

    }
    //check if x,y is valid pixel within coordinates of the image
    bool withinImage(float x, float y){
       if(x > -1 && x < color_image.rows && y > -1 && y < color_image.cols)
            return true;
        return false; 
    }
    //check if given pixel lies within bounding box of an object
    //TODO: return which object
    void objectIntersection(float x, float y){

    }

    //get the coordinates of the pixels that need to be searched
    std::vector<geometry_msgs::Point> bresenham (std::vector<geometry_msgs::Point> line) {
        // Make sure we have 2 valid points
        std::vector<geometry_msgs::Point> grid_line;
        if (line.size() < 2 )  {
            return grid_line;
        }
        
        float x1 = line[0].x;
        float y1 = line[0].y;
        float x2 = line[1].x;
        float y2 = line[1].y;

        float m_new = 2 * (y2 - y1); 
        float slope_error_new = m_new - (x2 - x1); 
        for (float x = x1, y = y1; x <= x2; x++) 
        { 
            geometry_msgs::Point cor;
            cor.x = x;
            cor.y = y;
            grid_line.push_back(cor);
            // cout << "(" << x << "," << y << ")\n"; 
        
            // Add slope to increment angle formed 
            slope_error_new += m_new; 
        
            // Slope error reached limit, time to 
            // increment y and update slope error. 
            if (slope_error_new >= 0) 
            { 
                y++; 
                slope_error_new  -= 2 * (x2 - x1); 
            } 
        } 
        return grid_line;
    }

    /*
     * Magnitude of a vector
     */
    float magnitude(const float *vec, int n) {
        float total = 0.0;
        for (int i = 0; i < n; i++)
    	total += vec[i] * vec[i];
        return sqrt(total);
    }
    
    /*
     * Stores the cross product of v1 and v2 in output
     * Strictly must be defined for vectors of size 3, so we don't need a size
     * 
     * Definition of cross product is overly complicated, but this is used
     * in the calculation later for distance from a point to a line
     *
     * This can be understood as a simplification of the formula:
     * | i	   j     k     |
     * | v1[0] v1[1] v1[2] |
     * | v2[0] v2[1] v2[2] |
     * where you are taking the determinant of that matrix and
     * i,j,k are the orthonormal element vectors
     */
    void crossProduct(const float v1[3], const float v2[3], float output[3]) {
    	output[0] = v1[1] * v2[2] - v1[2] * v2[1];
    	output[1] = v1[2] * v2[0] - v1[0] * v2[2];
    	output[3] = v1[0] * v2[1] - v1[1] * v2[0];
    }
    
    
    /*
     * As per http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
     * This solves for the minimum distance from pt (an <x,y,z> vector) to the
     * vector formed between points v1 and v2
     *
     * We can use this to test the distance from each point along our 2d line
     * to the gesture created by the point
     */    
    float pointLineDistance3d(const float v1[3], const float v2[3], const float pt[3]) {
    	float den[3];    // Denominator vector: v2 - v1
    	float d1[3];     // Numerator left half: pt - v1
    	float d2[3];     // Numerator right half: pt - v2
    	float cross[3];  // Numerator vector: d1 x d2
    	// Set up vectors
    	for (int i = 0; i < 3; i++) {
    	    den[i] = v2[i] - v1[i];
    	    d1[i] = pt[i] - v1[i];
    	    d2[i] = pt[i] - v2[i];
    	}
    	crossProduct(d1, d2, cross);

    	// Return calculation
    	return magnitude(cross, 3) / magnitude(den, 3);
    }
	
};


int main(int argc, char **argv) {
 ros::init(argc, argv, "point_node");
    ros::NodeHandle node;
    cv::Mat img = cv::imread("/home/users/mmurphy/pointing/src/donut.png");

    Client client(img);
    client.connection();
    /* op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

    try {
        // Use single thread for debugging
        //if (FLAGS_disable_multi_thread) {
        //    opWrapper.disableMultiThreading();
        //} //only use if openpose build with flags enabled
        

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
    */
    std::cout<<"running"<<std::endl;
    ros::spin();
    std::cout<<"running"<<std::endl;

    // Run openposes on the cv mats

    return 0;

}
