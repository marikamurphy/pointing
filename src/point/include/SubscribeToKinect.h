#ifndef SUBSCRIBE_TO_KINECT_H
#define SUBSCRIBE_TO_KINECT_H

#include "Client.h"
#include "TFBroadcastPR.h"
#include "tf2_ros/message_filter.h"//do we need?
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>


class SubscribeToKinect {
private:
    /* Struct to store camera parameters */
    Eigen::Matrix3f cam_cal;
    Eigen::MatrixXf p_ideal; // Might be wrong /* p_ident * rotation_trans_mat = p_ideal */
    Eigen::MatrixXf p_real; // Also might not have correct sizing /* p_ideal * cam_cal = p_real*/
    Eigen::MatrixXf raw_2d_points;
    /* Depth values turned into 3D coordinates.  Each column is a point and 
     * the rows are x, y, z, and w. */ 
    Eigen::MatrixXf raw_3d_coords;
    /* All of the depth values at each location.  Given in terms of the distance from the camera*/
    cv::Mat depth_image;
    cv::Mat color_image;
    int **map2to3;
    op::Wrapper& opWrapper;
    geometry_msgs::Point elbow;
    geometry_msgs::Point wrist;

public:
    ros::Publisher marker_pub;

    /* Constructor for Subscribe to Kinect.  Initializes opWrapper and sets value for p_ident. */
    SubscribeToKinect(op::Wrapper& wrapper);

    void master_callback(const sensor_msgs::Image::ConstPtr &color, const sensor_msgs::Image::ConstPtr &depth);

    /* Get camera calibration values.  Adjust them and put into camera calibration matrix. */
    Eigen::Matrix3f camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    /* Takes a depth value and converts it to a 3D x, y, and z. */
    void depth_to_3D(Eigen::Matrix3f cam_cal, int index, float &x, float &y, float &z);

    /* Converts all of the depth values in depth_image and puts the x, y, and z into raw_3D_coords. */
    void calculate_3D_coords();

    void multiply_stuff();

    /* Create the grid that holds the mapping.*/
    void initialize_point_grid();

    /* Given a pixel in the form x, y, return the 3D value corresponding to that location. */
    geometry_msgs::Point get_3d_point(int x, int y);

    /* Take the values of raw_2d_points and put them into a grid.
     * To retrieve a point of a pixel <x, y>, grab the index at <x, y> and 
     * index into raw_3d_points. */
    void build2Dto3DMap();

    // Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
    /* Get the Datum with the body key points. */
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> get_key_points();

    /* Adjust a x, y, depth point to reflect the camera calibration. */
    geometry_msgs::Point transform_point(float x, float y, float depth_val);

    /* Print all of the body keypoints for the first person in the list. */
    void print_keypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumPtr);

    // Publish the relevant keypoints for wrists and elbows.
    void show_body_keypoints(const op::Array<float>& keyPoints);
      \
    // Locate limb positions 
    std::vector<geometry_msgs::Point> find_end_points(const op::Array<float>& keyPoints);

    // Find what's being pointed at out in space
    std::vector<geometry_msgs::Point> extend_point (std::vector<geometry_msgs::Point> origin, int scalar); 
    
    
    /* Show points on vector in rviz. Creates a new geometry_msg::Point each time it is called. */
    void print_points (std::vector<geometry_msgs::Point> pts);

    void print_rotation_trans_mat (geometry_msgs::TransformStamped transform);
    

};

#endif
