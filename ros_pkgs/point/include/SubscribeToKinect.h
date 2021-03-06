#ifndef SUBSCRIBE_TO_KINECT_H
#define SUBSCRIBE_TO_KINECT_H

#include "Client.h"
#include "TFBroadcastPR.h"
#include "ProjectiveUtil.h"
// #include "OpenPoseUtil.h" //eventually re-incorporate
//#include "tf2_ros/message_filter.h"//do we need?
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/synchronizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
    Eigen::MatrixXf p_ident;
    Eigen::MatrixXf rotation_trans_mat;
    /* Depth values turned into 3D coordinates.  Each column is a point and 
     * the rows are x, y, z, and w. */ 
    Eigen::MatrixXf raw_3d_coords;
    /* All of the depth values at each location.  Given in terms of the distance from the camera*/
    cv::Mat depth_image;
    cv::Mat color_image;
    int **map2to3;
    
    geometry_msgs::Point elbow;
    geometry_msgs::Point wrist;

public:
    ros::Publisher marker_pub;

    /* Constructor for Subscribe to Kinect.  Initializes opWrapper and sets value for p_ident. */
    SubscribeToKinect();

    void master_callback(const sensor_msgs::Image::ConstPtr &color, const sensor_msgs::Image::ConstPtr &depth);

    /* Get camera calibration values.  Adjust them and put into camera calibration matrix. */
    Eigen::Matrix3f camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    /* Show points on vector in rviz. Creates a new geometry_msg::Point each time it is called. */
    void print_points (std::vector<geometry_msgs::Point> pts);

    void print_rotation_trans_mat (geometry_msgs::TransformStamped transform);
    
};

#endif
