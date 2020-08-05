#include "Client.h"
#include "SubscribeToKinect.h"
#include "tf2_ros/message_filter.h"//do we need?
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/synchronizer.h>
// #include <openpose/headers.hpp> //Openpose dependencies
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_node");
    ros::NodeHandle node;
    
    /* Create the subscribe to kinect object. */
    SubscribeToKinect kSub;
    //SubscribeToKinect kSub(); // we do this once we want to pass the constructor arguments
    /* Grab the color and depth image. */
    message_filters::Subscriber<sensor_msgs::Image> depth_image_test(node, "/head_rgbd_sensor_depth_registered_frame", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_test(node, "/head_rgbd_sensor_rgb_frame", 10); //TODO: fix for hsr
    // Synchronize the data from the depth and rgb camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_test, depth_image_test);
    sync.registerCallback(boost::bind(&SubscribeToKinect::master_callback, &kSub, _1, _2));

    // I don't know what this does. I hope it's not important.
    // kSub.marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // tf2_ros::Buffer buffer;
    // tf2_ros::TransformListener listener(buffer);
    // geometry_msgs::TransformStamped transform;

    // while (node.ok()) {
    //     try{                                    
    //     transform = buffer.lookupTransform("/head_rgbd_sensor_rgb_frame", "/head_rgbd_sensor_depth_registered_frame",  
    //                             ros::Time(0));
    //     }
    //     catch (tf2::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    //     }
    //     kSub.print_rotation_trans_mat(transform);
    // }

    // ros::spin();

    /* Create cam_cal */
    Eigen::Matrix3f cam_cal = getCameraIntrinsicMatrix(205.46963709898583, 205.46963709898583, 320.5, 240.5);
    
    return 0;

}