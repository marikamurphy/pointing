
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

Eigen::MatrixXf getPIdent() {
    Eigen::MatrixXf pIdent = Eigen::ArrayXXf::Zero(3, 4);
    for(int i = 0; i < 3; i++)
        pIdent(i,i) = 1.0f;
    return pIdent;
}

Eigen::MatrixXf getCameraIntrinsicMatrix(float fx, float fy, float cx, float cy) {
    Eigen::MatrixXf cameraIntrinsic = Eigen::MatrixXf::Identity(3, 3);
    cameraIntrinsic(0,0) = fx;
    cameraIntrinsic(1,1) = fy;
    cameraIntrinsic(0,3) = cx;
    cameraIntrinsic(1,3) = cy;
    return cameraIntrinsic;
}

