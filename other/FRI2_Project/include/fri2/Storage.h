#ifndef _POINT_STORAGE_H_
#define _POINT_STORAGE_H_

#include <ros/ros.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
class Storage {
public:
    Storage() : __index(0), cx(0.0), cy(0.0), fx(0.0), fy(0.0) {}
    void add(openpose_ros_msgs::OpenPoseHumanList);
    void calibrate(const sensor_msgs::CameraInfo&);

    float cx;
    float cy;
    float fx;
    float fy;
    openpose_ros_msgs::OpenPoseHuman get();
private:
    openpose_ros_msgs::OpenPoseHuman __data[10];
    uint16_t __numPoints;
    uint8_t __index;
};

#endif
