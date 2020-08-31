#ifndef SUBSCRIBE_TO_KINECT_H
#define SEUBSCRIBE_TO_KINECT_H

#include <opencv2/core/mat.hpp> // cv::Mat
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ros/ros.h> // Ros functionalities
#include <sensor_msgs/CameraInfo.h> 
#include <sensor_msgs/PointCloud2.h>
#include <vector> // Vector

class subscribe_to_kinect {
    private:
        cv::Mat depth_image;
        cv::Mat color_image;

    public:
}

#endif