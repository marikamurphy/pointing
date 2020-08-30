/* This is where all things ros should go.  I would be ideal if we
 * didn't have to use ros anywhere but this file.
 */

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ros/ros.h> // Ros functionalities
#include <sensor_msgs/CameraInfo.h> 
#include <sensor_msgs/PointCloud2.h>


class SubscribeToKinect {
    private:


    public:

    /* Constructor*/
    SubscribeToKinect::SubscribeToKinect() {}

    // Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
    // For getting key points from an image
    int SubscribeToKinect::getKeyPoints(const cv::Mat cvImageToProcess) {
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
}