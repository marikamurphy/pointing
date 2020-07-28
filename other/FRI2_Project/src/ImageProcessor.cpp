#include <ros/ros.h>
#include <fri2/ImageProcessor.h>

void ImageProcessor::imageCb(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, this->img_encoding);
    cv::Mat img = cv_ptr->image;
    ROS_INFO("%s: %ld %ld", this->subscriptionString.c_str(), (long) img.rows, (long) img.cols);
}
