#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class SubscribeToKinect {
public:

    void imageCb(const sensor_msgs::Image::ConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr = 
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgbImage = cv_ptr->image;
    } 

    void depth_test_cb(const sensor_msgs::Image img) {
        cv::Mat cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapping_marker");
    ros::NodeHandle node;
    SubscribeToKinect kSub;
    ros::Subscriber depth_image_test = node.subscribe("/camera/depth/image"
        , 10, &SubscribeToKinect::depth_test_cb, &kSub);
    ros::spin();
    return 0;
}
