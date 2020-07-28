#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class ImageProcessor {
public:
    void imageCb(const sensor_msgs::Image::ConstPtr &);
    ImageProcessor() : subscriptionString("unprovided"),
	img_encoding(sensor_msgs::image_encodings::BGR8) {}
    ImageProcessor(const std::string &s) : subscriptionString(s), 
	img_encoding(sensor_msgs::image_encodings::BGR8) {}
    ImageProcessor(const std::string &s, const std::string &e) :
	subscriptionString(s), img_encoding(e) {}
private:
    std::string subscriptionString;
    std::string img_encoding;
};
