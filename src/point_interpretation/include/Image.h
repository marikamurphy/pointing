#ifndef IMAGE_H
#define IMAGE_H

#include "Client.h"
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <vector>
#include <opencv2/imgcodecs.hpp> // imread
#include <opencv2/highgui.hpp> // imshow and waitkey


using namespace Eigen;
using namespace cv;


class Image {
    public:
        Image ();
        cv::Mat loadImage(std::string img_path);
        vector<Point> sendImage(std::string img_path);
        vector<Point> sendImage(cv::Mat src);
    
};

#endif