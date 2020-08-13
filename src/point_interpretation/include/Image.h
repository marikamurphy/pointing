#ifndef IMAGE_H
#define IMAGE_H

#include "Client.h"
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

class Image {
    public:
        Image ();
        cv::Mat loadImage(string img_path);
        int *sendImage(cv::Mat src);
    
};

#endif