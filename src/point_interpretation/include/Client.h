#ifndef CLIENT_H
#define CLIENT_H

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

using namespace Eigen;
using namespace cv;
using namespace std;

class Client {
    public:
        Client ();
        int connection();
        vector<Point> sendCV(int sockfd, cv::Mat src);
        vector<Point> interpretBuf(char *buf);

    private:
        cv::Mat _photo;
        

};



#endif