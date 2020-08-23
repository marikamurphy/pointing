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
        Client();
        vector<Point> getBoxes();
        vector<string> getLabels();
        void sendImage(std::string img_path);
        void sendImage(Mat image);
        Mat loadImage(std::string img_path);
        int connection();
        void sendCV(int sockfd, cv::Mat src);
        vector<Point> interpretBuf(char *buf);
        vector<string> interpretLabels(char *buf);

    private:
        cv::Mat _photo;
        vector<Point> boxes;
        vector<string> labels;
        

};



#endif