#ifndef CLIENT_H
#define CLIENT_H

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

using namespace Eigen;

class Client {
    public:
        Client ();
        int connection();
        MatrixXd sendCV(int sockfd, cv::Mat src);
        MatrixXd interpretBuf(char *buf);

    private:
        cv::Mat _photo;
        

};



#endif