#ifndef CLIENT_H
#define CLIENT_H

#include <opencv2/imgproc/imgproc.hpp>

class Client {
    public:
        Client ();
        int connection();
        int sendCV(int sockfd, cv::Mat src);

    private:
        cv::Mat _photo;

};



#endif