#ifndef CLIENT_H
#define CLIENT_H

#include <opencv2/imgproc/imgproc.hpp>

class Client {
    public:
        Client (cv::Mat photo);
        void connection();

    private:
        cv::Mat _photo;

};



#endif