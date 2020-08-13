#include "Image.h"

using namespace Eigen;
using namespace cv

Image::Image(){

}

Mat Image::loadImage(string img_path){
    Mat image;
    image = imread(img_path, CV_LOAD_IMAGE_COLOR);   // Read the file
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);     
}

MatrixXd Image::sendImage(Mat src){
    Mat image = loadImage();
    Client *client = new Client();
    int sockfd = client->connection();
    return client->sendCV(sockfd, image);
}