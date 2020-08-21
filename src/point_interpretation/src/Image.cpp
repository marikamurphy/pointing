#include "Image.h"

using namespace Eigen;
using namespace cv;

Image::Image(){

}

Mat Image::loadImage(std::string img_path){
    Mat image;
    image = imread(img_path);   // Read the file
    if(! image.data )                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return image;
    }
    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image );                   // Show our image inside it.

    //waitKey(0);
    return image;     
}

 vector<Point> Image::sendImage(std::string img_path){
    Mat image = loadImage(img_path);
    Client *client = new Client();
    int sockfd = client->connection();
    return client->sendCV(sockfd, image);
}

 vector<Point> Image::sendImage(Mat image){
    Client *client = new Client();
    int sockfd = client->connection();
    return client->sendCV(sockfd, image);
}