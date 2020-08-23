#include "Client.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <iostream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PORT 3211 
#define IP "127.0.0.1"
#define ARRLEN 256
#define LABELLEN 1024

using namespace std;
using namespace Eigen;
using namespace cv;


Client::Client(){

}

vector<Point> Client::getBoxes(){
    return boxes;
}
vector<string> Client::getLabels(){
    return labels;
}

void Client::sendImage(std::string img_path){
    Mat image = loadImage(img_path);
    int sockfd = connection();
    sendCV(sockfd, image);
}

void Client::sendImage(Mat image){
    int sockfd = connection();
    sendCV(sockfd, image);
}

Mat Client::loadImage(std::string img_path){
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

int Client::connection(){

    // Create the client socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        printf("ERROR opening socket");
        return -1;
    }
    // Define the server address
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, IP, &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
    } 
    
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("ERROR connecting");
        return -1;
    }
    // Server will send what connetion number this is
    char buffer[ARRLEN];
    bzero(buffer, ARRLEN);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
        return -1;
    }
    
    printf("Successfully connected as client number: %s\n", buffer);
    return sockfd;
    
} 


void Client::sendCV(int sockfd, cv::Mat src)
{
    vector<uchar> buf;
    cv::imencode(".jpg", src, buf);
    // Send size first
    int len = buf.size();
    if (write(sockfd, &len, sizeof(len)) < 0)
    {
        perror("ERROR writing to socket");
    }
    cout << "sent size" << endl;
    if (write(sockfd, buf.data(), buf.size()) < 0)
    {
        perror("ERROR writing to socket");
    }
    cout << "sent data" << endl;
    
    //get back coordinates
    char buffer[ARRLEN];
    bzero(buffer, ARRLEN);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
    }
    printf("%s\n", buffer);

    char labelBuf[LABELLEN];
    bzero(labelBuf, LABELLEN);
    if (read(sockfd, labelBuf, sizeof(labelBuf)) < 0)
    {
        perror("ERROR reading from socket");
    }
    printf("%s\n", labelBuf);

    boxes = interpretBuf(buffer);
    labels = interpretLabels(labelBuf);
}

//Here we place the coordinates into a MatrixXd
vector<Point> Client::interpretBuf(char buf[ARRLEN]){
    
    vector<double> arr;
    int i = 1;
    while(buf[i] != ']'){
        std::string num ="";
        while(buf[i] != ',' && buf[i] != ']'){
            num.push_back(buf[i]);
            i++;
        }
        if(buf[i] == ',')
            i++;
        
        arr.push_back(std::stoi(num));
    }
    int size = arr.size();

    vector<Point> boxDict;
    Point pnt;
    for (int i = 0; i < size/2; i++) {
        pnt.x = arr.at(i*2);
        pnt.y = arr.at(i*2+1);
        boxDict.push_back (pnt);
    }
    return boxDict;
}

//Here we place the coordinates into a MatrixXd
vector<string> Client::interpretLabels(char buf[LABELLEN]){
    
    vector<string> arr;
    int i = 1;
    while(buf[i] != ']'){
        std::string label ="";
        while(buf[i] != ',' && buf[i] != ']'){
            if(buf[i] != '\"')
                label.push_back(buf[i]);
            i++;
        }

        if(buf[i] == ',')
            i++;
        
        arr.push_back(label);
    }
    
    
    return arr;
}
