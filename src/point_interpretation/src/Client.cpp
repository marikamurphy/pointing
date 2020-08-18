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

using namespace std;
using namespace Eigen;
using namespace cv;


Client::Client(){

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


vector<Point> Client::sendCV(int sockfd, cv::Mat src)
{
    vector<uchar> buf;
    vector<Point> nullp;
    cv::imencode(".jpg", src, buf);
    // Send size first
    int len = buf.size();
    if (write(sockfd, &len, sizeof(len)) < 0)
    {
        perror("ERROR writing to socket");
        return nullp;
    }
    cout << "sent size" << endl;
    if (write(sockfd, buf.data(), buf.size()) < 0)
    {
        perror("ERROR writing to socket");
        return nullp;
    }
    cout << "sent data" << endl;
    
    //get back coordinates
    char buffer[ARRLEN];
    bzero(buffer, ARRLEN);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
        return nullp;
    }
    printf("%s\n", buffer); //TODO: we will recieve coordinates back, need to actually read these in
    return interpretBuf(buffer);
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
