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

using namespace std;
using namespace Eigen;


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
    char buffer[256];
    bzero(buffer, 256);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
        return -1;
    }
    
    printf("Successfully connected as client number: %s\n", buffer);
    return sockfd;
    
} 


MatrixXd Client::sendCV(int sockfd, cv::Mat src)
{
    vector<uchar> buf;
    MatrixXd nullp;
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
    char buffer[256];
    bzero(buffer, 256);
    if (read(sockfd, buffer, sizeof(buffer)) < 0)
    {
        perror("ERROR reading from socket");
        return nullp;
    }
    printf("%s\n", buffer); //TODO: we will recieve coordinates back, need to actually read these in
    return interpretBuf(buffer);
}
//TODO
MatrixXd Client::interpretBuf(char *buf){
    MatrixXd coords;
    return coords;
}
