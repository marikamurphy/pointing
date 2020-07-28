#include "Client.h"
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#define PORT 3211 

Client::Client(cv::Mat photo){
    _photo = photo;

}

void Client::connection(){
    int sock = 0, valread; 
    int n = 0;
    int bytes = 0;
    struct sockaddr_in serv_addr; 
    char buffer[1024] = {0}; 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
    } 
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
    }

    int photoSize = _photo.total()*_photo.elemSize();
    printf("height: %d width: %d\n", _photo.rows, _photo.cols);

    if((bytes = send(sock, _photo.data, photoSize, 0))<0){
     printf("Error while sending..");
   }
    else{
        printf( "Frame sent sucessfuly" );
        printf( "%d bytes sent.", bytes ); 

    }
   
   valread = read( sock , buffer, 1024); 
   printf("%s\n",buffer ); 
    
} 