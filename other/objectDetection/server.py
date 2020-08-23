# first of all import the socket library 
import socket                
import json 
import cv2
import numpy as np
import objectDetection
INTSIZE = 4
HEADERSIZE = 10


def socketToNumpy(cameraFeed, sockData):
    k=3
    j=cameraFeed.shape[1]
    i=cameraFeed.shape[0]
    sockData = np.fromstring(sockData, np.uint8)
    cameraFeed = np.tile(sockData, 1).reshape((i,j,k))

    return cameraFeed

# next create a socket object 
s = socket.socket()          
print ("Socket successfully created")
  
# reserve a port on your computer in our 
# case it is 12345 but it can be anything 
port = 3211                
  
# Next bind to the port 
# we have not typed any ip in the ip field 
# instead we have inputted an empty string 
# this makes the server listen to requests  
# coming from other computers on the network 
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', port))         
print ("socket binded to %s" %(port) )
  
# put the socket into listening mode 
s.listen(5)      
print ("socket is listening")            
  
# a forever loop until we interrupt it or  
# an error occurs 
running = True
while running:
   numClients = 0 
   while numClients == 0:
        # Establish connection with client. 
        c, addr = s.accept()      
        print ('Got connection from', addr )
        numClients+=1
        c.send(bytes(1))
    
   size = 0
   #read in the size first
   n = c.recv(INTSIZE)
   n = int.from_bytes(n, byteorder='little')
   if n < 0:
       print("ERROR reading from socket (readCV size)")
   print("Size of image: %d\n" %(n))

    # grab until buffer is full, num received = 0, increment until expected
   toread = n
   buf = bytearray(toread)
   view = memoryview(buf)
   while toread:
      nbytes = c.recv_into(view)
      view = view[nbytes:] # slicing views is cheap
      toread -= nbytes
   
   nparr = np.fromstring(bytes(buf), np.uint8)
   image = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
#    cv2.namedWindow("server")
#    cv2.imshow("server", image)
#    key = cv2.waitKey(30)


   #send back coordinates in image
   boxes, pred_cls = objectDetection.object_detection_api(image)
   msg = json.dumps(boxes)
   msg = bytes(msg, "utf-8")
   c.send(msg)
   #send back labels
   msg = json.dumps(pred_cls)
   msg = bytes(msg, "utf-8")
   c.send(msg)
   # Close the connection with the client 
   c.close()