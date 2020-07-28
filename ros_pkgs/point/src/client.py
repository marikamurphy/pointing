# Import socket module 
import socket                
HEADERSIZE = 10  
 
def client(img, threshold):
    # Create a socket object
    s = socket.socket()          
    
    # Define the port on which you want to connect 
    port = 12345                
    
    # connect to the server on local computer 
    s.connect(('127.0.0.1', port)) 
    
    while True:
        full_msg = b''
        new_msg = True
        while True:
            msg = s.recv(16)
            if new_msg:
                msglen = int(msg[:HEADERSIZE])
                new_msg = False


            full_msg += msg

            print(len(full_msg))

            if len(full_msg)-HEADERSIZE == msglen:
                print("full msg recvd")
                print(full_msg[HEADERSIZE:])
                new_msg = True
                full_msg = b""
                # close the connection
                s.close()
                return

