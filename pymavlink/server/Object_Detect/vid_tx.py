#https://pyshine.com/Socket-programming-and-openc/

# This code is for the server 
# Lets import the libraries
import socket, cv2, pickle,struct,imutils




#vid -> var from cv2.VideoCapture
def init_vid_tx(port_=9998, ip='localhost'):

    global server_socket
    global client_socket
    global addr

    # Socket Create
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    #host_name  = socket.gethostname()
    #host_ip = socket.gethostbyname(host_name)
    host_ip = ip
    print('HOST IP:',host_ip)
    #port = 9999
    port = port_
    socket_address = (host_ip,port)

    # Socket Bind
    server_socket.bind(socket_address)

    # Socket Listen
    server_socket.listen(5)
    print("LISTENING AT:",socket_address)
    
    client_socket,addr = server_socket.accept()


def vid_2_client(vid_frame):

    global server_socket
    global client_socket
    global addr

    # Socket Accept
    #while True:
    print('GOT CONNECTION FROM:',addr)
    if client_socket:
        #vid = cv2.VideoCapture(0)
        
        #while(vid.isOpened()):
        #img,frame = vid_frame.read()
        #vid_frame = imutils.resize(vid_frame,width=320)
        a = pickle.dumps(vid_frame)
        message = struct.pack("Q",len(a))+a
        print(len(message))
        client_socket.sendall(message)
        print("After TX")
        #cv2.imshow('TRANSMITTING VIDEO',frame)
        key = cv2.waitKey(1) & 0xFF
        if key ==ord('q'):
            client_socket.close()
        