
'''

Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Code Attributed from: 
    https://pyshine.com/Socket-programming-and-openc/

Description: 
    Sends the annotated frames from PeopleFind.py to (pymavlink\client\GUI_vid_rx.py) to be displayed by the GUI.
    
'''

import socket, cv2, pickle,struct,imutils
import numpy as np

def resize_with_padding(image, target_size=(320, 320)):
    target_width, target_height = target_size
    h, w = image.shape[:2]

    scale = min(target_width / w, target_height / h)
    new_width = int(w * scale)
    new_height = int(h * scale)

    resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

    top = (target_height - new_height) // 2
    bottom = target_height - new_height - top
    left = (target_width - new_width) // 2
    right = target_width - new_width - left

    padded_image = cv2.copyMakeBorder(resized_image, top, bottom, left, right,
                                      borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))  # Black padding
    return padded_image


#vid -> var from cv2.VideoCapture
def init_vid_tx(port_=9998, ip='localhost'):

    global server_socket
    global client_socket
    global addr

    # Socket Create
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip = ip
    print('HOST IP:',host_ip)
    port = port_
    socket_address = (host_ip,port)

    server_socket.bind(socket_address)

    server_socket.listen(5)
    print("LISTENING AT:",socket_address)
    
    client_socket,addr = server_socket.accept()


def vid_2_client(vid_frame, id=0):

    global server_socket
    global client_socket
    global addr

    if client_socket:
        vid_frame = resize_with_padding(vid_frame, (640, 480))
        a = pickle.dumps((id, vid_frame))
        message = struct.pack("Q",len(a))+a
        client_socket.sendall(message)
        key = cv2.waitKey(1) & 0xFF
        if key ==ord('q'):
            client_socket.close()
        