'''

Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Description: 
Attributed from https://pyshine.com/Socket-programming-and-openc/
Drone Video Feed Socket Rx. Supplies GUI.py with video frame and intruder ID. Retrieves annotated video feed from (pymavlink\server\Object_Detect\PeopleFind.py).  


'''

import socket,cv2, pickle,struct

def init_vid_rx(port_=9998, ip='localhost'):
    global client_socket
    global payload_size
    global data

    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip = ip
    port = port_
    client_socket.connect((host_ip,port)) # a tuple
    data = b""
    payload_size = struct.calcsize("Q")

def vid_rx():
    global client_socket
    global payload_size
    global data

    while True:
        while len(data) < payload_size:
            packet = client_socket.recv(4096)
            if not packet: break
            data+=packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        if len(packed_msg_size) == 0:
            continue
        msg_size = struct.unpack("Q",packed_msg_size)[0]
        
        while len(data) < msg_size:
            data += client_socket.recv(4096)
        frame_data = data[:msg_size]
        data  = data[msg_size:]
        frame_id, frame = pickle.loads(frame_data)
        key = cv2.waitKey(1) & 0xFF
        if key  == ord('q'):
            client_socket.close()
            break

        yield frame_id, frame
