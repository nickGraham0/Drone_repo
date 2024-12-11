'''

Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Utilized: https://realpython.com/python-sockets/
Description: 
Takes input from GUI.py and sends a formatted socket msg to (pymavlink\server\controller.py) for handling.
Recieves responses (drone location) from (pymavlink\server\controller.py) and transits msg to GUI.py

'''

import socket

def connect_to_server():
    global sock
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 15555))
        print("Connected to server.")
    except Exception as e:
        print(f"Error connecting to server: {e}")

def send(msg):
    global sock
    try:
        
        connect_to_server()

        print(f'Sending command: {msg}')
        sock.sendall(msg.encode('utf8'))

    except Exception as e:
        print(f"Error sending msg: {e}")

def recieve():
    global sock
    try:
        if sock is None:
            connect_to_server()

        sock.settimeout(5.0)
        data = sock.recv(1024).decode('utf8')
        
        return data
    except socket.timeout:
        print("No data received (timeout).")
    except Exception as e:
        print(f"Error receiving drone location: {e}")
