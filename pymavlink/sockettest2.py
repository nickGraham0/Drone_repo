import socket 
import pickle 

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.connect(('172.20.10.10', 6681))

data = b""
''' 
while True:
    packet = server_socket.recv(4096*2)
    if not packet: break
    data += packet
    print("getting packet")
'''

packet = server_socket.recv(128)
#print(len(packet))
#packet += server_socket.recv(4096)

#data = server_socket.recv(1024*100)
array = pickle.loads(data)
#print(data)

server_socket.close()