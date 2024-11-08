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

        # Send the command
        print(f'Sending command: {msg}')
        sock.sendall(msg.encode('utf8'))

    except Exception as e:
        print(f"Error sending msg: {e}")

def recieve():
    global sock
    try:
        if sock is None:
            connect_to_server()

        # Wait for the response with a timeout
        sock.settimeout(5.0)  # Set timeout to 5 seconds
        data = sock.recv(1024).decode('utf8')
        #print(f"Received drone location: {data}")
        
        return data
    except socket.timeout:
        print("No data received (timeout).")
    except Exception as e:
        print(f"Error receiving drone location: {e}")
