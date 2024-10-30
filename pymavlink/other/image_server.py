import socket

def start_server(host='127.0.0.1', port=15555):
    # Create a socket object
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((host, port))  # Bind to the address and port
        server_socket.listen(1)  # Listen for incoming connections
        print(f"Server listening on {host}:{port}")
        
        conn, addr = server_socket.accept()  # Accept a connection
        with conn:
            print(f"Connected by {addr}")
            with open('rcved_image.jpg', 'wb') as file:  # Open file to write the image
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    file.write(data)
            print("Image received and saved as 'received_image.jpg'")

if __name__ == "__main__":
    start_server()
