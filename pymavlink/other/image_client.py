import socket

def send_image(image_path, host='127.0.0.1', port=15555):
    # Create a socket object
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((host, port))  # Connect to the server
        with open(image_path, 'rb') as file:  # Open image file in binary mode
            # Send the file in chunks
            while (chunk := file.read(1024)):
                client_socket.sendall(chunk)
        print(f"Image '{image_path}' sent to server")

if __name__ == "__main__":
    send_image('image.jpg')  # Replace with the actual path to the image
