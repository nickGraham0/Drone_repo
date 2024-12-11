import socket
import cv2
import pickle
import struct

def receive_video(ip='localhost', port=9998):
    """Connect to the server and receive video frames."""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket_address = (ip, port)

    print(f"Connecting to server at {ip}:{port}...")
    client_socket.connect(socket_address)
    print("Connected to the server!")

    data = b""
    payload_size = struct.calcsize("Q")

    try:
        while True:
            # Receive data in chunks
            while len(data) < payload_size:
                packet = client_socket.recv(4096)  # 4096 bytes
                if not packet:
                    print("Connection closed by server.")
                    return
                data += packet

            # Unpack the size of the incoming frame
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            # Retrieve the full frame
            while len(data) < msg_size:
                data += client_socket.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            # Deserialize the frame and its ID
            frame_id, frame = pickle.loads(frame_data)

            # Display the received frame
            cv2.imshow(f"Video Stream (ID: {frame_id})", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"Error receiving video: {e}")
    finally:
        print("Closing connection...")
        client_socket.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    receive_video(ip='localhost', port=9998)
