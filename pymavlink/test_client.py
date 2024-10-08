import socket
import time

def send_command(command):
    """
    Sends a command to the server running on localhost at port 15555.
    The command is sent as a UTF-8 encoded string.
    """
    try:
        # Create a TCP/IP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            # Connect to the server
            sock.connect(('localhost', 15555))

            # Send the command
            print(f'Sending command: {command}')
            sock.sendall(command.encode('utf8'))

            # You can also read a response from the server if necessary:
            # data = sock.recv(1024)
            # print(f"Received: {data.decode()}")

    except Exception as e:
        print(f"Error: {e}")

# Test sending different commands
def main():
    # Send some example commands to the server
    send_command('up')
    time.sleep(1)  # Wait a second before sending the next command
    send_command('down')
    time.sleep(1)
    send_command('left')
    time.sleep(1)
    send_command('right')
    time.sleep(1)

    # Send waypoint data
    waypoint_data = 'wp 12.34 56.78 90.12'
    send_command(waypoint_data)

# Run the client
if __name__ == "__main__":
    main()
