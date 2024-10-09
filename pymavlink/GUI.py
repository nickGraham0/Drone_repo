import tkinter as tk
from tkinter import ttk, filedialog
import socket
import csv

sock = None

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
        print(f"Received drone location: {data}")
        return data
    except socket.timeout:
        print("No data received (timeout).")
    except Exception as e:
        print(f"Error receiving drone location: {e}")

def on_up():
    print("Up button pressed")
    send("up")

def on_down():
    print("Down button pressed")
    send("down")

def on_left():
    print("Left button pressed")
    send("left")

def on_right():
    print("Right button pressed")
    send("right")

def on_drone_loc():
    print("Drone Location button pressed")
    send("drone_loc")
    msg = recieve()
    print(msg)

# Create the main window
root = tk.Tk()
nb = ttk.Notebook(root)

# Frame 1 and 2
frame1 = ttk.Frame(nb)
frame2 = ttk.Frame(nb)

# Label in frame2
label2 = ttk.Label(frame2, text="This is Window Two")
label2.pack(pady=50, padx=20)

# New button in Window 2 to send 'drone_loc'
drone_loc_button = ttk.Button(frame2, text="Get Drone Location", command=on_drone_loc)
drone_loc_button.pack(pady=20)

def open_csv():
    # Open a file dialog to select the CSV file
    file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
    if file_path:
        print(f"Selected file: {file_path}")
        read_and_process_csv(file_path)

def read_and_process_csv(filename):
    # Read the CSV file and print the waypoints
    try:
        with open(filename, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for i, row in enumerate(csv_reader):
                lat = float(row['latitude'])
                lon = float(row['longitude'])
                alt = float(row['altitude'])
                print(f"Waypoint {i}: Latitude={lat}, Longitude={lon}, Altitude={alt}")
                # You can send the waypoint to the drone here using the send_waypoint function
    except Exception as e:
        print(f"Error reading CSV file: {e}")





# Label in frame1
label1 = ttk.Label(frame1, text="This is Window One")
label1.pack(pady=50, padx=20)

# Button Frame in frame1
button_frame = ttk.Frame(frame1)
button_frame.pack(pady=20)

# Up Button
up_button = ttk.Button(button_frame, text="Up", command=on_up)
up_button.grid(row=0, column=1, padx=10, pady=5)

# Down Button
down_button = ttk.Button(button_frame, text="Down", command=on_down)
down_button.grid(row=2, column=1, padx=10, pady=5)

# Left Button
left_button = ttk.Button(button_frame, text="Left", command=on_left)
left_button.grid(row=1, column=0, padx=10, pady=5)

# Right Button
right_button = ttk.Button(button_frame, text="Right", command=on_right)
right_button.grid(row=1, column=2, padx=10, pady=5)

# Create a new frame for the input fields (Frame 4)
frame4 = ttk.Frame(nb)

# Create a label and CSV file upload button in Frame 4
csv_label = ttk.Label(frame4, text="Upload Waypoints CSV:")
csv_label.grid(row=0, column=0, padx=10, pady=10)

csv_button = ttk.Button(frame4, text="Select CSV File", command=open_csv)
csv_button.grid(row=0, column=1, padx=10, pady=10)

# Add the frames to the notebook as separate tabs
nb.add(frame1, text="Window 1")
nb.add(frame2, text="Window 2")

# Frame 3 (extra window)
frame3 = ttk.Frame(nb)
label3 = ttk.Label(frame3, text="This is Window Three")
label3.pack(pady=50, padx=20)
nb.add(frame3, text="Window 3")

# Add frame4 with input fields as another tab
nb.add(frame4, text="Upload Waypoints")

# Pack the notebook into the root window
nb.pack(padx=5, pady=5, expand=True)

# Start the Tkinter event loop
root.mainloop()
