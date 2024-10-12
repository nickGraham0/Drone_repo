import tkinter as tk
from tkinter import ttk, filedialog
import socket
import csv
import datetime 
import os 

sock = None

# Function to save drone location to a CSV file
def save_to_csv(data):
    # Get current date and time
    now = datetime.datetime.now()
    # Format it as MM_DAY_TIME (e.g., 10_07_1530.csv)
    file_name = now.strftime("%m_%d_%H%M") + ".csv"
    
    # Set the path for saving the file (you can change the directory if needed)
    directory = os.getcwd()  # Gets the current working directory
    file_path = os.path.join(directory, file_name)

    with open(file_path, mode='w', newline='') as file:
        csv_writer = csv.writer(file)
        # Assuming data is of the format: "Current Position: x=10, y=5, z=-2"
        headers = ["Latitude", "Longitude", "Altitude"]
        csv_writer.writerow(headers)
        
        # Parse the received data
        parts = data.replace("Current Position: ", "").split(", ")
        x = parts[0].split("=")[1]
        y = parts[1].split("=")[1]
        z = parts[2].split("=")[1]

        # Write the parsed coordinates to the CSV
        csv_writer.writerow([x, y, z])

    print(f"Saved drone location to {file_path}")

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
    send("up\n")

def on_down():
    print("Down button pressed")
    send("down\n")

def on_left():
    print("Left button pressed")
    send("left\n")

def on_right():
    print("Right button pressed")
    send("right\n")

def on_drone_loc():
    print("Drone Location button pressed")
    send("drone_loc\n")
    msg = recieve()
    if msg: 
        print(msg)
        #save_to_csv(msg)

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
            msg = "wp"
            waypoints = []
            for i, row in enumerate(csv_reader):
                lat = float(row['Latitude'])
                lon = float(row['Longitude'])
                waypoints.append(f"{lat} {lon}")
            
            # Concatenate all waypoints with spaces
            msg += " " + " ".join(waypoints)
            print(msg)  # This is for debugging
            
            # Send the complete message to the drone
            send(msg + "\n")  # Assuming send is defined elsewhere
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
