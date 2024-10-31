import csv
import datetime 
from tkinter import ttk, filedialog
from GUI_server import send
import os 

def save_to_csv(data):
    now = datetime.datetime.now()
    file_name = now.strftime("%m_%d_%H%M") + ".csv"
    
    directory = os.getcwd()  # Gets the current working directory
    file_path = os.path.join(directory, file_name)

    with open(file_path, mode='w', newline='') as file:
        csv_writer = csv.writer(file)
        # Assuming data is of the format: "Current Position: x=10, y=5, z=-2"
        headers = ["Latitude", "Longitude", "Altitude"]
        csv_writer.writerow(headers)
        
        parts = data.replace("Current Position: ", "").split(", ")
        x = parts[0].split("=")[1]
        y = parts[1].split("=")[1]
        z = parts[2].split("=")[1]

        csv_writer.writerow([x, y, z])

    print(f"Saved drone location to {file_path}")

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