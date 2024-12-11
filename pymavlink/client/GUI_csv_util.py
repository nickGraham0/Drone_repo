'''

Utilized: https://www.geeksforgeeks.org/working-csv-files-python/
Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Description: 
Utility file that either constructs a csv file or processes and sends the formatted contents of csv file to (pymavlink\server\controller.py)

'''

import csv
import datetime 
from tkinter import ttk, filedialog
from GUI_port import send
import os 

def save_to_csv(data):
    now = datetime.datetime.now()
    file_name = now.strftime("%m_%d_%H%M") + ".csv"
    
    directory = os.getcwd()  # Gets the current working directory
    file_path = os.path.join(directory, file_name)

    with open(file_path, mode='w', newline='') as file:
        csv_writer = csv.writer(file)
        headers = ["Latitude", "Longitude", "Altitude"]
        csv_writer.writerow(headers)
        
        parts = data.replace("Current Position: ", "").split(", ")
        x = parts[0].split("=")[1]
        y = parts[1].split("=")[1]
        z = parts[2].split("=")[1]

        csv_writer.writerow([x, y, z])

    print(f"Saved drone location to {file_path}")

def open_csv():
    file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
    if file_path:
        print(f"Selected file: {file_path}")
        read_and_process_csv(file_path)

def read_and_process_csv(filename):
    try:
        with open(filename, mode='r') as file:
            csv_reader = csv.DictReader(file)
            msg = "wp"
            waypoints = []
            for i, row in enumerate(csv_reader):
                lat = float(row['Latitude'])
                lon = float(row['Longitude'])
                waypoints.append(f"{lat} {lon}")
            
            msg += " " + " ".join(waypoints)
            print(msg)
            
            send(msg + "\n")
    except Exception as e:
        print(f"Error reading CSV file: {e}")