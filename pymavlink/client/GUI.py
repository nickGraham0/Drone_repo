import tkinter as tk
from tkinter import ttk, filedialog
from tkinter import *
import socket
import csv
import datetime 
import os 
from io import BytesIO
from PIL import Image, ImageTk
import math
import cv2 
from GUI_vid_rx import init_vid_rx, vid_rx
from GUI_port import connect_to_server, send, recieve
from GUI_csv_util import save_to_csv, open_csv, read_and_process_csv
from GUI_handlers import on_down, on_left, on_right, on_up, on_drone_loc

from tkinter import Toplevel

VID_CHECK = True


#===== INIT =====
if VID_CHECK == True:
    init_vid_rx()

root = tk.Tk()
nb = ttk.Notebook(root)


#=====Variables=====


video_labels = []


seen_ids = {0}  # Track unique IDs
shown_id = tk.IntVar()  # Currently displayed ID for Intruders
shown_id.set(0)  # Default to ID 0

# Attributed from https://stackoverflow.com/questions/28412496/updating-optionmenu-from-list
def update_dropdown_menu(dropdown, option_var, options):
    menu = dropdown["menu"]
    menu.delete(0, "end")  # Clear existing options
    for option in options:
        menu.add_command(label=option, command=lambda value=option: option_var.set(value))

#Attributed from https://www.tutorialspoint.com/how-do-i-create-a-popup-window-in-tkinter
def create_popup(new_id):
    popup = Toplevel(root)
    popup.title(f"New Intruder Detected: ID {new_id}")
    popup.geometry("300x100")

    label = tk.Label(popup, text=f"Intruder detected with ID: {new_id}", font=("Arial", 12))
    label.pack(pady=20)

    ok_button = tk.Button(popup, text="OK", command=popup.destroy)
    ok_button.pack(pady=5)

#Attributed from https://www.geeksforgeeks.org/how-to-show-webcam-in-tkinter-window-python/
def show_frames(labels):
    global seen_ids

    skip = False
    id, frame = next(vid_rx())  # Fetch ID and frame
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Add new ID to seen_ids and update dropdown menu
    if id not in seen_ids:
        seen_ids.add(int(id))
        update_dropdown_menu(id_dropdown, shown_id, sorted(seen_ids))  # Update dropdown
        create_popup(int(id))  # Show popup for new ID

    # Check if the current ID matches the selected one
    if int(id) == 0:
        label = video_labels[0]  # Default video feed
    elif int(id) == shown_id.get():
        label = video_labels[1]  # Intruder feed
    else:
        skip = True

    if not skip:
        # Update the label with the current frame
        img = Image.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)
        label.imgtk = imgtk
        label.configure(image=imgtk)

    # Schedule the next frame update
    root.after(20, lambda: show_frames(labels))





#===== Frame 1 =====

frame1 = ttk.Frame(nb)
label1 = ttk.Label(frame1, text="Manual Control")
label1.pack(pady=50, padx=20)
nb.add(frame1, text="Manual Control")

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


#===== Frame 2 =====

frame2 = ttk.Frame(nb)

label2 = ttk.Label(frame2, text="Drone Location")
label2.pack(pady=50, padx=20)
nb.add(frame2, text="Drone Location")

drone_loc_button = ttk.Button(frame2, text="Get Drone Location", command=on_drone_loc)
drone_loc_button.pack(pady=20)

#===== Frame 3 =====

frame3 = ttk.Frame(nb)
label3 = ttk.Label(frame3, text="Video Feed")
label3.pack(pady=50, padx=20)
nb.add(frame3, text="Video Feed")

if (VID_CHECK == True):
    #https://www.tutorialspoint.com/using-opencv-with-tkinter
    # Define Internal function to loop showing frame
    video_labels.append(label3)

#===== Frame 5 =====

frame5 = ttk.Frame(nb)
label5 = ttk.Label(frame5, text="Intruders")
label5.pack(pady=50, padx=20)

if (VID_CHECK == True):
    #https://www.tutorialspoint.com/using-opencv-with-tkinter
    # Define Internal function to loop showing frame
    video_labels.append(label5)

# Add dropdown to select ID for Intruders output
id_dropdown_label = tk.Label(frame5, text="Select ID to View:")
id_dropdown_label.pack(pady=5)

id_dropdown = tk.OptionMenu(frame5, shown_id, *sorted(seen_ids))  # Dropdown menu
id_dropdown.pack(pady=10)

nb.add(frame5, text="Intruders")

#===== Frame 4 =====

frame4 = ttk.Frame(nb)

csv_label = ttk.Label(frame4, text="Waypoints")
csv_label.grid(row=0, column=0, padx=10, pady=10)

csv_button = ttk.Button(frame4, text="Select CSV File", command=open_csv)
csv_button.grid(row=0, column=1, padx=10, pady=10)

nb.add(frame4, text="Waypoints")


#===== MainLoop =====#

nb.pack(padx=5, pady=5, expand=True)

show_frames(video_labels)

# Start the Tkinter event loop
root.mainloop()



