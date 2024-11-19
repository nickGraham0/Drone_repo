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

video_labels = []


def add_video_label(frame, frame_id):
    """Add a new label for the detected intruder video feed."""
    global video_labels

    # Check if frame_id already exists
    if frame_id < len(video_labels):
        label = video_labels[frame_id]
    else:
        # Create a new label for the detection
        label = tk.Label(video_grid_frame, text=f"Video {frame_id + 1}", width=20, height=10, bg="black", fg="white")
        label.grid(row=len(video_labels) // 3, column=len(video_labels) % 3, padx=5, pady=5)
        video_labels.append(label)

    # Display the frame in the label
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    label.imgtk = imgtk
    label.configure(image=imgtk)

def show_frames(labels):
    try:
        frame_id, frame = next(vid_rx())

        # Add or update video label for this frame_id
        add_video_label(frame, int(frame_id))

        # Schedule the next frame update
        root.after(20, lambda: show_frames(labels))
    except StopIteration:
        print("Video stream ended.")



#===== INIT =====
if VID_CHECK == True:
    init_vid_rx()

root = tk.Tk()
nb = ttk.Notebook(root)

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
label5.pack(pady=10, padx=20)
video_grid_frame = ttk.Frame(frame5)
video_grid_frame.pack(pady=20)

if (VID_CHECK == True):    
    ''' 
    for r in range(3):
            for c in range(3):
                index = r * 3 + c
                # Create a Label for each video feed
                label = tk.Label(video_grid_frame, text=f"Video {index + 1}", width=100, height=100, bg="black", fg="white")
                label.grid(row=r, column=c, padx=5, pady=5)
    '''
    #video_labels.append(label)

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



