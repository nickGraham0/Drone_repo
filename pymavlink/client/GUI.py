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

from GUI_map import update_drone_loc, refresh_map
import asyncio
import folium

from tkhtmlview import HTMLLabel

VIDEO_FEED = False


#===== INIT =====
if (VIDEO_FEED == True):
    init_vid_rx()

root = tk.Tk()
nb = ttk.Notebook(root)

#===== Map =====

#asyncio.create_task(update_drone_loc())

# Frame for the map
frame5 = ttk.Frame(root)
frame5.pack(fill="both", expand=True)

# Display the map using HTMLLabel from tkhtmlview
map_label = HTMLLabel(frame5, html=open("gui_map.html").read())
map_label.pack(fill="both", expand=True)

#asyncio.create_task(refresh_map(map_label, root))


#===== Async Loop =====

#asyncio.gather(update_drone_loc(), refresh_map(map_label, root))

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

if (VIDEO_FEED):
    frame3 = ttk.Frame(nb)
    label3 = ttk.Label(frame3, text="Video Feed")
    label3.pack(pady=50, padx=20)
    nb.add(frame3, text="Video Feed")

    #https://www.tutorialspoint.com/using-opencv-with-tkinter
    # Define Internal function to loop showing frame
    def show_frames():
        cv2image= cv2.cvtColor(next(vid_rx()),cv2.COLOR_BGR2RGB)
        img = Image.fromarray(cv2image)

        imgtk = ImageTk.PhotoImage(image = img)
        label3.imgtk = imgtk
        label3.configure(image=imgtk)

        label3.after(20, show_frames)

    show_frames()

#===== Frame 4 =====

frame4 = ttk.Frame(nb)

csv_label = ttk.Label(frame4, text="Waypoints")
csv_label.grid(row=0, column=0, padx=10, pady=10)

csv_button = ttk.Button(frame4, text="Select CSV File", command=open_csv)
csv_button.grid(row=0, column=1, padx=10, pady=10)

nb.add(frame4, text="Waypoints")


#===== MainLoop =====#

nb.pack(padx=5, pady=5, expand=True)

# Start the Tkinter event loop

# ===== Async Tasks =====

async def update_drone_loc_task():
    await update_drone_loc()

async def refresh_map_task():
    await refresh_map(map_label, root)

# Function to manage asyncio tasks periodically in Tkinter's main loop
def periodic_asyncio_tasks():
    asyncio.create_task(update_drone_loc_task())
    asyncio.create_task(refresh_map_task())
    root.after(1000, periodic_asyncio_tasks)  # Schedule again in 1 second

# Start the periodic asyncio task runner
root.after(0, periodic_asyncio_tasks)

root.mainloop()
