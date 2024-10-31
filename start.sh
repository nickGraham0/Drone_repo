#!/bin/bash

# Navigate to the server directory and start server scripts in the background
echo "Starting server processes..."

# Start listen.py in the background (assuming it's fine to run without a specific venv)
python3 ./pymavlink/server/listen.py &
listen_pid=$!
echo "listen.py started with PID $listen_pid"

# Activate the virtual environment for detect.py and start it in the background
echo "Activating virtual environment for detect.py..."
source ./mnt/c/Users/Nick/Desktop/drone_obj_detect_local/objDet_env/bin/activate
python3 ./pymavlink/Object_Detect/detect.py &
detect_pid=$!
echo "detect.py started with PID $detect_pid in virtual environment"
deactivate  # Deactivate the virtual environment for detect.py

# Wait a moment to ensure the servers are up and running
sleep 2

# Start the client without using the specific venv for detect.py
echo "Starting client..."
python3 ./pymavlink/client/GUI.py
client_pid=$!
echo "GUI.py started with PID $client_pid"

# Wait for the client to exit
wait $client_pid

# Once the client exits, kill the server processes
echo "Shutting down server processes..."
kill $listen_pid $detect_pid
echo "All processes stopped."
