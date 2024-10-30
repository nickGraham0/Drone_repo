#!/bin/bash

# Start MAVProxy with the ArduCopter SITL simulation and console, running in the background
echo "Starting MAVProxy..."
sim_vehicle.py -v ArduCopter -L CobbCountyRC --console --map --out=udp:127.0.0.1:14550 MAVPROXY_PID=$!
echo "MAVProxy started with PID: $MAVPROXY_PID"


# Start your custom Python server (running your drone communication logic)
echo "Starting Python server..."
python3 listen.py &
SERVER_PID=$!
echo "Server started with PID: $SERVER_PID"



# Wait a few seconds to ensure the server starts up properly
sleep 5

# Start your Python GUI
echo "Starting Python GUI..."
python3 GUI.py
GUI_PID=$!
echo "GUI started with PID: $GUI_PID"

# Wait for all processes to complete (optional)
wait $GUI_PID

# Optional: Monitor processes and output status
echo "All processes are running."
