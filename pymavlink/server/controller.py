'''

Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Description: 
Python Pymavlink Flight Controller Script for handling Drone Movement.
Includes: Arm-Takeoff, Manual Controller, Path Following
To be run in conjuction with pymavlink\client\GUI.py


'''


from pymavlink import mavutil
import time
import asyncio
from PIL import Image
import io 
import math

USE_POSITION        = 0b110111111000  # 0x0DF8 / 3576 (decimal)
USE_VELOCITY        = 0b110111000111  # 0x0DC7 / 3527 (decimal)
USE_ACCELERATION    = 0b110000111111  # 0x0C3F / 3135 (decimal)
USE_POS_VEL         = 0b110111000000  # 0x0DC0 / 3520 (decimal)
USE_POS_VEL_ACCEL   = 0b110000000000  # 0x0C00 / 3072 (decimal)
USE_YAW             = 0b100111111111  # 0x09FF / 2559 (decimal)
USE_YAW_RATE        = 0b010111111111  # 0x05FF / 1535 (decimal)

arm     = 1
disarm  = 0

altitude = -3

curr_x = 0
curr_y = 0
curr_z = altitude


#If running in WSL env and using SITL
#drone = mavutil.mavlink_connection('udpin:localhost:14550') 

#If running with Physical Flight Controller
drone = mavutil.mavlink_connection('COM7', baud=57600)

drone_task_path = None

takeoff_altitude = 3
takeoff_params = [0,0,0,0,0,0,takeoff_altitude]   #Drone Takeoff to 3m 

'''

Description: Sends information to client program
Param: 
    writer  - asyncio writer object that handles communication with client
    msg     - info to write to client 

'''
async def send_to_gui(writer, msg):
    # Send info to Client GUI
    writer.write(msg.encode('utf8'))
    await writer.drain()
    print(f"Sent drone location: {msg}")
    writer.close()
    await writer.wait_closed()

'''

Description: Waits for initial drone signal (heartbeat). Indicates connection with drone

'''
def drone_init():
    drone.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))


'''

Description: Changes the ArduPilot Mode of the Drone.
Main Modes: Guided (Normal Flying mode), Stabilize (RC Controller Mode)
Param: 
    mode - String representing mode to change Drone mode 

'''
def drone_mode(mode = "GUIDED"):
    # Commands a mode change
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        drone.mode_mapping()[mode],  
        0, 0, 0, 0, 0
    )

    print("Set Guided")
    ack_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Allow some time for Drone to change Mode. 
    time.sleep(10)

#https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/arm-disarm.py

'''

Description: Arms the Drone in preperation for takeoff. Repeatedly sends arm command until drone acknowledges (Pre-arm checks satisfied).
Param:
    timeout - Number of attempts to arm drone

'''
def drone_arm(timeout=60):

    # Arm the Takeoff System
    for _ in range(timeout):
        drone.mav.command_long_send(drone.target_system, 
                                    drone.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                    0, 
                                    arm, 
                                    0, 0, 0, 0, 0, 0)

        arm_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if arm_msg and arm_msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if arm_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Drone successfully armed.")
                return True

        # Allow Some time for Drone to Complete Pre-Arm checks
        time.sleep(1)

'''

Description: Commands the Drone to Return-to-launch (RTL). Drone will return to point it launched from.

'''
def drone_rtl():

    drone.mav.command_long_send(drone.target_system, 
                                    drone.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 
                                    0,                
                                    0,0,0,0,0,0,takeoff_altitude)

    rtl_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"rtl ACK:  {rtl_msg}")    


'''

Description: Commands the Drone to land. Drone will gracefully decrease altitude to land.

'''
def drone_land():

    drone.mav.command_long_send(drone.target_system, 
                                    drone.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_LAND, 
                                    0,                
                                    0,0,0,0,0,0,0)

    landing_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Landing ACK:  {landing_msg}")    

'''

Description: Commands drone to Arm and then Takeoff. Drone will takeoff to configured Altitude
Param:
    takeoff_params - Flight Parameters drone will try to reach during takeoff
    arm_time - Time delay to allow drone to arm and takeoff

'''
def drone_takeoff(takeoff_params, arm_time = 10):
    # Arm Drone Motors - Command Arm
    drone_arm()

    # Command Takeoff
    drone.mav.command_long_send(drone.target_system, 
                                drone.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                0,                
                                takeoff_params[0],  #Pitch
                                takeoff_params[1],  #Empty
                                takeoff_params[2],  #Empty 
                                takeoff_params[3],  #Yaw Angle 
                                takeoff_params[4],  #Latitude 
                                takeoff_params[5],  #Longitude  
                                takeoff_params[6])  #Altitude

    takeoff_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")
    
    # Time to allow for drone to takeoff before returning control to script 
    time.sleep(arm_time) 

'''

Description: Commands the Drone to returns it current global location.

'''
def drone_tel_location():
    position_msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if position_msg:
        # Retrieve real Coordinates
        lat = position_msg.lat / 1e7
        lon = position_msg.lon / 1e7
        alt = position_msg.relative_alt / 1000  # Relative altitude in meters

        print(f"Current Position: Latitude={lat}, Longitude={lon}, Altitude={alt}")
        return f"Current Position: Latitude={lat}, Longitude={lon}, Altitude={alt}"

'''

Description: Commands Drone to Move to a Relative Location.

'''
def drone_control():
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, 
                            drone.target_system,                    
                            drone.target_component,                 
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # Coordinate Frame - Local Frame
                            int(USE_POSITION),                      # Byte Mask. Command signals drone to match Position 
                            curr_x, curr_y, curr_z,                 # X, Y, Z positions to reach
                            0, 0, 0,                                # Velocity 
                            0, 0, 0,                                # Acceleration
                            0, 0))                                  # Yaw, yaw rate
    # Debg - See Location of Drone After Command
    drone_tel_location()

'''

https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
Description: Function that calculates the distance (m) between two points. Haversine Formula
Param:
    lat1 - Latitude of Point 1
'''
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Approximate radius of earth in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    #Calculate Distance
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

''' 

Description: Asynchronous function to wait until the drone reaches the waypoint based off its current location.
Param:
    lat_target - The latitude the drone should meet
    lon_taget - The longitude the drone should meet
    altitude_target - The altitude the Drone is to meet
    tolerance - The radial distance from target that is considered on target
    timeout - Time until function timesout

'''
async def wait_until_reached(lat_target, lon_target, altitude_target, tolerance=5, timeout=60):
    start_time = asyncio.get_event_loop().time()
    while True:
        # Get the current position of the drone
        position_msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
        if position_msg:
            # Retrieve real Coordinates
            lat_current = position_msg.lat / 1e7
            lon_current = position_msg.lon / 1e7
            alt_current = position_msg.relative_alt / 1000  # Relative altitude in meters
            
            distance = calculate_distance(lat_current, lon_current, lat_target, lon_target)
            
            # Reached Waypoint condition
            if distance <= tolerance:
                print("Reached waypoint!")
                break
            else:
                print("Tolerance")
                print(distance)
                
        # Check for timeout
        if asyncio.get_event_loop().time() - start_time > timeout:
            print("Timeout: Did not reach waypoint in time.")
            break
            
        await asyncio.sleep(1)


'''

Description: Command, on a loop, for Drone to follow a series of locations.
Params:
    path_coords - Tuples of (lat,lng) of waypoints

'''
async def drone_path(path_coords):

    print(path_coords)
    
    # Command Drone to Follow Each location
    for wp in path_coords:
        lat = wp[0]
        lng = wp[1]
        drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, 
                                drone.target_system,                    
                                drone.target_component,                  
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                int(USE_POSITION),                      
                                int(lat * 1e7),                         # Latitude in degrees
                                int(lng * 1e7),                         # Longitude in degrees                              
                                0, 0, 0,                                # (FIXED since FINAL DEMO) Velocity (x, y, z) = 0, since we're moving by position
                                3,                                      # altitude
                                0, 0, 0,
                                0, 0, 0,                                
                                0, 0))                                  
           
        drone_tel_location()

        print(curr_z)
        # Wait until the drone reaches the waypoint
        await wait_until_reached(lat, lng, curr_z)

'''

Description: Command Drone to move up on x,y plane.

'''
def drone_control_up(): 

    global curr_x
    global curr_y
    global curr_z

    curr_x += 1

    drone_control()

'''

Description: Command Drone to move down on x,y plane.

'''
def drone_control_down(): 

    global curr_x
    global curr_y
    global curr_z

    curr_x -= 1

    drone_control()

'''

Description: Command Drone to move left on x,y plane.

'''
def drone_control_left(): 

    global curr_x
    global curr_y
    global curr_z

    curr_y -= 1

    drone_control()

'''

Description: Command Drone to move right on x,y plane.

'''
def drone_control_right(): 
    global curr_x
    global curr_y
    global curr_z

    curr_y += 1

    drone_control()

'''

Description: Main function for flight controller. Starts asyncio server to handle flight controls

'''
def main():
    try:
        drone_init()
        asyncio.run(run_server())
    finally:
        if drone:
            drone.close()
            print("MAVLink connection closed.")

'''

Description: Asynchronously handle socket requests from GUI client. Serves as a command router that calls functions based on percieved command header.
Params:
    reader - asyncio reader object that takes in information for GUI client
    writer - asyncio writer object that handles communication with GUI client

'''
async def handle_client(reader, writer):
    
    global drone_task_path

    print("Handle Client")
    request = None

    request = (await reader.readline()).decode('utf8').strip()  # Read until newline

    request = request.strip()

    # Split the request and retrieve the first token as command
    tokens = request.split()
    command = tokens[0] if tokens else ""

    print("found command: " + str(command))

    command = str(command)

    #Handle Path Following Init From Client
    #Creates a list of lat,lng that can be used to direct drone
    if command == 'wp':                 # Init Waypoint Path
        print("Waypoint")

        lat_lon_pairs = []
        # Iterate through the remaining tokens in pairs
        for i in range(1, len(tokens), 2):
            try:
                lat = float(tokens[i])
                lon = float(tokens[i+1]) 
                lat_lon_pairs.append((lat, lon))
            except (IndexError, ValueError):
                print(f"Invalid lat/lon pair at index {i}")
                break
        print(lat_lon_pairs)
        drone_mode("GUIDED")
        # Creates a seperate runnable task on asyncio's task loop.
        # Allows for "concurrent" processing w/o having to worry about threads
        drone_task_path = asyncio.create_task(drone_path(lat_lon_pairs))
    
    #Handle takeoff signal from client
    if command == 'takeoff':                  
        print("takeoff") 
        drone_mode("GUIDED")
        drone_takeoff(takeoff_params)

    #Handle land signal from client
    if command == 'land':                  
        print("land") 
        drone_land()

    #Handle RTL signal from client
    if command == 'rtl':                  
        print("rtl") 
        drone_rtl()
        
    #Handle Drone Manual movement
    if command == 'up':                  
        print("up")   
        drone_control_up()
    if command == 'down':                
        print("down")
        drone_control_down()
    if command == 'left':                  
        print("left")
        drone_control_left()
    if command == 'right':                 
        print("right")
        drone_control_right()

    #Handle request for drone location
    if command == 'drone_loc':                 
        print("drone_loc")
        loc = drone_tel_location()     
        if loc:
            await send_to_gui(writer, loc) 

''' 

Description: Asynchronous function to start the asycnio server


'''
async def run_server():
    print("Running Server")
    server = await asyncio.start_server(handle_client, 'localhost', 15555)
    print("After Running Server")
    async with server:
        await server.serve_forever()

main()