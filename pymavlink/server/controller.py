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

#drone = mavutil.mavlink_connection('udpin:localhost:14550') 
drone = mavutil.mavlink_connection('COM7', baud=57600)
#drone = mavutil.mavlink_connection('COM5') 
#print(f"Baud rate used by the connection: {drone.port.baudrate}")

#drone = mavutil.mavlink_connection('192.168.4.2:5760') 

drone_task_path = None

takeoff_altitude = 3
takeoff_params = [0,0,0,0,0,0,takeoff_altitude]   #Drone Takeoff to 3m 


async def send_to_gui(writer, msg):
    # Send the drone's location back to the client
    writer.write(msg.encode('utf8'))
    await writer.drain()
    print(f"Sent drone location: {msg}")
    
    # Close the connection after handling the request
    writer.close()
    await writer.wait_closed()

def wait_for_ardupilot_ready(timeout=60):
    print("Waiting for ArduPilot to be ready...")

    barometer1_calibrated = False
    barometer2_calibrated = False
    gyro_initialized = False
    ardupilot_ready = False
    ahrs_active = False
    ekf_imu0_initialized = False
    ekf_imu1_initialized = False
    imu0_alignment_complete = False
    imu1_alignment_complete = False
    gps_detected = False
    fence_present = False
    field_elevation_set = False
    pre_arm_good = False

    while True:
        msg = drone.recv_match(blocking=True)

        # Barometer calibration
        if msg.get_type() == 'STATUSTEXT':
            text = msg.text.lower()
            if "barometer 1 calibration complete" in text:
                barometer1_calibrated = True
                print("Barometer 1 calibrated")
            elif "barometer 2 calibration complete" in text:
                barometer2_calibrated = True
                print("Barometer 2 calibrated")
            elif "ardupilot ready" in text:
                ardupilot_ready = True
                print("ArduPilot Ready")
            elif "pre-arm good" in text:
                pre_arm_good = True
                print("Pre-arm checks passed")

        # Mode check: STABILIZE
        elif msg.get_type() == 'HEARTBEAT':
            mode = mavutil.mode_string_v10(msg)
            if mode == 'STABILIZE':
                print(f"Mode: {mode}")

        # Gyroscope initialization
        elif msg.get_type() == 'GYROCAL':
            gyro_initialized = True
            print("Gyro initialized")

        # AHRS and EKF3 IMU Initialization
        elif msg.get_type() == 'AHRS2':
            if msg.get_type() == 'AHRS2':
                ahrs_active = True
                print("AHRS active")
        
        elif msg.get_type() == 'EKF_STATUS_REPORT':
            # IMU Initialization
            if msg.flags & mavutil.mavlink.EKF_ATTITUDE and msg.flags & mavutil.mavlink.EKF_VELOCITY_HORIZ and msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS:
                ekf_imu0_initialized = True
                ekf_imu1_initialized = True
                print("EKF3 IMU0 and IMU1 initialized and ready")
            # IMU Alignment Complete
            if msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS:
                imu0_alignment_complete = True
                imu1_alignment_complete = True
                print("IMU0 and IMU1 alignment complete")

        # GPS initialization and detection
        elif msg.get_type() == 'GPS_RAW_INT':
            if msg.fix_type >= 3:  # Assuming 3D Fix
                gps_detected = True
                print(f"GPS detected: {msg.fix_type}")

        # Fence
        elif msg.get_type() == 'FENCE_STATUS':
            if msg.breach_status == 0:  # No fence breach
                fence_present = True
                print("Fence present")

        # Field Elevation
        elif msg.get_type() == 'STATUSTEXT' and "Field Elevation Set" in msg.text:
            field_elevation_set = True
            print(f"Field Elevation Set: {msg.text}")

        # Check if all conditions are met
        if (imu0_alignment_complete and imu1_alignment_complete):
            print("All pre-flight checks complete. Ready to proceed.")
            break



def drone_init():
    #=======================INIT===========================
    drone.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))
    #wait_for_ardupilot_ready()

def drone_mode(mode = "GUIDED"):
    #======================GUIDED MODE=============================
    # Set the drone to GUIDED mode
    # Set the vehicle to GUIDED mode
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        drone.mode_mapping()[mode],  # Mode index for GUIDED
        0, 0, 0, 0, 0
    )

    print("Set Guided")
    #Ensure that Proper Acknowledgement for mode change 
    ack_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    time.sleep(10)

#https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/arm-disarm.py
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

        time.sleep(1)


def drone_rtl():

    drone.mav.command_long_send(drone.target_system, 
                                    drone.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 
                                    0,                
                                    0,0,0,0,0,0,takeoff_altitude)

    rtl_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"rtl ACK:  {rtl_msg}")    

def drone_land():

    drone.mav.command_long_send(drone.target_system, 
                                    drone.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_LAND, 
                                    0,                
                                    0,0,0,0,0,0,0)

    landing_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Landing ACK:  {landing_msg}")    

def drone_takeoff(takeoff_params, arm_time = 10):
    #======================TAKE OFF=============================
    # Arm the Takeoff System
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
    
    time.sleep(arm_time) 

def drone_tel_location():
    position_msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if position_msg:
        lat = position_msg.lat / 1e7  # Latitude in degrees
        lon = position_msg.lon / 1e7  # Longitude in degrees
        alt = position_msg.relative_alt / 1000  # Relative altitude in meters

        print(f"Current Position: Latitude={lat}, Longitude={lon}, Altitude={alt}")
        return f"Current Position: Latitude={lat}, Longitude={lon}, Altitude={alt}"
    
def drone_control():
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, 
                            drone.target_system,                    #System ID
                            drone.target_component,                 #Flight Controller ID 
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    #Coordinate Frame
                            int(USE_POSITION),                      #Byte Mask of Ignored (1) fields:  bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
                            curr_x, curr_y, curr_z,                             # X, Y, Z positions (5 meters forward, 0 right, -10 meters (going up))
                            0, 0, 0,                                # Velocity (x, y, z) = 0, since we're moving by position
                            0, 0, 0,                                # Acceleration (x, y, z) = 0, not used here
                            0, 0))                                  # Yaw, yaw rate (not changing yaw in this example)
    drone_tel_location()

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Approximate radius of earth in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

# Asynchronous function to wait until the drone reaches the waypoint
async def wait_until_reached(lat_target, lon_target, altitude_target, tolerance=5, timeout=60):
    start_time = asyncio.get_event_loop().time()
    while True:
        # Get the current position of the drone
        position_msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if position_msg:
            lat_current = position_msg.lat / 1e7
            lon_current = position_msg.lon / 1e7
            alt_current = position_msg.relative_alt / 1000  # Relative altitude in meters
            
            # Calculate the distance to the target waypoint
            distance = calculate_distance(lat_current, lon_current, lat_target, lon_target)
            
            #print(f"Current Position: Latitude={lat_current}, Longitude={lon_current}, Altitude={alt_current}")
            #print(f"Distance to target: {distance} meters, Altitude difference: {abs(alt_current - altitude_target)} meters")
            
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
            
        await asyncio.sleep(1)  # Non-blocking sleep, check every second


# Drone will continuously follow path
async def drone_path(path_coords):

    print(path_coords)
    #while True:
    for wp in path_coords:
        lat = wp[0]
        lng = wp[1]
        drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, 
                                drone.target_system,                    #System ID
                                drone.target_component,                 #Flight Controller ID 
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,    #Coordinate Frame
                                int(USE_POSITION),                      #Byte Mask of Ignored (1) fields:  bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
                                int(lat * 1e7),                         # Latitude in degrees * 1e7 (MAVLink expects it in this format)
                                int(lng * 1e7),                         # Longitude in degrees * 1e7 (MAVLink expects it in this format)                                0, 0, 0,                                # Velocity (x, y, z) = 0, since we're moving by position
                                3,                                     # altitude
                                0, 0, 0,
                                0, 0, 0,                                # Acceleration (x, y, z) = 0, not used here
                                0, 0))                                  # Yaw, yaw rate (not changing yaw in this example)
        
        
        
        drone_tel_location()

        # Wait until the drone reaches the waypoint

        print(curr_z)
        await wait_until_reached(lat, lng, curr_z)

def drone_control_up(): 

    global curr_x
    global curr_y
    global curr_z

    curr_x += 1

    drone_control()

def drone_control_down(): 

    global curr_x
    global curr_y
    global curr_z

    curr_x -= 1

    drone_control()

def drone_control_left(): 

    global curr_x
    global curr_y
    global curr_z

    curr_y -= 1

    drone_control()

def drone_control_right(): 
    global curr_x
    global curr_y
    global curr_z

    curr_y += 1

    drone_control()

def main():
    try:
        #takeoff_altitude = 10
        #takeoff_params = [0,0,0,0,0,0,10]   #Drone Takeoff to 10m 
        drone_init()
        #exit()
        #drone_mode("GUIDED")
        #drone_takeoff(takeoff_params, takeoff_altitude)
        asyncio.run(run_server())
    finally:
        # Close the MAVLink connection when the program finishes
        if drone:
            drone.close()
            print("MAVLink connection closed.")

# Asynchronous function to handle a connected client
async def handle_client(reader, writer):
    
    global drone_task_path

    print("Handle Client")
    request = None  # Initialize the request variable

    # Read the client's message (up to 255 bytes) and decode from utf8
    request = (await reader.readline()).decode('utf8').strip()  # Read until newline

    request = request.strip()

    # Split the request and retrieve the first token as command
    tokens = request.split()
    command = tokens[0] if tokens else ""

    print("found command: " + str(command))

    command = str(command)

    if command == 'wp':                 # Init Waypoint Path
        print("Waypoint")

        lat_lon_pairs = []
        # Iterate through the remaining tokens in pairs
        for i in range(1, len(tokens), 2):  # Start from index 1, step 2 (pair by pair)
            try:
                lat = float(tokens[i])
                lon = float(tokens[i+1])  # Make sure there is a corresponding longitude
                lat_lon_pairs.append((lat, lon))
            except (IndexError, ValueError):
                print(f"Invalid lat/lon pair at index {i}")
                break
        print(lat_lon_pairs)
        drone_mode("GUIDED")
        drone_task_path = asyncio.create_task(drone_path(lat_lon_pairs))
        #drone_path(lat_lon_pairs)
        
    if command == 'takeoff':                  
        print("takeoff") 
        drone_mode("GUIDED")
        drone_takeoff(takeoff_params)

    if command == 'land':                  
        print("land") 
        #drone_mode("GUIDED")
        drone_land()

    if command == 'rtl':                  
        print("rtl") 
        #drone_mode("GUIDED")
        drone_rtl()
        
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

    if command == 'drone_loc':                 
        print("drone_loc")
        loc = drone_tel_location()     
        if loc:
            await send_to_gui(writer, loc) 

# Asynchronous function to start the server
async def run_server():
    print("Running Server")
    server = await asyncio.start_server(handle_client, 'localhost', 15555)
    print("After Running Server")
    async with server:
        await server.serve_forever()

# Entry point to run the asyncio server
main()