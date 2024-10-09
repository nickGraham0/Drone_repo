from pymavlink import mavutil
import time

import asyncio
from PIL import Image
import io 

USE_POSITION        = 0b110111111000  # 0x0DF8 / 3576 (decimal)
USE_VELOCITY        = 0b110111000111  # 0x0DC7 / 3527 (decimal)
USE_ACCELERATION    = 0b110000111111  # 0x0C3F / 3135 (decimal)
USE_POS_VEL         = 0b110111000000  # 0x0DC0 / 3520 (decimal)
USE_POS_VEL_ACCEL   = 0b110000000000  # 0x0C00 / 3072 (decimal)
USE_YAW             = 0b100111111111  # 0x09FF / 2559 (decimal)
USE_YAW_RATE        = 0b010111111111  # 0x05FF / 1535 (decimal)

arm     = 1
disarm  = 0

altitude = -10

curr_x = 0
curr_y = 0
curr_z = altitude

drone = mavutil.mavlink_connection('udpin:localhost:14550') 

def wait_for_ardupilot_ready():
    print("Waiting for ArduPilot to be ready...")
    
    while True:
        msg = drone.recv_match(blocking=True)
        
        # Check for STATUSTEXT message indicating "ArduPilot Ready"
        if msg.get_type() == 'STATUSTEXT':
            text = msg.text.lower()
            if "ardupilot ready" in text:
                print("ArduPilot is ready!")
                break
        
        # Check if EKF and GPS are initialized and healthy
        elif msg.get_type() == 'EKF_STATUS_REPORT':
            if msg.flags & mavutil.mavlink.EKF_ATTITUDE and msg.flags & mavutil.mavlink.EKF_VELOCITY_HORIZ and msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS:
                print("EKF and GPS are initialized and ready.")
        
        # Check for ARMING_STATE to ensure pre-arm checks have passed
        elif msg.get_type() == 'HEARTBEAT':
            if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == 0:
                print("Pre-arm checks passed. Ready to arm.")
                break


def drone_init():
    #=======================INIT===========================
    drone.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))
    wait_for_ardupilot_ready()

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

def drone_takeoff(takeoff_params, arm_time = 10):
    #======================TAKE OFF=============================
    # Arm the Takeoff System
    drone.mav.command_long_send(drone.target_system, 
                                drone.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                0, 
                                arm, 
                                0, 0, 0, 0, 0, 0)

    arm_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

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
    position_msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if position_msg:
        print(f"Current Position: x={position_msg.x}, y={position_msg.y}, z={position_msg.z}")


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


def drone_control_up(): 

    global curr_x
    global curr_y
    global curr_z

    curr_x += 10

    drone_control()

def drone_control_down(): 

    global curr_x
    global curr_y
    global curr_z

    curr_x -= 10

    drone_control()

def drone_control_left(): 

    global curr_x
    global curr_y
    global curr_z

    curr_y -= 10

    drone_control()

def drone_control_right(): 
    global curr_x
    global curr_y
    global curr_z

    curr_y += 10

    drone_control()

def main():
    takeoff_altitude = 10
    takeoff_params = [0,0,0,0,0,0,10]   #Drone Takeoff to 10m 

    drone_init()
    drone_mode("GUIDED")
    drone_takeoff(takeoff_params, takeoff_altitude)

    asyncio.run(run_server())

# Asynchronous function to handle a connected client
async def handle_client(reader, writer):
    
    print("Handle Client")
    request = None  # Initialize the request variable

    # Read the client's message (up to 255 bytes) and decode from utf8
    request = (await reader.read(255)).decode('utf8').strip()
    
    request = request.strip()

    command = str(request)

    print("found command: " + str(command))
    if command == 'wp':                 # Init Waypoint Path
        request = (await reader.read(4096)).decode('utf8')
        print("Waypoint")

        tokens = request.split()

        params = [float(x) for x in tokens]
        request = params
        print(request)
        
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
      
                      
# Asynchronous function to start the server
async def run_server():
    print("Running Server")
    server = await asyncio.start_server(handle_client, 'localhost', 15555)
    print("After Running Server")
    async with server:
        await server.serve_forever()

# Entry point to run the asyncio server
main()