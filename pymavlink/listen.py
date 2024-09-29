from pymavlink import mavutil
import time

USE_POSITION        = 0b110111111000  # 0x0DF8 / 3576 (decimal)
USE_VELOCITY        = 0b110111000111  # 0x0DC7 / 3527 (decimal)
USE_ACCELERATION    = 0b110000111111  # 0x0C3F / 3135 (decimal)
USE_POS_VEL         = 0b110111000000  # 0x0DC0 / 3520 (decimal)
USE_POS_VEL_ACCEL   = 0b110000000000  # 0x0C00 / 3072 (decimal)
USE_YAW             = 0b100111111111  # 0x09FF / 2559 (decimal)
USE_YAW_RATE        = 0b010111111111  # 0x05FF / 1535 (decimal)

arm     = 1
disarm  = 0

takeoff_altitude = 10
tgt_sys_id  = 1 # 1 -> default
tgt_comp_id = 1 # 1 -> Autopilot
takeoff_params = [0,0,0,0,0,0,10]   #Drone Takeoff to 10m 
#start a connection listening to UDP port
#SITL lauches 2 mavlink streams
drone = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for heartbeat... ArduPilot always sends heartbeat
#   Set system and component ID of remote system for the link

def drone_loop():
    #=======================INIT===========================
    drone.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))


    #======================Arm=============================

    #Use command long, a generic command function 
    ''' 
    drone.mav.command_long_send(drone.target_system,        #System ID
                                drone.target_component,     #Flight Controller ID
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                0, 
                                arm,                        #Set Arm/Disarm
                                0, 0, 0, 0, 0, 0)           #Unused

    #Wait for ack from drone for arming before continue
    msg = drone.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    '''


    #======================GUIDED MODE=============================
    # Set the drone to GUIDED mode
    # Set the vehicle to GUIDED mode
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        drone.mode_mapping()["GUIDED"],  # Mode index for GUIDED
        0, 0, 0, 0, 0
    )

    print("Set Guided")
    #Ensure that Proper Acknowledgement for mode change 
    ack_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    ''' 
    # Raise throttle to 50% (1500 on channel 3)
    drone.mav.rc_channels_override_send(
        drone.target_system,   # System ID
        drone.target_component,  # Component ID (usually autopilot)
        0,                     # Channel 1 (leave unchanged)
        0,                     # Channel 2 (leave unchanged)
        3000,                  # Channel 3 (Throttle -> set to 50%)
        0,                     # Channel 4 (leave unchanged)
        0, 0, 0, 0            # Other channels (leave unchanged)
    )
    print("Trottle")
    '''


    #Ensure that Proper Acknowledgement for mode change 
    ack_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Trottle ACK:  {ack_msg}")

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


    #======================CONTROL DRONE=============================
    time.sleep(10) 
    #Position Target msg for Drone
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, 
                            drone.target_system,                #System ID
                            drone.target_component,             #Flight Controller ID 
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED, #Coordinate Frame
                            int(USE_POSITION),                  #Byte Mask of Ignored (1) fields:  bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
                            10, 0, -10,                  # X, Y, Z positions (5 meters forward, 0 right, -10 meters up)
                            0, 0, 0,                    # Velocity (x, y, z) = 0, since we're moving by position
                            0, 0, 0,                    # Acceleration (x, y, z) = 0, not used here
                            0, 0))                         # Yaw, yaw rate (not changing yaw in this example)
    move_msg = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Move ACK:  {move_msg}")

    while 1:
        #Lookout for LOCAL_POSITION_NED msgs from Drone
        #msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        #print(msg)
        #print(drone.target_system)
        #print(drone.target_component)

        msg = drone.recv_match(type='STATUSTEXT', blocking=True)
        print(f"Status message: {msg.text}")
        time.sleep(1)


def main():
    # Example waypoints (latitude, longitude, altitude)
    # Waypoints near Cobb County RC Club (CobbCountyRC)
    waypoints = [
        (34.013819, -84.724446, 10),  # CobbCountyRC, at 10 meters altitude
        (34.014500, -84.724000, 10),  # 80 meters northeast
        (34.013500, -84.725000, 10),  # 80 meters southwest
        (34.013000, -84.724200, 10),  # 90 meters southeast
        (34.014000, -84.725500, 10)   # 120 meters northwest
    ]

    drone_loop()