from pymavlink import mavutil
import time

USE_POSITION        = 0b110111111000  # 0x0DF8 / 3576 (decimal)
USE_VELOCITY        = 0b110111000111  # 0x0DC7 / 3527 (decimal)
USE_ACCELERATION    = 0b110000111111  # 0x0C3F / 3135 (decimal)
USE_POS_VEL         = 0b110111000000  # 0x0DC0 / 3520 (decimal)
USE_POS_VEL_ACCEL   = 0b110000000000  # 0x0C00 / 3072 (decimal)
USE_YAW             = 0b100111111111  # 0x09FF / 2559 (decimal)
USE_YAW_RATE        = 0b010111111111  # 0x05FF / 1535 (decimal)


#start a connection listening to UDP port
#SITL lauches 2 mavlink streams
drone = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for heartbeat... ArduPilot always sends heartbeat
#   Set system and component ID of remote system for the link


drone.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))

# Arm the drone
drone.arducopter_arm()
print("Drone armed")

# Wait until the drone has armed its motors
while not drone.motors_armed():
    print("Waiting for arming...")
    drone.motors_armed_wait()

# Set the drone to GUIDED mode
drone.set_mode('GUIDED')

# Wait for the mode change to take effect
time.sleep(3)

#Position Target msg for Drone
drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, 
                        drone.target_system,                #System ID
                        drone.target_component,             #Flight Controller ID 
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, #Coordinate Frame
                        int(USE_POSITION),                #Byte Mask of Ignored (1) fields:  bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
                        int(-35.3629849 * 10 ** 7),         #Latitude
                        int(149.1649185 * 10 ** 7),         #Longitude
                        10,      #Altitude
                        0, 0, 0, #Velocity: x,y,z
                        0, 0, 0, #Acceleration
                        1.57,    #Yaw
                        0.5))    #Yaw rate


while 1:
    #Lookout for LOCAL_POSITION_NED msgs from Drone
    msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
    time.sleep(1)
