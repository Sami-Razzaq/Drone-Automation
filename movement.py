# Arms the drone, takesoff to 10m then moves

from pdb import set_trace

from pymavlink import mavutil
from pymavlink.mavutil import mavlink
import time


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

print('Connection Done')

# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()

# Gives the Target System & Target Component of the drone
targetSys = the_connection.target_system
targetComp = the_connection.target_component

print(f"Heartbeat from system - Target System: {targetSys}, Target Compnent: {targetComp}")

# MAV_CMD_DO_SET_MODE
# Change Mode to Guided - Mode Value = 4
# Stabalize=0, AltHold=2, Loiter=5, RTL=6, Land=9
the_connection.mav.command_long_send(targetSys, targetComp, mavlink.MAV_CMD_DO_SET_MODE, 
                                     0, 1, 4, 0, 0, 0, 0, 0)

# Prints the Command Acknowledge - Feedback of command
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(f"Mode Change (Guided) ACK Result: {msg.result}")

# Arms the drone
the_connection.mav.command_long_send(targetSys, targetComp, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                     0, 1, 0, 0, 0, 0, 0, 0)

# Prints the Command Acknowledge - Feedback of command
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(f"Arm ACK Result: {msg.result}")

# Waits 2s
time.sleep(2)

# Takeoff
h = 3
x = 50
y = 50
z = -5
# yaw in radians (1.57 = 90deg)
yaw = 0
yaw_rate = 0.5
the_connection.mav.command_long_send(targetSys, targetComp, mavlink.MAV_CMD_NAV_TAKEOFF, 
                                     0, 0, 0, 0, 0, 0, 0, h)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(f"Takeoff ACK Result: {msg.result}")

# Checks if given Alt is reached, waits some secs and lands
while 1:

    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt
    print(f"Alt: {alt}")
    
    if alt > (h*1000):
        
        if alt > (h*1000) and alt < (h*1000 + 500):
            print("Altitude Reached")
            
        # Sends command to drone to move to 20m forward (x) at 10m height (z)
        # type_mask = 0b11011111000
        # Sets type mask for Position, usage - x,y,z = 0, rest = 1, bit10=0(default)
        # type_mask = 0b10011111000
        # Sets type mask for Position + Yaw Rate, usage - x,y,z,yaw_rate = 0, rest = 1, bit10=0(default)
        # type_mask = 0b01011111000
        # Sets type mask for Position + Yaw, usage - x,y,z,yaw = 0, rest = 1, bit10=0(default)
        # If anyway given but mask = 1, then it will be ignored

        the_connection.mav.send(mavlink.MAVLink_set_position_target_local_ned_message(
            10, targetSys, targetComp, 
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b01011111000, 
            x, y, z, 0, 0, 0, 0, 0, 0, yaw, yaw_rate))
        
        msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp = msg.wp_dist
        print(f"Waypoint Distance: {wp}")
        
        msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        cx = int(msg.x)
        cy= int(msg.y)
        cz= int(msg.z)
        print(f"x: {cx}, y: {cy}, z: {cz}")
        
        if wp == 0 and (cz == z or alt > abs(z*1000)) :
            
            print("Waypoint Reached")
        
            # # Land the drone at the current position
            # the_connection.mav.command_long_send(targetSys, targetComp, mavlink.MAV_CMD_NAV_LAND, 
            #                                     0, 0, 0, 0, 0, 0, 0, 0)

            # msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            # print(f"Land ACK Result: {msg.result}")
            
            # Change Mode to RTL - Mode Value = 6
            the_connection.mav.command_long_send(targetSys, targetComp, mavlink.MAV_CMD_DO_SET_MODE, 
                                                0, 1, 6, 0, 0, 0, 0, 0)

            # Prints the Command Acknowledge - Feedback of command
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(f"Mode Change (RTL) ACK Result: {msg.result}")
            
            break