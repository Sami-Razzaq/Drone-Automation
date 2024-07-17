from pdb import set_trace

from pymavlink import mavutil
from pymavlink.mavutil import mavlink
import time


# Start a connection listening on a UDP port
# the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)    # Jetson Connection


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