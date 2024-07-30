from pdb import set_trace

from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)    # Jetson Connection
# the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

print('Connection Done')

# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" 
      % (the_connection.target_system, the_connection.target_component))

while True:

    # Arms the drone
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
    # Prints the Command Acknowledge - Feedback of command
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # Waits 5s
    time.sleep(5)

    # Disarms the drone
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    msg1 = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg1)
    # Waits 5s
    time.sleep(5)