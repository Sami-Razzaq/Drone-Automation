from pdb import set_trace

from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
# the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
# the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14551')    # Use Secondary Mavlink Stream
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)    # Jetson Connection

print('Connection Done')

# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages

while True:

    # Gets message ATTITUDE from drone
    msg = the_connection.recv_match(blocking=True)
    # msg = the_connection.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=True)
    # msg = the_connection.messages['GLOBAL_POSITION_INT'].relative_alt
    # set_trace()
    print(msg)

    # Wait 3 seconds
    # time.sleep(2)