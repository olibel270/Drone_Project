from pymavlink import mavutil
import time
import random

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpout:localhost:14650')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

# Once connected, use 'the_connection' to get and send messages
for _ in range(10000):
    time_usec = int(round(time.time() * 1000))
    print(time_usec)
    posx = 1+random.random()*10
    posy = 1-random.random()*10
    posz = 2+random.random()/2
    the_connection.mav.vision_position_estimate_send(time_usec, posx,posy,posz,0,0,0,[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1])  # Note, you can access message fields as attributes!
    print('VISION_POSITION_ESTIMATE sent')
    time.sleep(0.1)
