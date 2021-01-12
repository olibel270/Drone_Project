#!/usr/bin/env python

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
##

import rospy
import random
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink

# Global position of the origin
lat = 45.546659 * 1e7   # Terni
lon = -73.606107 * 1e7   # Terni
alt = 50 * 1e3

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def send_vision_position_estimate(mav, pub, tmp):
    """
    Send a mavlink VISION_POSITION_ESTIMATE message.
    """
    target_system = mav.srcSystem
    #target_system = 0   # 0 --> broadcast to everyone
    x = 1+random.random()/10
    y = 1+random.random()/10
    z = 1+random.random()/10

    if tmp==1:
        x+=1
        y+=1
        z+=1

    msg = MAV_APM.MAVLink_vision_position_estimate_message(
            0,
            x,
            y,
            z,
            0,0,0)

    send_message(msg, mav, pub)

if __name__=="__main__":
    try:
        rospy.init_node("origin_publisher")
        mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)

        # Set up mavlink instance
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

        # wait to initialize
        while mavlink_pub.get_num_connections() <= 0:
            pass
   
        for i in range(1000):
            rospy.sleep(0.1)
            if i>200:
                send_vision_position_estimate(mav, mavlink_pub,1)
            else:
                send_vision_position_estimate(mav, mavlink_pub,1)
    except rospy.ROSInterruptException:
        pass
